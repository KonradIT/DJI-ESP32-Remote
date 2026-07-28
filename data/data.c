/*
 * DJI Camera Remote Control - Data Management Layer
 * 
 * This file implements the data management and protocol handling layer that
 * sits between the BLE communication layer and the application logic. It manages
 * command/response matching, data queuing, and protocol-level event handling. 
 * 
 * Key Responsibilities:
 * - Command/Response Correlation: Matches responses to commands using sequence numbers
 * - Data Parsing: Processes incoming BLE data through DJI protocol parser
 * - Queue Management: Handles waiting commands and responses with timeouts
 * - Callback Dispatch: Routes parsed data to appropriate handlers
 * - Memory Management: Automatic cleanup of expired entries
 * 
 * Architecture:
 * - Thread-safe operation using FreeRTOS semaphores
 * - Asynchronous command handling with timeout support
 * - Callback-based event notification system
 * - Automatic resource cleanup to prevent memory leaks
 * 
 * Data Flow:
 * 1. Commands sent via BLE generate sequence-tracked entries
 * 2. Incoming BLE data is parsed for protocol frames
 * 3. Responses are matched to pending commands by sequence number
 * 4. Callbacks notify application layers of results
 * 5. Expired entries are automatically cleaned up
 * 
 * The data layer provides synchronous and asynchronous command execution
 * modes, allowing both blocking waits for responses and fire-and-forget
 * command transmission.
 * 
 * Hardware: M5Stack Basic V2.7 with ESP32 BLE capabilities
 * Framework: FreeRTOS with semaphores and timers
 */

#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/timers.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "esp_log.h"

#include "data.h"
#include "ble.h"
#include "duml.h"
#include "osmo_duml.h"

/* Logging tag for ESP_LOG functions */
#define TAG "DATA"

/*
 * Minimum gap between writes to the camera's 0xFFF5 characteristic.
 * 0xFFF5 is write-without-response: the Osmo Nano silently drops
 * back-to-back frames, so every TX path funnels through paced_ble_write().
 */
/*
 * Mimo's own BLE snoop spaces consecutive writes 18-40 ms apart (pairing at
 * t=0, wake at +39.4 ms), so 100 ms was over-cautious and stretched our whole
 * handshake ~3x versus the sequence the camera expects. 30 ms keeps
 * back-to-back frames from being dropped on this write-without-response
 * characteristic while still matching Mimo's cadence.
 */
#define TX_MIN_GAP_MS 30

/* Maximum number of concurrent command/response pairs that can be tracked
 * Limits memory usage and prevents resource exhaustion
 */
#define MAX_SEQ_ENTRIES 10

/* Cleanup timer interval for expired entry removal
 * Runs every 60 seconds to clean up stale command entries
 */
#define CLEANUP_INTERVAL_MS 60000

/* Maximum age for command entries before automatic cleanup
 * Entries older than 120 seconds are considered expired and removed
 */
#define MAX_ENTRY_AGE 120

/* Data layer initialization flag
 * Prevents double initialization and ensures proper setup
 */
static bool data_layer_initialized = false;

/* Command tracking entry structure
 * Stores information about pending commands waiting for responses
 */
typedef struct {
    // Whether the entry is valid
    bool in_use;

    // Added: true means based on seq, false means based on cmd_set and cmd_id
    bool is_seq_based;

    // Valid if is_seq_based is true
    uint16_t seq;

    // Valid if is_seq_based is false
    uint8_t cmd_set;

    // Valid if is_seq_based is false
    uint8_t cmd_id;

    // Generic structure after parsing
    void *parse_result;

    // Length of parsed result
    size_t parse_result_length;

    // For synchronous waiting
    SemaphoreHandle_t sem;

    // Last access timestamp for LRU policy
    TickType_t last_access_time;

    // True while a task is blocked on `sem` inside data_wait_for_result_by_*().
    // An awaiting entry must never be evicted (LRU) or reaped (cleanup timer):
    // freeing it would vSemaphoreDelete() a semaphore the waiter is blocked on
    // and free parse_result out from under it (use-after-free). The waiter
    // clears this flag and frees the entry itself, always under s_map_mutex.
    bool awaiting;
} entry_t;

/* Maintains mapping from seq to parsed results */
static entry_t s_entries[MAX_SEQ_ENTRIES];

/* Mutex to protect s_seq_entries */
static SemaphoreHandle_t s_map_mutex = NULL;

/* Timer handle */
static TimerHandle_t cleanup_timer = NULL;

/* Task handle for delayed notification processing */
static TaskHandle_t notify_task_handle = NULL;

/* Queue for notification data */
static QueueHandle_t notify_queue = NULL;

/* Structure for notification data */
typedef struct {
    int camera_index;
    uint8_t *data;
    size_t data_length;
} notify_data_t;

/* Forward declarations */
static void notify_processing_task(void *pvParameters);
static void process_notification_data(int camera_index, const uint8_t *raw_data, size_t raw_data_length);

/* Per-camera TX pacing state (see TX_MIN_GAP_MS) */
#define DATA_MAX_CAMERAS 3
static SemaphoreHandle_t s_tx_lock[DATA_MAX_CAMERAS];
static TickType_t s_last_tx_tick[DATA_MAX_CAMERAS];

/**
 * @brief Paced write to the camera's 0xFFF5 characteristic
 *
 * Serializes writes per camera and enforces a minimum inter-frame gap so
 * consecutive frames are not dropped by the write-without-response transport.
 */
static esp_err_t paced_ble_write(int camera_index, const uint8_t *data, size_t length) {
    if (camera_index < 0 || camera_index >= DATA_MAX_CAMERAS) {
        return ESP_ERR_INVALID_ARG;
    }

    /* Never write before GATT discovery has resolved the write handle: an ATT
     * write to handle 0 wedges the camera's ATT bearer and kills the link. */
    uint16_t write_handle = ble_get_write_handle(camera_index);
    if (write_handle == 0) {
        ESP_LOGW(TAG, "Camera %d: write handle not discovered yet, dropping frame", camera_index);
        return ESP_ERR_INVALID_STATE;
    }

    if (s_tx_lock[camera_index]) {
        xSemaphoreTake(s_tx_lock[camera_index], portMAX_DELAY);
    }

    TickType_t now = xTaskGetTickCount();
    TickType_t gap = now - s_last_tx_tick[camera_index];
    if (s_last_tx_tick[camera_index] != 0 && gap < pdMS_TO_TICKS(TX_MIN_GAP_MS)) {
        vTaskDelay(pdMS_TO_TICKS(TX_MIN_GAP_MS) - gap);
    }

    /*
     * DIAGNOSTIC (TX_USE_WRITE_REQUEST): 0xFFF5 is a write-without-response
     * characteristic, but a Write Command gives the peer no way to report an
     * ATT error — and the camera currently tears the link down ~1 s after our
     * first write to it. Using a Write Request makes the camera's ATT status
     * visible (e.g. insufficient authentication/encryption) in
     * on_write_complete(). Set to 0 for normal operation.
     */
#define TX_USE_WRITE_REQUEST 0
#if TX_USE_WRITE_REQUEST
    esp_err_t ret = ble_write_with_response(
        ble_get_conn_id(camera_index),
        write_handle,
        data,
        length
    );
#else
    esp_err_t ret = ble_write_without_response(
        ble_get_conn_id(camera_index),
        write_handle,
        data,
        length
    );
#endif
    s_last_tx_tick[camera_index] = xTaskGetTickCount();

    if (s_tx_lock[camera_index]) {
        xSemaphoreGive(s_tx_lock[camera_index]);
    }
    return ret;
}

/**
 * @brief Initialize seq_entries and mark all entries as unused
 */
static void reset_entries(void) {
    for (int i = 0; i < MAX_SEQ_ENTRIES; i++) {
        s_entries[i].in_use = false;
        s_entries[i].is_seq_based = false;
        s_entries[i].seq = 0;
        s_entries[i].cmd_set = 0;
        s_entries[i].cmd_id = 0;
        s_entries[i].last_access_time = 0;
        s_entries[i].awaiting = false;
        if (s_entries[i].parse_result) {
            free(s_entries[i].parse_result);
            s_entries[i].parse_result = NULL;
        }
        s_entries[i].parse_result_length = 0;
        if (s_entries[i].sem) {
            vSemaphoreDelete(s_entries[i].sem);
            s_entries[i].sem = NULL;
        }
    }
}

/**
 * @brief Find entry by sequence number
 * 
 * @param seq Sequence number to find
 * @return entry_t* Pointer to found entry, NULL if not found
 */
static entry_t* find_entry_by_seq(uint16_t seq) {
    for (int i = 0; i < MAX_SEQ_ENTRIES; i++) {
        if (s_entries[i].in_use && s_entries[i].is_seq_based && s_entries[i].seq == seq) {
            s_entries[i].last_access_time = xTaskGetTickCount();
            return &s_entries[i];
        }
    }
    return NULL;
}

/**
 * @brief Find entry by command set and ID
 * 
 * @param cmd_set Command set
 * @param cmd_id Command ID
 * @return entry_t* Pointer to found entry, NULL if not found
 */
static entry_t* find_entry_by_cmd_id(uint16_t cmd_set, uint16_t cmd_id) {
    for (int i = 0; i < MAX_SEQ_ENTRIES; i++) {
        if (s_entries[i].in_use && !s_entries[i].is_seq_based && 
            s_entries[i].cmd_set == cmd_set && s_entries[i].cmd_id == cmd_id) {
            s_entries[i].last_access_time = xTaskGetTickCount();
            return &s_entries[i];
        }
    }
    return NULL;
}

/**
 * @brief Free an entry
 * 
 * @param entry Pointer to the entry to be freed
 */
static void free_entry(entry_t *entry) {
    if (entry) {
        entry->in_use = false;
        entry->is_seq_based = false;
        entry->seq = 0;
        entry->cmd_set = 0;
        entry->cmd_id = 0;
        entry->last_access_time = 0;
        entry->awaiting = false;
        if (entry->parse_result) {
            free(entry->parse_result);
            entry->parse_result = NULL;
        }
        entry->parse_result_length = 0;
        if (entry->sem) {
            vSemaphoreDelete(entry->sem);
            entry->sem = NULL;
        }
    }
}

/**
 * @brief Allocate a free entry based on sequence number
 * 
 * @param seq Frame sequence number
 * @return entry_t* Pointer to allocated entry, NULL if failed
 */
static entry_t* allocate_entry_by_seq(uint16_t seq) {
    // First check if an entry with the same seq exists. Never reuse one that a
    // waiter is blocked on (awaiting) — freeing it would be a use-after-free.
    // With an atomic generate_seq() two in-flight entries cannot share a seq,
    // so a match here is a stale entry; the awaiting guard is defensive.
    entry_t *existing_entry = find_entry_by_seq(seq);
    if (existing_entry && !existing_entry->awaiting) {
        ESP_LOGI(TAG, "Overwriting existing entry for seq=0x%04X", seq);
        free_entry(existing_entry);
    }

    // For tracking the least recently used entry
    entry_t* oldest_entry = NULL;

    // Initialize with current time
    TickType_t oldest_access_time = xTaskGetTickCount();

    for (int i = 0; i < MAX_SEQ_ENTRIES; i++) {
        if (!s_entries[i].in_use) {
            s_entries[i].in_use = true;
            s_entries[i].is_seq_based = true;
            s_entries[i].seq = seq;
            s_entries[i].cmd_set = 0;
            s_entries[i].cmd_id = 0;
            s_entries[i].parse_result = NULL;
            s_entries[i].parse_result_length = 0;
            s_entries[i].awaiting = false;
            s_entries[i].sem = xSemaphoreCreateBinary();
            if (s_entries[i].sem == NULL) {
                ESP_LOGE(TAG, "Failed to create semaphore for seq=0x%04X", seq);
                s_entries[i].in_use = false;
                return NULL;
            }
            s_entries[i].last_access_time = xTaskGetTickCount();
            return &s_entries[i];
        }

        // Track the least recently used entry — but never evict one a waiter is
        // blocked on (awaiting), or its semaphore/result would be freed under it.
        if (!s_entries[i].awaiting && s_entries[i].last_access_time < oldest_access_time) {
            oldest_access_time = s_entries[i].last_access_time;
            oldest_entry = &s_entries[i];
        }
    }

    // If no free entry, delete the least recently used entry
    if (oldest_entry) {
        ESP_LOGW(TAG, "Deleting the least recently used entry: seq=0x%04X or cmd_set=0x%04X cmd_id=0x%04X",
                 oldest_entry->is_seq_based ? oldest_entry->seq : 0,
                 oldest_entry->cmd_set,
                 oldest_entry->cmd_id);
        free_entry(oldest_entry);
        // Reallocate
        oldest_entry->in_use = true;
        oldest_entry->is_seq_based = true;
        oldest_entry->seq = seq;
        oldest_entry->cmd_set = 0;
        oldest_entry->cmd_id = 0;
        oldest_entry->parse_result = NULL;
        oldest_entry->parse_result_length = 0;
        oldest_entry->awaiting = false;
        oldest_entry->sem = xSemaphoreCreateBinary();
        if (oldest_entry->sem == NULL) {
            ESP_LOGE(TAG, "Failed to create semaphore for seq=0x%04X", seq);
            oldest_entry->in_use = false;
            return NULL;
        }
        oldest_entry->last_access_time = xTaskGetTickCount();
        return oldest_entry;
    }

    ESP_LOGW(TAG, "No evictable entry for seq=0x%04X (all awaiting)", seq);
    return NULL;
}

/**
 * @brief Allocate a free entry based on command set and ID
 * 
 * @param cmd_set Command set
 * @param cmd_id Command ID
 * @return entry_t* Pointer to allocated entry, NULL if failed
 */
static entry_t* allocate_entry_by_cmd(uint8_t cmd_set, uint8_t cmd_id) {
    // First check if an entry with the same cmd_set and cmd_id exists
    entry_t *existing_entry = find_entry_by_cmd_id(cmd_set, cmd_id);
    if (existing_entry) {
        // Entry exists, reuse it - free old parse_result to prevent memory leak
        ESP_LOGD(TAG, "Entry for cmd_set=0x%04X cmd_id=0x%04X already exists, reusing", cmd_set, cmd_id);
        if (existing_entry->parse_result) {
            free(existing_entry->parse_result);
            existing_entry->parse_result = NULL;
            existing_entry->parse_result_length = 0;
        }
        existing_entry->last_access_time = xTaskGetTickCount();
        return existing_entry;
    }

    // Allocate new entry
    entry_t* oldest_entry = NULL;  // For tracking the least recently used non-seq-based entry
    TickType_t oldest_access_time = xTaskGetTickCount();

    for (int i = 0; i < MAX_SEQ_ENTRIES; i++) {
        if (!s_entries[i].in_use) {
            // Found a free entry
            s_entries[i].in_use = true;
            s_entries[i].is_seq_based = false;
            s_entries[i].seq = 0;
            s_entries[i].cmd_set = cmd_set;
            s_entries[i].cmd_id = cmd_id;
            s_entries[i].parse_result = NULL;
            s_entries[i].parse_result_length = 0;
            s_entries[i].awaiting = false;
            s_entries[i].sem = xSemaphoreCreateBinary();
            if (s_entries[i].sem == NULL) {
                ESP_LOGE(TAG, "Failed to create semaphore for cmd_set=0x%04X cmd_id=0x%04X", cmd_set, cmd_id);
                s_entries[i].in_use = false;
                return NULL;
            }
            s_entries[i].last_access_time = xTaskGetTickCount();
            return &s_entries[i];
        }

        // Only evict non-seq-based entries, and never one a waiter is blocked on
        if (!s_entries[i].is_seq_based && !s_entries[i].awaiting &&
            s_entries[i].last_access_time < oldest_access_time) {
            oldest_access_time = s_entries[i].last_access_time;
            oldest_entry = &s_entries[i];
        }
    }

    // If no free entry, try to delete the least recently used non-seq-based entry
    if (oldest_entry) {
        ESP_LOGW(TAG, "Deleting the least recently used cmd-based entry: cmd_set=0x%04X cmd_id=0x%04X",
                 oldest_entry->cmd_set,
                 oldest_entry->cmd_id);
        free_entry(oldest_entry);

        // Reallocate the deleted entry
        oldest_entry->in_use = true;
        oldest_entry->is_seq_based = false;
        oldest_entry->seq = 0;
        oldest_entry->cmd_set = cmd_set;
        oldest_entry->cmd_id = cmd_id;
        oldest_entry->parse_result = NULL;
        oldest_entry->parse_result_length = 0;
        oldest_entry->awaiting = false;
        oldest_entry->sem = xSemaphoreCreateBinary();
        if (oldest_entry->sem == NULL) {
            ESP_LOGE(TAG, "Failed to create semaphore for cmd_set=0x%04X cmd_id=0x%04X", cmd_set, cmd_id);
            oldest_entry->in_use = false;
            return NULL;
        }
        oldest_entry->last_access_time = xTaskGetTickCount();
        return oldest_entry;
    }

    ESP_LOGE(TAG, "No available cmd-based entry to allocate for cmd_set=0x%04X cmd_id=0x%04X", cmd_set, cmd_id);
    return NULL;
}

/**
 * @brief Timer cleanup function
 * 
 * Clean up expired entries and delete unused entries.
 * Periodically run cleanup tasks to free up memory that is no longer needed.
 * 
 * @param xTimer Timer handle that triggered this callback
 */
static void cleanup_old_entries(TimerHandle_t xTimer) {
    // Get current system tick count
    TickType_t current_time = xTaskGetTickCount();
    if (xSemaphoreTake(s_map_mutex, pdMS_TO_TICKS(100)) != pdTRUE) {
        ESP_LOGE(TAG, "Failed to take mutex in cleanup");
        return;
    }
    // Check each entry for expiration. Never reap an entry a waiter is still
    // blocked on (awaiting) — the waiter owns its lifetime.
    for (int i = 0; i < MAX_SEQ_ENTRIES; i++) {
        if (s_entries[i].in_use && !s_entries[i].awaiting &&
            (current_time - s_entries[i].last_access_time) > pdMS_TO_TICKS(MAX_ENTRY_AGE * 1000)) {
            if (s_entries[i].is_seq_based) {
                ESP_LOGI(TAG, "Cleaning up unused entry seq=0x%04X", s_entries[i].seq);
            } else {
                ESP_LOGI(TAG, "Cleaning up unused entry cmd_set=0x%04X cmd_id=0x%04X", s_entries[i].cmd_set, s_entries[i].cmd_id);
            }
            free_entry(&s_entries[i]);
        }
    }
    xSemaphoreGive(s_map_mutex);
}

/**
 * @brief Data layer initialization
 * 
 * Initialize data layer, including creating mutex, clearing entries, starting cleanup timer task, etc.
 */
void data_init(void) {
    // Initialize mutex
    s_map_mutex = xSemaphoreCreateMutex();
    if (s_map_mutex == NULL) {
        ESP_LOGE(TAG, "Failed to create mutex");
        return;
    }

    // Clear all entries
    reset_entries();

    // Per-camera TX pacing locks
    for (int i = 0; i < DATA_MAX_CAMERAS; i++) {
        s_tx_lock[i] = xSemaphoreCreateMutex();
        s_last_tx_tick[i] = 0;
        if (s_tx_lock[i] == NULL) {
            ESP_LOGE(TAG, "Failed to create TX lock for camera %d", i);
        }
    }

    // Initialize timer for cleaning up expired entries
    cleanup_timer = xTimerCreate("cleanup_timer", pdMS_TO_TICKS(CLEANUP_INTERVAL_MS), pdTRUE, NULL, cleanup_old_entries);
    if (cleanup_timer == NULL) {
        ESP_LOGE(TAG, "Failed to create cleanup timer");
    } else {
        xTimerStart(cleanup_timer, 0);
    }

    // Initialize notification queue
    notify_queue = xQueueCreate(MAX_SEQ_ENTRIES, sizeof(notify_data_t));
    if (notify_queue == NULL) {
        ESP_LOGE(TAG, "Failed to create notification queue");
    }

    // Initialize notification task
    // Increased stack size to 4096 to prevent stack overflow during notification processing
    if (xTaskCreate(notify_processing_task, "notify_processing_task", 4096, NULL, 1, &notify_task_handle) != pdPASS) {
        ESP_LOGE(TAG, "Failed to create notification processing task");
    }

    // Mark data layer as initialized
    data_layer_initialized = true;
    ESP_LOGI(TAG, "Data layer initialized successfully");
}

/**
 * @brief Check if data layer is initialized
 * 
 * @return bool Returns true if data layer is initialized, false otherwise
 */
bool is_data_layer_initialized(void) {
    return data_layer_initialized;
}

/**
 * @brief Send a DUML frame and track its sequence number for a response
 *
 * The GATT write itself is always write-without-response (0xFFF5 offers no
 * other property on the Osmo Nano); "with response" here means a seq entry
 * is kept so the caller can data_wait_for_result_by_seq() for the camera's
 * DUML-level response frame (flags 0xC0).
 *
 * @param camera_index Camera slot index (0-2)
 * @param seq Frame sequence number
 * @param raw_data Data to be sent
 * @param raw_data_length Length of data
 *
 * @return esp_err_t ESP_OK on success, error code on failure
 */
esp_err_t data_write_with_response(int camera_index, uint16_t seq, const uint8_t *raw_data, size_t raw_data_length) {
    // Validate input parameters
    if (!raw_data || raw_data_length == 0) {
        ESP_LOGE(TAG, "Invalid data or length");
        return ESP_ERR_INVALID_ARG;
    }
    if (camera_index < 0 || camera_index >= DATA_MAX_CAMERAS) {
        ESP_LOGE(TAG, "Invalid camera index: %d", camera_index);
        return ESP_ERR_INVALID_ARG;
    }

    // Take mutex for thread safety
    if (xSemaphoreTake(s_map_mutex, pdMS_TO_TICKS(100)) != pdTRUE) {
        ESP_LOGE(TAG, "Failed to take mutex");
        return ESP_ERR_INVALID_STATE;
    }

    // Allocate an entry for this sequence
    entry_t *entry = allocate_entry_by_seq(seq);
    if (!entry) {
        ESP_LOGE(TAG, "No free entry, can't write");
        xSemaphoreGive(s_map_mutex);
        return ESP_ERR_NO_MEM;
    }

    xSemaphoreGive(s_map_mutex);

    esp_err_t ret = paced_ble_write(camera_index, raw_data, raw_data_length);

    // Handle write failure
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "paced_ble_write failed: %s", esp_err_to_name(ret));
        // Clean up on failure
        if (xSemaphoreTake(s_map_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
            free_entry(entry);
            xSemaphoreGive(s_map_mutex);
        }
        return ret;
    }

    return ESP_OK;
}

/**
 * @brief Send data frame without response
 * 
 * Send data frame to device via BLE without waiting for response.
 * 
 * @param camera_index Camera slot index (0-2)
 * @param seq Frame sequence number
 * @param raw_data Data to be sent
 * @param raw_data_length Length of data
 * 
 * @return esp_err_t ESP_OK on success, error code on failure
 */
esp_err_t data_write_without_response(int camera_index, uint16_t seq, const uint8_t *raw_data, size_t raw_data_length) {
    (void)seq;   /* fire-and-forget: no response tracking */

    // Validate input parameters
    if (!raw_data || raw_data_length == 0) {
        ESP_LOGE(TAG, "Invalid raw_data or raw_data_length");
        return ESP_ERR_INVALID_ARG;
    }
    if (camera_index < 0 || camera_index >= DATA_MAX_CAMERAS) {
        ESP_LOGE(TAG, "Invalid camera index: %d", camera_index);
        return ESP_ERR_INVALID_ARG;
    }

    esp_err_t ret = paced_ble_write(camera_index, raw_data, raw_data_length);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "paced_ble_write failed: %s", esp_err_to_name(ret));
    }
    return ret;
}

/**
 * @brief Wait for parsing result of specific sequence number
 * 
 * Wait for parsing result of a specific sequence number and return to caller.
 * 
 * @param seq Frame sequence number
 * @param timeout_ms Timeout in milliseconds
 * @param out_result Return parsed result
 * @param out_result_length Return length of parsed result
 * 
 * @return esp_err_t ESP_OK on success, error code on failure
 */
esp_err_t data_wait_for_result_by_seq(uint16_t seq, int timeout_ms, void **out_result, size_t *out_result_length) {
    // Validate input parameters
    if (!out_result || !out_result_length) {
        ESP_LOGE(TAG, "out_result or out_result_length is NULL");
        return ESP_ERR_INVALID_ARG;
    }

    // Get start time and calculate timeout ticks
    TickType_t start_time = xTaskGetTickCount();
    TickType_t timeout_ticks = pdMS_TO_TICKS(timeout_ms);

    while (true) {
        // Take mutex for thread safety
        if (xSemaphoreTake(s_map_mutex, pdMS_TO_TICKS(100)) != pdTRUE) {
            ESP_LOGE(TAG, "Failed to take mutex");
            return ESP_ERR_INVALID_STATE;
        }

        // Try to find entry
        entry_t *entry = find_entry_by_seq(seq);

        if (entry) {
            // Mark as awaiting so no allocator/cleanup can free this entry (and
            // delete its semaphore) while we are blocked on it, then release the
            // mutex and block. `sem` and `entry` stay valid because awaiting
            // pins the slot.
            entry->awaiting = true;
            SemaphoreHandle_t sem = entry->sem;
            xSemaphoreGive(s_map_mutex);

            BaseType_t got = xSemaphoreTake(sem, timeout_ticks);

            // Re-take the mutex for all entry access; the notify task writes
            // parse_result under it. portMAX_DELAY so we always clear awaiting
            // and free the slot (never leak it on a transient mutex miss).
            xSemaphoreTake(s_map_mutex, portMAX_DELAY);
            entry->awaiting = false;

            if (got != pdTRUE) {
                ESP_LOGW(TAG, "Wait for seq=0x%04X timed out", seq);
                free_entry(entry);
                xSemaphoreGive(s_map_mutex);
                return ESP_ERR_TIMEOUT;
            }

            esp_err_t rc;
            if (entry->parse_result) {
                *out_result = malloc(entry->parse_result_length);
                if (*out_result != NULL) {
                    memcpy(*out_result, entry->parse_result, entry->parse_result_length);
                    *out_result_length = entry->parse_result_length;
                    rc = ESP_OK;
                } else {
                    ESP_LOGE(TAG, "Failed to allocate memory for out_result");
                    rc = ESP_ERR_NO_MEM;
                }
            } else {
                ESP_LOGE(TAG, "Parse result is NULL for seq=0x%04X", seq);
                rc = ESP_ERR_NOT_FOUND;
            }

            free_entry(entry);
            xSemaphoreGive(s_map_mutex);
            return rc;
        }

        // Check for timeout if entry not found
        TickType_t elapsed_time = xTaskGetTickCount() - start_time;
        if (elapsed_time >= timeout_ticks) {
            ESP_LOGW(TAG, "Timeout while waiting for seq=0x%04X, no entry found", seq);
            xSemaphoreGive(s_map_mutex);
            return ESP_ERR_TIMEOUT;
        }

        // Entry not found, release lock and wait before retry
        xSemaphoreGive(s_map_mutex);
        vTaskDelay(pdMS_TO_TICKS(10)); // Wait 10ms before retry
    }
}

/**
 * @brief Wait for parsing result by command set and ID, and return sequence number
 * 
 * Wait for parsing result of a specific command set and ID, and return its corresponding sequence number.
 * 
 * @param cmd_set Command set
 * @param cmd_id Command ID
 * @param timeout_ms Timeout in milliseconds
 * @param out_seq Return sequence number
 * @param out_result Return parsed result
 * @param out_result_length Return length of parsed result
 * 
 * @return esp_err_t ESP_OK on success, error code on failure
 */
esp_err_t data_wait_for_result_by_cmd(uint8_t cmd_set, uint8_t cmd_id, int timeout_ms, uint16_t *out_seq, void **out_result, size_t *out_result_length) {
    // Validate input parameters
    if (!out_result || !out_seq || !out_result_length) {
        ESP_LOGE(TAG, "out_result, out_seq or out_result_length is NULL");
        return ESP_ERR_INVALID_ARG;
    }

    // Get start time and calculate timeout ticks
    TickType_t start_time = xTaskGetTickCount();
    TickType_t timeout_ticks = pdMS_TO_TICKS(timeout_ms);

    while (true) {
        // Take mutex for thread safety
        if (xSemaphoreTake(s_map_mutex, pdMS_TO_TICKS(100)) != pdTRUE) {
            ESP_LOGE(TAG, "Failed to take mutex");
            return ESP_ERR_INVALID_STATE;
        }

        // Try to find entry
        entry_t *entry = find_entry_by_cmd_id(cmd_set, cmd_id);

        if (entry) {
            // Check if entry already has result
            if (entry->parse_result != NULL) {
                // Entry already has result, get it immediately
                *out_result = malloc(entry->parse_result_length);
                if (*out_result == NULL) {
                    ESP_LOGE(TAG, "Failed to allocate memory for out_result");
                    xSemaphoreGive(s_map_mutex);
                    return ESP_ERR_NO_MEM;
                }
                
                // Copy entry->parse_result data to out_result
                memcpy(*out_result, entry->parse_result, entry->parse_result_length);
                *out_result_length = entry->parse_result_length;
                *out_seq = entry->seq;
                
                // Free entry
                free_entry(entry);
                xSemaphoreGive(s_map_mutex);
                return ESP_OK;
            }
            
            // Entry exists but no result yet — pin it (awaiting) so no allocator
            // frees its semaphore while we block, then wait. `entry` stays valid
            // across the block because awaiting prevents eviction/cleanup.
            entry->awaiting = true;
            SemaphoreHandle_t sem_to_wait = entry->sem;
            xSemaphoreGive(s_map_mutex);

            BaseType_t got = xSemaphoreTake(sem_to_wait, timeout_ticks);

            xSemaphoreTake(s_map_mutex, portMAX_DELAY);
            entry->awaiting = false;

            if (got != pdTRUE) {
                ESP_LOGW(TAG, "Wait for cmd_set=0x%04X cmd_id=0x%04X timed out", cmd_set, cmd_id);
                free_entry(entry);
                xSemaphoreGive(s_map_mutex);
                return ESP_ERR_TIMEOUT;
            }

            esp_err_t rc;
            if (entry->parse_result) {
                *out_result = malloc(entry->parse_result_length);
                if (*out_result != NULL) {
                    memcpy(*out_result, entry->parse_result, entry->parse_result_length);
                    *out_result_length = entry->parse_result_length;
                    *out_seq = entry->seq;
                    rc = ESP_OK;
                } else {
                    ESP_LOGE(TAG, "Failed to allocate memory for out_result");
                    rc = ESP_ERR_NO_MEM;
                }
            } else {
                ESP_LOGE(TAG, "Parse result is NULL for cmd_set=0x%04X cmd_id=0x%04X", cmd_set, cmd_id);
                rc = ESP_ERR_NOT_FOUND;
            }

            free_entry(entry);
            xSemaphoreGive(s_map_mutex);
            return rc;
        }

        // Check for timeout if entry not found
        TickType_t elapsed_time = xTaskGetTickCount() - start_time;
        if (elapsed_time >= timeout_ticks) {
            ESP_LOGW(TAG, "Timeout while waiting for cmd_set=0x%04X cmd_id=0x%04X, no entry found", cmd_set, cmd_id);
            xSemaphoreGive(s_map_mutex);
            return ESP_ERR_TIMEOUT;
        }

        // Entry not found, release lock and wait before retry
        xSemaphoreGive(s_map_mutex);
        vTaskDelay(pdMS_TO_TICKS(10)); // Wait 10ms before retry
    }
}

/**
 * @brief Register camera status update callback
 * 
 * This function registers a callback function for camera status updates. After registration,
 * the callback function will be called to synchronize the latest camera status when specific notifications are received.
 * 
 * @param callback Callback function pointer, pointing to user-defined callback function
 */
static camera_status_update_cb_t status_update_callback = NULL;
void data_register_status_update_callback(camera_status_update_cb_t callback) {
    status_update_callback = callback;
}

static new_camera_status_update_cb_t new_status_update_callback = NULL;
void data_register_new_status_update_callback(new_camera_status_update_cb_t callback) {
    new_status_update_callback = callback;
}

/**
 * @brief Task for processing notification data
 * 
 * This task runs in task context and processes notification data from the queue
 * 
 * @param pvParameters Task parameters (unused)
 */
static void notify_processing_task(void *pvParameters) {
    notify_data_t notify_data;
    
    while (1) {
        // Wait for notification data from queue
        if (xQueueReceive(notify_queue, &notify_data, portMAX_DELAY) == pdTRUE) {
            // Process the notification data with camera_index
            process_notification_data(notify_data.camera_index, notify_data.data, notify_data.data_length);
            
            // Free the allocated data
            free(notify_data.data);
        }
    }
}

/**
 * @brief Is this (cmd_set, cmd_id) a status frame the status layer wants?
 */
static bool is_status_frame(uint8_t cmd_set, uint8_t cmd_id) {
    if (cmd_set == OSMO_CMDSET_CAMERA) {
        return cmd_id == OSMO_CMDID_STATUS_PUSH ||
               cmd_id == OSMO_CMDID_STATE_QUERY ||
               cmd_id == OSMO_CMDID_STATUS_POLL ||
               cmd_id == OSMO_CMDID_STORAGE_PUSH;
    }
    /* Named config pushes (cam_video_param_v2 = resolution + fps, …). These
     * only start arriving once the per-parameter subscribe is sent, which is
     * why this filter never needed the case before. */
    if (cmd_set == OSMO_CMDSET_SESSION && cmd_id == OSMO_CMDID_CFG_ITEM) {
        return true;
    }
    return cmd_set == OSMO_CMDSET_BATTERY && cmd_id == OSMO_CMDID_BATTERY_PUSH;
}

/**
 * @brief Deliver a frame's payload to the status layer as a tagged blob
 *
 * The registered callback owns the blob and must free() it.
 */
static void dispatch_status_blob(int camera_index, const duml_frame_t *frame) {
    if (!status_update_callback || !is_status_frame(frame->cmd_set, frame->cmd_id)) {
        return;
    }
    osmo_push_blob_t *blob = malloc(sizeof(osmo_push_blob_t) + frame->payload_len);
    if (blob == NULL) {
        ESP_LOGE(TAG, "Failed to allocate status blob");
        return;
    }
    blob->cmd_set = frame->cmd_set;
    blob->cmd_id = frame->cmd_id;
    blob->payload_len = frame->payload_len;
    if (frame->payload_len > 0) {
        memcpy(blob->payload, frame->payload, frame->payload_len);
    }
    status_update_callback(camera_index, blob);
}

/**
 * @brief Answer a camera-originated REQUEST frame (flags 0x40)
 *
 * The camera drops the link (~6 s) if its requests go unanswered.  Reply
 * with flags 0xC0, destination = the request's source, the request's seq,
 * and the request's payload echoed back — except the 0x00/0x81 device-info
 * exchange, which expects our "APP" identity blob.
 */
/*
 * DIAGNOSTIC: set to 0 to answer NO camera-originated requests.
 *
 * Across every capture the camera's BLE GATT wedges within ~100-400 ms of our
 * reply to its 0x00/0x81 device-info request, and the passive probe (which
 * never paired, so was never asked) survived 24 s. This isolates whether our
 * auto-ack reply is what kills it.
 */
#define AUTO_ACK_ENABLED 1

static void auto_ack_request(int camera_index, const duml_frame_t *req) {
#if !AUTO_ACK_ENABLED
    ESP_LOGW(TAG, "AUTO-ACK DISABLED: not answering request 0x%02X/0x%02X from 0x%02X",
             req->cmd_set, req->cmd_id, req->src);
    (void)camera_index;
    return;
#else
    const uint8_t *payload = req->payload;
    size_t payload_len = req->payload_len;

    if (req->cmd_set == OSMO_CMDSET_SESSION && req->cmd_id == OSMO_CMDID_DEVICE_INFO) {
        payload = OSMO_APP_DEVICE_INFO;
        payload_len = OSMO_APP_DEVICE_INFO_LEN;
    }

    /* Swap the request's addresses, exactly as the (hardware-verified) Osmosis
     * app does: the reply's source is the address the camera used to address
     * US (req->dst), not a hardcoded 0x02 — the camera talks to several of our
     * endpoints and rejects a reply that comes from the wrong one. */
    uint8_t frame_buf[DUML_MAX_FRAME_LEN];
    size_t frame_len = duml_build_from(frame_buf, sizeof(frame_buf),
                                       req->dst, req->src, req->seq, OSMO_FLAGS_RESPONSE,
                                       req->cmd_set, req->cmd_id,
                                       payload, payload_len);
    if (frame_len == 0) {
        ESP_LOGW(TAG, "Auto-ack for 0x%02X/0x%02X too large, skipped", req->cmd_set, req->cmd_id);
        return;
    }

    esp_err_t ret = paced_ble_write(camera_index, frame_buf, frame_len);
    ESP_LOGI(TAG, "Auto-acked request 0x%02X/0x%02X from 0x%02X (plen=%zu): %s",
             req->cmd_set, req->cmd_id, req->src, payload_len, esp_err_to_name(ret));
#endif /* AUTO_ACK_ENABLED */
}

/**
 * @brief Process one DUML notification frame (task context)
 *
 * Frames on 0xFFF4 are DUML (SOF 0x55).  Routing by flags byte:
 *  - responses (bit7 set): wake the seq-matched waiter; unsolicited
 *    responses to our fire-and-forget polls go to the status layer
 *  - requests (0x40): signal any cmd-based waiter (pairing approval is
 *    delivered this way), then auto-ack so the camera keeps the link up
 *  - notifies (0x00): status pushes -> status layer
 *
 * @param camera_index Index of camera sending the notification (0, 1, or 2)
 * @param raw_data Raw notification data
 * @param raw_data_length Data length
 */
/* Handle one already-parsed DUML frame. Taken by value so the body can use
 * `frame.` directly; duml_frame_t is small (the payload is a borrowed pointer
 * into the caller's receive buffer). */
static void handle_duml_frame(int camera_index, duml_frame_t frame) {
    /* cam%d matters: with two cameras connected, an unlabelled RX line cannot
     * be attributed, which made it impossible to tell which body was answering
     * (or not answering) during the Xtra investigation. */
#if DEBUG_DUML_PACKETS
    ESP_LOGI(TAG, "RX cam%d src=0x%02X dst=0x%02X flags=0x%02X cmd=0x%02X/0x%02X seq=0x%04X plen=%u",
             camera_index, frame.src, frame.dst, frame.cmd_type, frame.cmd_set,
             frame.cmd_id, frame.seq, frame.payload_len);
    if (frame.payload_len > 0) {
        ESP_LOG_BUFFER_HEX_LEVEL(TAG, frame.payload, frame.payload_len, ESP_LOG_INFO);
    }
#endif

    if (frame.cmd_type & OSMO_FLAGS_IS_ACK_BIT) {
        /* Response frame — find the waiter by seq */
        bool delivered = false;
        if (xSemaphoreTake(s_map_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
            entry_t *entry = find_entry_by_seq(frame.seq);
            if (entry) {
                /* Empty acks are represented as one 0x00 status byte so
                 * waiters always receive a non-NULL result. */
                size_t result_len = frame.payload_len > 0 ? frame.payload_len : 1;
                uint8_t *result = malloc(result_len);
                if (result) {
                    if (frame.payload_len > 0) {
                        memcpy(result, frame.payload, frame.payload_len);
                    } else {
                        result[0] = 0x00;
                    }
                    entry->parse_result = result;
                    entry->parse_result_length = result_len;
                    xSemaphoreGive(entry->sem);
                    delivered = true;
                } else {
                    ESP_LOGE(TAG, "No memory for response seq=0x%04X", frame.seq);
                }
            }
            xSemaphoreGive(s_map_mutex);
        }
        if (!delivered) {
            /* Unsolicited response (e.g. to our fire-and-forget status polls) */
            dispatch_status_blob(camera_index, &frame);
        }
        return;
    }

    if (frame.cmd_type & OSMO_FLAGS_REQUEST) {
        /* Camera-originated request. Deliver to any cmd-based waiter first
         * (pairing approval 0x07/0x46 arrives as a request), then ack it. */
        if (xSemaphoreTake(s_map_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
            entry_t *entry = allocate_entry_by_cmd(frame.cmd_set, frame.cmd_id);
            if (entry) {
                size_t result_len = frame.payload_len > 0 ? frame.payload_len : 1;
                uint8_t *result = malloc(result_len);
                if (result) {
                    if (frame.payload_len > 0) {
                        memcpy(result, frame.payload, frame.payload_len);
                    } else {
                        result[0] = 0x00;
                    }
                    entry->parse_result = result;
                    entry->parse_result_length = result_len;
                    entry->seq = frame.seq;
                    entry->last_access_time = xTaskGetTickCount();
                    xSemaphoreGive(entry->sem);
                }
            }
            xSemaphoreGive(s_map_mutex);
        }
        auto_ack_request(camera_index, &frame);
        return;
    }

    /* Notify / push frame (flags 0x00) */
    dispatch_status_blob(camera_index, &frame);
}

/**
 * @brief Process one BLE notification, which may carry several DUML frames
 *
 * The camera coalesces frames into a single notification (e.g. a pairing
 * response immediately followed by a status push). Walk the buffer frame by
 * frame instead of parsing only the first one and discarding the rest.
 */
static void process_notification_data(int camera_index, const uint8_t *raw_data, size_t raw_data_length) {
    if (!raw_data || raw_data_length < 2) {
        ESP_LOGW(TAG, "Notify data is too short or null, skip parse");
        return;
    }

    size_t offset = 0;
    while (offset + DUML_FRAME_OVERHEAD <= raw_data_length) {
        const uint8_t *p = raw_data + offset;
        size_t remaining = raw_data_length - offset;

        if (p[0] != DUML_SOF) {
            /* Not DUML (e.g. R-SDK 0xAA, which the Nano does not speak).
             * Nothing reliable to resync on, so stop. */
            ESP_LOGD(TAG, "Ignoring non-DUML data at offset %u (SOF 0x%02X)",
                     (unsigned)offset, p[0]);
            return;
        }

        size_t frame_len = duml_frame_len(p, remaining);
        duml_frame_t frame;
        if (frame_len < DUML_FRAME_OVERHEAD || frame_len > remaining ||
            !duml_parse(p, remaining, &frame)) {
            ESP_LOGW(TAG, "Invalid DUML frame at offset %u (%u bytes left), dropped",
                     (unsigned)offset, (unsigned)remaining);
            ESP_LOG_BUFFER_HEX_LEVEL(TAG, p, remaining, ESP_LOG_WARN);
            return;
        }

        handle_duml_frame(camera_index, frame);
        offset += frame_len;
    }
}

/**
 * @brief Handle camera notifications and parse data (callback function)
 * 
 * This function is called from BLE interrupt context and queues the data for processing
 * 
 * @param camera_index Index of camera sending notification (0, 1, or 2)
 * @param raw_data Raw notification data
 * @param raw_data_length Data length
 */
void receive_camera_notify_handler(int camera_index, const uint8_t *raw_data, size_t raw_data_length) {
    // Validate input parameters
    if (!raw_data || raw_data_length < 2) {
        ESP_LOGW(TAG, "Camera %d: Notify data is too short or null, skip parse", camera_index);
        return;
    }

    // Allocate memory for the data
    uint8_t *data_copy = malloc(raw_data_length);
    if (data_copy == NULL) {
        ESP_LOGE(TAG, "Camera %d: Failed to allocate memory for notification data", camera_index);
        return;
    }

    // Copy the data
    memcpy(data_copy, raw_data, raw_data_length);

    // Prepare notification data structure
    notify_data_t notify_data = {
        .camera_index = camera_index,
        .data = data_copy,
        .data_length = raw_data_length
    };

    // Send to queue for processing in task context
    if (xQueueSend(notify_queue, &notify_data, 0) != pdTRUE) {
        ESP_LOGE(TAG, "Camera %d: Failed to queue notification data", camera_index);
        free(data_copy);
    }
}