/*
 * DJI Camera Remote Control - Connection Logic Layer
 * 
 * This file implements the complete connection management system for DJI camera
 * communication, handling both BLE and protocol-level connections.
 * 
 * Connection Flow:
 * 1. BLE Initialization: Set up ESP32 BLE stack
 * 2. Device Discovery: Scan for and connect to target camera
 * 3. Service Discovery: Find DJI communication characteristics
 * 4. Protocol Handshake: Establish DJI protocol connection
 * 5. Maintenance: Handle disconnections and reconnection attempts
 * 
 * State Management:
 * - BLE_NOT_INIT: Initial state before BLE initialization
 * - BLE_INIT_COMPLETE: BLE ready, no connection
 * - BLE_SEARCHING: Actively scanning for cameras
 * - BLE_CONNECTED: BLE link established, protocol pending
 * - PROTOCOL_CONNECTED: Full connection, ready for commands
 * - BLE_DISCONNECTING: Graceful disconnection in progress
 * 
 * Connection Types:
 * - Pairing (verify_mode=1): Initial camera registration
 * - Reconnection (verify_mode=0): Automatic connection to known camera
 * - Wake-up: BLE advertising to rouse sleeping cameras
 * 
 * Error Handling:
 * - Automatic reconnection attempts on unexpected disconnection
 * - Timeout management for all connection phases
 * - State restoration on connection failures
 * 
 * Hardware: M5Stack Basic V2.7 with ESP32 BLE capabilities
 * Protocol: DJI proprietary communication protocol
 */

#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

#include "ble.h"
#include "data.h"
#include "duml.h"
#include "osmo_duml.h"
#include "enums_logic.h"
#include "connect_logic.h"
#include "command_logic.h"
#include "status_logic.h"
#include "../main/ui.h"  // For camera_state_t and g_camera_states

/* Logging tag for ESP_LOG functions */
#define TAG "LOGIC_CONNECT"

/* Boot scan timeout in milliseconds */
#define BOOT_SCAN_TIMEOUT_MS 30000

/* DUML pairing token — shown verbatim on the camera screen next to Approve.
 * "DRMT" (this remote's own token) instead of "osmo", which the Osmosis app
 * already uses. */
#define OSMO_PAIRING_TOKEN "DRMT"

/*
 * Session keepalive: a sleeping/paired Osmo Nano tears the BLE link down ~5-6 s
 * after it goes quiet, so once connected we ping 0x00/0x2b [01 01] -> 0xF0 at
 * ~1 Hz for every connected slot, for the whole session.
 */
static TaskHandle_t s_keepalive_task = NULL;

/*
 * Per-slot gate: the [01 01] keepalive may only start AFTER that slot's
 * session has been opened with [04 00]. Mimo always opens the session first;
 * pinging a session that was never opened is a state the camera never sees.
 */
static volatile bool s_session_open[NUM_CAMERAS] = { false };

void connect_logic_set_session_open(int camera_index, bool open) {
    if (camera_index >= 0 && camera_index < NUM_CAMERAS) {
        s_session_open[camera_index] = open;
    }
}

static void keepalive_task(void *arg) {
    (void)arg;
    while (1) {
        for (int i = 0; i < NUM_CAMERAS; i++) {
            if (!ble_is_camera_connected(i)) {
                s_session_open[i] = false;   /* link gone: require a fresh 04 00 */
                continue;
            }
            /* Require the write handle too: ble_is_camera_connected() goes true
             * at BLE_GAP_EVENT_CONNECT, long before GATT discovery resolves the
             * handles. Pinging in that window wrote to ATT handle 0, which
             * wedged the camera's ATT bearer and killed the link. */
            if (s_session_open[i] && ble_get_write_handle(i) != 0) {
                osmo_send(i, DUML_ADDR_SESSION, OSMO_CMDSET_SESSION, OSMO_CMDID_SESSION_PING,
                          OSMO_FLAGS_REQUEST, OSMO_SESSION_KEEPALIVE, sizeof(OSMO_SESSION_KEEPALIVE));
            }
        }
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

static void start_keepalive(void) {
    if (s_keepalive_task == NULL) {
        if (xTaskCreate(keepalive_task, "osmo_keepalive", 3072, NULL, 2, &s_keepalive_task) != pdPASS) {
            ESP_LOGE(TAG, "Failed to create keepalive task");
        }
    }
}

/* Global connection state tracking
 * Manages the current state of BLE and protocol connections
 * Used throughout the system to determine available operations
 */
static connect_state_t connect_state = BLE_NOT_INIT;

/* Boot scan tracking structure
 * Manages the single boot scan for all paired camera slots
 */
static bool s_boot_scan_active = false;
static uint32_t s_boot_scan_start_tick = 0;
static bool s_boot_scan_slot_pending[3] = {false, false, false};
static bool s_boot_scan_slot_found[3] = {false, false, false};
static bool s_boot_scan_slot_not_found[3] = {false, false, false};  // Track cameras not found during boot scan

/* Boot connect phase tracking
 * True when the system is connecting to cameras found during boot scan.
 * Set to true after boot scan ends and connections begin.
 * Set to false after all boot-initiated connection attempts complete.
 */
static bool s_boot_connect_in_progress = false;

/* Per-slot connection attempt tracking
 * Tracks whether an active connection attempt is in progress for each slot
 */
static bool s_slot_is_connecting[3] = {false, false, false};

/**
 * @brief Get current connection state
 * 
 * Returns the current state of the connection system, allowing other
 * components to determine what operations are available and respond
 * appropriately to connection status changes.
 * 
 * @return connect_state_t Current connection state
 */
connect_state_t connect_logic_get_state(void) {
    return connect_state;
}

/**
 * @brief Handle camera disconnection events
 * 
 * Callback function triggered when the BLE connection to the camera is lost.
 * Implements sophisticated disconnection handling based on current state:
 * 
 * - Expected disconnections: Clean state reset
 * - Unexpected disconnections: Automatic reconnection attempt
 * - Failed reconnections: Graceful fallback to disconnected state
 * 
 * The function attempts one automatic reconnection for unexpected disconnections
 * to maintain seamless operation during temporary connection issues.
 */
void receive_camera_disconnect_handler() {
    switch (connect_state) {
        case BLE_SEARCHING:
            /* Already searching - no action needed */
            break;
        case BLE_INIT_COMPLETE:
            ESP_LOGI(TAG, "Already in DISCONNECTED state.");
            break;
        case BLE_DISCONNECTING: {
            ESP_LOGI(TAG, "Normal disconnection process.");
            /* Expected disconnection - clean state reset */
            connect_state = BLE_INIT_COMPLETE;
            camera_status_initialized = false;
            ESP_LOGI(TAG, "Current state: DISCONNECTED.");
            break;
        }
        case BLE_CONNECTED:
        case PROTOCOL_CONNECTED:
        default: {
            ESP_LOGW(TAG, "Unexpected disconnection from state: %d, attempting reconnection...", connect_state);
            
            /* Unexpected disconnection - attempt automatic reconnection */
            bool reconnected = false;
            ESP_LOGI(TAG, "Reconnection attempt...");
            // Attempt reconnection for camera 0
            if (connect_logic_ble_connect(0, SCAN_MODE_SLOT_RECONNECT) == ESP_OK) {
                /* Wait up to 30 seconds for reconnection to complete */
                for (int j = 0; j < 300; j++) {
                    if (ble_is_camera_connected(0)) {  // Check camera 0 connection
                        ESP_LOGI(TAG, "Reconnection successful");
                        reconnected = true;
                        return;  /* Successful reconnection - maintain current operation */
                    }
                    vTaskDelay(pdMS_TO_TICKS(100));
                }
            }

            if (!reconnected) {
                ESP_LOGE(TAG, "Reconnection failed after 1 attempts");
                /* Reconnection failed - perform clean disconnection */
                connect_state = BLE_INIT_COMPLETE;
                camera_status_initialized = false;
                ble_disconnect(0);  // Disconnect camera 0
                ESP_LOGI(TAG, "Current state: DISCONNECTED.");
            }
            break;
        }
    }
}

/**
 * @brief Initialize BLE subsystem for camera communication
 * 
 * Performs one-time initialization of the ESP32 BLE stack and prepares
 * the system for camera connections. This must be called before any
 * connection attempts.
 * 
 * Initialization includes:
 * - ESP32 BLE controller and host setup
 * - GATT client profile registration
 * - Service and characteristic UUID configuration
 * - Connection parameter setup
 * 
 * @return 0 on success, -1 on failure
 */
int connect_logic_ble_init() {
    esp_err_t ret;

    /* Initialize ESP32 BLE stack with DJI camera service configuration */
    ret = ble_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize BLE, error: %s", esp_err_to_name(ret));
        return -1;
    }

    connect_state = BLE_INIT_COMPLETE;

    /* Keepalive runs for the whole session; it no-ops while nothing is
     * connected and pings each slot once a link comes up. */
    start_keepalive();

    ESP_LOGI(TAG, "BLE init successfully");
    return 0;
}

/**
 * @brief Connect to BLE device
 * 
 * Execute the following steps: set callbacks, start scanning and attempt connection, wait for connection completion and characteristic handle discovery.
 * 
 * If connection fails, returns error and resets connection state.
 * 
 * @param camera_index Camera slot index (0-2)
 * @param scan_mode Scan mode to use (PAIRING, AUTOCONNECT_BOOT, or SLOT_RECONNECT)
 * @return int Returns 0 on success, -1 on failure
 */
int connect_logic_ble_connect(int camera_index, scan_mode_t scan_mode) {
    // Mark this slot as connecting
    if (camera_index >= 0 && camera_index < 3) {
        s_slot_is_connecting[camera_index] = true;
    }
    
    // Don't set BLE_SEARCHING for AUTOCONNECT_BOOT mode - it's handled by boot scan logic
    if (scan_mode != SCAN_MODE_AUTOCONNECT_BOOT) {
        connect_state = BLE_SEARCHING;
    }

    esp_err_t ret;

    /* 1. Set a global Notify callback for receiving remote data and protocol parsing */
    ble_set_notify_callback(receive_camera_notify_handler);
    ble_set_state_callback(receive_camera_disconnect_handler);

    /* 2. Start scanning with specified mode */
    ESP_LOGI(TAG, "Starting BLE scan for camera %d with mode %d", camera_index, scan_mode);
    ret = ble_start_scan(scan_mode, camera_index, 30000);  // 30 second timeout
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start scan for camera %d, error: 0x%x", camera_index, ret);
        if (camera_index >= 0 && camera_index < 3) {
            s_slot_is_connecting[camera_index] = false;
        }
        connect_state = BLE_INIT_COMPLETE;
        return -1;
    }

    /* 3. Wait up to 30 seconds to ensure BLE connection success */
    ESP_LOGI(TAG, "Waiting up to 10s for BLE to connect for camera %d...", camera_index);
    bool connected = false;
    for (int i = 0; i < 100; i++) { // 100 * 100ms = 10s
        if (ble_is_camera_connected(camera_index)) {  // Check specified camera connection
            ESP_LOGI(TAG, "BLE connected successfully to camera %d", camera_index);
            connected = true;
            break;
        }
        vTaskDelay(pdMS_TO_TICKS(100));
    }
    if (!connected) {
        ESP_LOGW(TAG, "BLE connection timed out for camera %d", camera_index);
        if (camera_index >= 0 && camera_index < 3) {
            s_slot_is_connecting[camera_index] = false;
        }
        connect_state = BLE_INIT_COMPLETE;
        return -1;
    }

    /* 4. Wait for characteristic handle discovery completion (up to 10 seconds) */
    ESP_LOGI(TAG, "Waiting up to 10s for characteristic handles discovery...");
    bool handles_found = false;
    for (int i = 0; i < 100; i++) { // 100 * 100ms = 10s
        if (ble_get_notify_handle(camera_index) != 0 &&
            ble_get_write_handle(camera_index) != 0 &&
            ble_get_cccd_handle(camera_index) != 0) {
            ESP_LOGI(TAG, "Required characteristic handles found for camera %d", camera_index);
            handles_found = true;
            break;
        }
        vTaskDelay(pdMS_TO_TICKS(100));
    }
    if (!handles_found) {
        ESP_LOGW(TAG, "Characteristic handles not found within timeout for camera %d", camera_index);
        if (camera_index >= 0 && camera_index < 3) {
            s_slot_is_connecting[camera_index] = false;
        }
        connect_state = BLE_INIT_COMPLETE;
        return -1;
    }

    /* 5. Register notification */
    ret = ble_register_notify(ble_get_conn_id(camera_index), ble_get_notify_handle(camera_index));
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to register notify for camera %d, error: %s", camera_index, esp_err_to_name(ret));
        if (camera_index >= 0 && camera_index < 3) {
            s_slot_is_connecting[camera_index] = false;
        }
        connect_state = BLE_INIT_COMPLETE;
        return -1;
    }

    // Update state to BLE connected
    connect_state = BLE_CONNECTED;

    // Delay RGB light display
    vTaskDelay(pdMS_TO_TICKS(2000));
    ESP_LOGI(TAG, "BLE connect successfully");
    return 0;
}

/**
 * @brief Connect directly to a camera without scanning (for boot scan results)
 * 
 * Used after boot scan completes to connect to cameras that were already
 * discovered during the scan. This avoids starting a new per-camera scan.
 * The target device info must already be set via ble_set_target_device().
 * 
 * @param camera_index Camera slot index (0-2)
 * @return int 0 on success, -1 on failure
 */
int connect_logic_ble_connect_direct(int camera_index) {
    if (camera_index < 0 || camera_index >= 3) {
        ESP_LOGE(TAG, "connect_logic_ble_connect_direct: Invalid camera index: %d", camera_index);
        return -1;
    }
    
    ESP_LOGI(TAG, "AUTOCONNECT_BOOT direct connection for slot %d (no new scan)", camera_index);
    
    // Mark this slot as connecting
    s_slot_is_connecting[camera_index] = true;
    
    // Note: We don't set connect_state to BLE_SEARCHING because we're not scanning
    // The boot scan has already completed

    esp_err_t ret;

    /* 1. Set global Notify callback for receiving remote data and protocol parsing */
    ble_set_notify_callback(receive_camera_notify_handler);
    ble_set_state_callback(receive_camera_disconnect_handler);

    /* 2. Initiate direct connection (no scanning) */
    ESP_LOGI(TAG, "Starting direct BLE connection for camera %d", camera_index);
    ret = ble_connect_direct(camera_index);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initiate direct connection for camera %d, error: 0x%x", camera_index, ret);
        s_slot_is_connecting[camera_index] = false;
        return -1;
    }

    /* 3. Wait up to 10 seconds to ensure BLE connection success */
    ESP_LOGI(TAG, "Waiting up to 10s for BLE direct connect for camera %d...", camera_index);
    bool connected = false;
    for (int i = 0; i < 100; i++) { // 100 * 100ms = 10s
        if (ble_is_camera_connected(camera_index)) {
            ESP_LOGI(TAG, "BLE connected successfully to camera %d (direct)", camera_index);
            connected = true;
            break;
        }
        vTaskDelay(pdMS_TO_TICKS(100));
    }
    if (!connected) {
        ESP_LOGW(TAG, "BLE direct connection timed out for camera %d", camera_index);
        s_slot_is_connecting[camera_index] = false;
        return -1;
    }

    /* 4. Wait for characteristic handle discovery completion (up to 10 seconds) */
    ESP_LOGI(TAG, "Waiting up to 10s for characteristic handles discovery...");
    bool handles_found = false;
    for (int i = 0; i < 100; i++) { // 100 * 100ms = 10s
        if (ble_get_notify_handle(camera_index) != 0 &&
            ble_get_write_handle(camera_index) != 0 &&
            ble_get_cccd_handle(camera_index) != 0) {
            ESP_LOGI(TAG, "Required characteristic handles found for camera %d", camera_index);
            handles_found = true;
            break;
        }
        vTaskDelay(pdMS_TO_TICKS(100));
    }
    if (!handles_found) {
        ESP_LOGW(TAG, "Characteristic handles not found within timeout for camera %d", camera_index);
        s_slot_is_connecting[camera_index] = false;
        return -1;
    }

    /* 5. Register notification */
    ret = ble_register_notify(ble_get_conn_id(camera_index), ble_get_notify_handle(camera_index));
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to register notify for camera %d, error: %s", camera_index, esp_err_to_name(ret));
        s_slot_is_connecting[camera_index] = false;
        return -1;
    }

    // Update state to BLE connected
    connect_state = BLE_CONNECTED;

    // Small delay for stability
    vTaskDelay(pdMS_TO_TICKS(500));
    ESP_LOGI(TAG, "BLE direct connect successfully for camera %d", camera_index);
    return 0;
}

/**
 * @brief Disconnect BLE connection
 * 
 * Attempt to disconnect from BLE device.
 * 
 * @return int Returns 0 on success, -1 on failure
 */
int connect_logic_ble_disconnect(int camera_index) {
    connect_state_t old_state = connect_state;
    connect_state = BLE_DISCONNECTING;

    ESP_LOGI(TAG, "Disconnecting camera %d...", camera_index);

    esp_err_t ret = ble_disconnect(camera_index);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to disconnect camera %d, BLE error: %s",
                 camera_index, esp_err_to_name(ret));
        connect_state = old_state;
        return -1;
    }

    ESP_LOGI(TAG, "Camera %d disconnected successfully", camera_index);
    return 0;
}

/**
 * @brief Establish DJI protocol connection with camera
 * 
 * Performs the complete DJI protocol handshake sequence over the established
 * BLE connection. This involves a complex bidirectional authentication process
 * that varies depending on whether this is a new pairing or reconnection.
 * 
 * Protocol Handshake Sequence:
 * 1. Send connection request with device credentials
 * 2. Handle camera response (may be response or command frame)
 * 3. Wait for camera's connection command with verification
 * 4. Send final connection response to complete handshake
 * 
 * Verification Modes:
 * - verify_mode=0: Reconnection to previously paired camera
 * - verify_mode=1: New pairing requiring camera-side confirmation
 * - verify_mode=2: Camera verification response
 * 
 * @param device_id Unique device identifier for this remote
 * @param mac_addr_len Length of MAC address (typically 6)
 * @param mac_addr Device MAC address for protocol identification
 * @param fw_version Firmware version for compatibility checking
 * @param verify_mode Authentication mode (0=reconnect, 1=pair)
 * @param verify_data Random verification code for security
 * @param camera_reserved Camera-specific identifier
 * @return 0 on successful protocol connection, -1 on failure
 */
int connect_logic_protocol_connect(int camera_index, uint32_t device_id, uint8_t mac_addr_len, const int8_t *mac_addr,
                                   uint32_t fw_version, uint8_t verify_mode, uint16_t verify_data,
                                   uint8_t camera_reserved) {
    /* The R-SDK handshake parameters are unused by the Osmo Nano DUML flow. */
    (void)device_id; (void)mac_addr_len; (void)mac_addr; (void)fw_version;
    (void)verify_mode; (void)verify_data; (void)camera_reserved;

    if (camera_index < 0 || camera_index >= NUM_CAMERAS) {
        return -1;
    }
    ESP_LOGI(TAG, "Camera %d: starting Osmo Nano DUML session", camera_index);

    /* Identify the camera from its BLE advertised name (OsmoNano-XXXX). */
    const char *dev_name = ble_get_connected_device_name(camera_index);
    if (dev_name && dev_name[0] != '\0') {
        strncpy(g_camera_states[camera_index].model_name, dev_name,
                sizeof(g_camera_states[camera_index].model_name) - 1);
    } else {
        strncpy(g_camera_states[camera_index].model_name, "Osmo Nano",
                sizeof(g_camera_states[camera_index].model_name) - 1);
    }
    g_camera_states[camera_index].model_name[sizeof(g_camera_states[camera_index].model_name) - 1] = '\0';

    /* STEP 1 — open the session (0x00/0x2b [04 00] -> 0xF0), before pairing.
     * Only after this may the keepalive task start its [01 01] pings. */
    osmo_send(camera_index, DUML_ADDR_SESSION, OSMO_CMDSET_SESSION, OSMO_CMDID_SESSION_PING,
              OSMO_FLAGS_REQUEST, OSMO_SESSION_OPEN, sizeof(OSMO_SESSION_OPEN));
    connect_logic_set_session_open(camera_index, true);
    vTaskDelay(pdMS_TO_TICKS(150));

    /* STEP 2 — SetPairingPIN (0x07/0x45 -> 0x07). Token shown on the camera. */
    /*
     * Fire-and-forget, exactly like Mimo: pairing is written and the NEXT
     * frame (the wake) follows ~39 ms later. Do NOT block waiting for the
     * pairing response — the camera does not always answer, and a blocking
     * wait delayed the wake past the point where the camera gives up
     * (measured: a 5 s timeout pushed the wake 3.3 s beyond the camera's
     * death). The response, when it comes, is handled asynchronously by the
     * data layer; the camera's own status stream tells us whether we are in.
     */
    uint8_t pair_payload[64];
    size_t pair_len = osmo_build_pairing_payload(pair_payload, sizeof(pair_payload), OSMO_PAIRING_TOKEN);
    bool paired = false;
    if (pair_len > 0) {
        uint16_t pair_seq = generate_seq();
        duml_write_tracked(camera_index, DUML_ADDR_WIFI, OSMO_CMDSET_WIFI, OSMO_CMDID_SET_PAIRING,
                           pair_payload, pair_len, pair_seq);

        /* Our Nano answers 0x07/0x45 at +232 ms (Mimo's phone saw +21 ms), so
         * a 35 ms leash missed it entirely and we sent the wake before the
         * camera had accepted the pairing. 800 ms covers the observed latency
         * with margin while still bounding the stall. */
        void *pin = NULL;
        size_t pin_len = 0;
        if (data_wait_for_result_by_seq(pair_seq, 800, &pin, &pin_len) == ESP_OK && pin) {
            uint8_t status = (pin_len >= 2) ? ((uint8_t *)pin)[1] : 0xFF;
            ESP_LOGI(TAG, "Camera %d: pairing status 0x%02X (%s)", camera_index, status,
                     status == 0x01 ? "already paired" :
                     status == 0x02 ? "approve on camera screen" : "unexpected");
            paired = (status == 0x01 || status == 0x02);
            free(pin);
        } else {
            ESP_LOGW(TAG, "Camera %d: no pairing reply within 800 ms, continuing", camera_index);
        }
    }

    /*
     * STEP 3 — wake / session-activate: 0x53/0x10 [00 00 00 00] -> 0x1C.
     *
     * This is REQUIRED, ~40 ms after the pairing response. Analysis of real
     * DJI Mimo BLE snoops shows the camera answers 01 00 00 00 and only then
     * enters its live session, at which point it emits 200-300 0x00/0x99
     * config-item frames. Without this the camera never enters that state and
     * powers its radio down ~600 ms later — which is exactly the failure we
     * were chasing.
     *
     * An earlier comment here claimed 0x53/0x10 tears the link down; that was
     * a mis-attribution. What actually triggers the WiFi AP hand-off (and thus
     * the BLE teardown) is the WiFi-credential fetch 0x07/0x07 + 0x07/0x0E,
     * which this firmware never sends.
     */
    /*
     * Wake at ~+40 ms after the pairing WRITE — byte-exact from the Mimo snoop
     * (pairing write t=0, its response +21 ms, wake +39.4 ms). MUST go to
     * DUML_ADDR_SYSTEM (0x1C): addressed to the camera (0x01) it answers 0xE0.
     * The camera replies 01 00 00 00 (a structured answer, not the 0xE0 error
     * stub) and its 0x00/0x99 config flood starts ~126 ms after pairing.
     */
    vTaskDelay(pdMS_TO_TICKS(5));
    osmo_send(camera_index, DUML_ADDR_SYSTEM, OSMO_CMDSET_SYSTEM, OSMO_CMDID_SYSTEM_WAKE,
              OSMO_FLAGS_REQUEST, (const uint8_t[]){0, 0, 0, 0}, 4);

    /*
     * 0x00/0x32 -> 0x88 at ~+165 ms, payload ASCII "11" + 3 zero bytes. The
     * camera answers with a 60-byte device-info block containing its serial —
     * that reply is the concrete gate proving the session is really alive.
     * (0x07/0x39 at +271 ms is deliberately NOT sent: the camera answers it
     * 0xE0 even for Mimo, so it is pure noise.)
     */
    vTaskDelay(pdMS_TO_TICKS(125));
    static const uint8_t session_info_payload[5] = { '1', '1', 0x00, 0x00, 0x00 };
    osmo_send(camera_index, DUML_ADDR_DM368_4, OSMO_CMDSET_SESSION, OSMO_CMDID_SESSION_INFO,
              OSMO_FLAGS_REQUEST, session_info_payload, sizeof(session_info_payload));

    /*
     * Subscribe to the named config parameters (0x00/0x99 -> 0x28), ONE FRAME
     * PER NAME — the form Mimo uses (verb 0x02). The camera then pushes that
     * parameter's value and every subsequent change.
     *
     * We previously sent a single group subscribe (`01 00 06 00 "camera"`).
     * The camera ACKed it with plen=0 and never sent an item, which read as
     * "this channel isn't supported" for days — it was simply malformed.
     *
     * This is how resolution/fps (camcap_video_format) are reached; they are
     * NOT in the 0x02/0x8E pid space.
     */
    for (size_t i = 0; i < OSMO_CFG_NAMES_COUNT; i++) {
        uint8_t sub[64];
        size_t n = osmo_build_cfg_subscribe(sub, sizeof(sub), (uint32_t)(0xCBD6 + i),
                                            OSMO_CFG_NAMES[i]);
        if (n == 0) {
            ESP_LOGW(TAG, "cfg subscribe: '%s' too long for buffer", OSMO_CFG_NAMES[i]);
            continue;
        }
        vTaskDelay(pdMS_TO_TICKS(60));
        osmo_send(camera_index, DUML_ADDR_DM368_1, OSMO_CMDSET_SESSION,
                  OSMO_CMDID_CFG_ITEM, OSMO_FLAGS_REQUEST, sub, n);
    }

    /* STEP 3 — the session is up; keepalive keeps it alive. */
    connect_state = PROTOCOL_CONNECTED;
    s_slot_is_connecting[camera_index] = false;

    ESP_LOGI(TAG, "Camera %d: DUML session established (paired=%d)", camera_index, paired);

    esp_err_t save_err = save_all_cameras_to_nvs();
    if (save_err != ESP_OK) {
        ESP_LOGW(TAG, "Failed to save camera %d pairing to NVS", camera_index);
    }
    return 0;
}

int connect_logic_ble_wakeup(void) {
    ESP_LOGI(TAG, "Attempting to wake up camera via BLE advertising");

    esp_err_t ret = ble_start_advertising();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start BLE advertising: %s", esp_err_to_name(ret));
        return -1;
    }

    ESP_LOGI(TAG, "BLE advertising started, attempting to wake up camera");
    return 0;
}

/**
 * @brief Start wake-up broadcast for a specific camera slot
 * 
 * Initiates a BLE advertising broadcast to wake up a sleeping camera.
 * The broadcast uses the "WKP" format with the camera's MAC address.
 * 
 * @param camera_index Camera slot index (0-2)
 * @return ESP_OK on success, ESP_ERR_* on failure
 */
esp_err_t connect_logic_start_wake_broadcast_for_slot(int camera_index) {
    if (camera_index < 0 || camera_index >= 3) {
        ESP_LOGE(TAG, "Invalid camera index: %d", camera_index);
        return ESP_ERR_INVALID_ARG;
    }
    
    // g_camera_states is already declared as extern in ui.h
    camera_state_t *cam_state = &g_camera_states[camera_index];
    
    // Check if slot is paired
    if (!cam_state->is_paired) {
        ESP_LOGE(TAG, "Camera %d: Not paired, cannot start wake broadcast", camera_index);
        return ESP_ERR_INVALID_STATE;
    }
    
    // Get camera MAC address (camera_mac is an array, not a pointer)
    const uint8_t *camera_mac = cam_state->camera_mac;
    // Check if MAC address is valid (not all zeros)
    if (camera_mac[0] == 0 && camera_mac[1] == 0 && camera_mac[2] == 0 && 
        camera_mac[3] == 0 && camera_mac[4] == 0 && camera_mac[5] == 0) {
        ESP_LOGE(TAG, "Camera %d: MAC address not available for wake broadcast", camera_index);
        return ESP_ERR_INVALID_STATE;
    }
    
    ESP_LOGI(TAG, "Starting wake broadcast for camera %d (MAC: %02X:%02X:%02X:%02X:%02X:%02X)",
             camera_index,
             camera_mac[0], camera_mac[1], camera_mac[2],
             camera_mac[3], camera_mac[4], camera_mac[5]);
    
    // Call BLE wake function with camera MAC
    esp_err_t ret = ble_wake_camera(camera_mac);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start wake broadcast for camera %d: %s", camera_index, esp_err_to_name(ret));
        return ret;
    }
    
    ESP_LOGI(TAG, "Wake broadcast started successfully for camera %d", camera_index);
    return ESP_OK;
}

/**
 * @brief Check if any camera slot is paired
 * 
 * @return true if at least one slot is paired, false otherwise
 */
static bool connect_logic_has_any_paired_slot(void) {
    for (int i = 0; i < 3; i++) {
        if (g_camera_states[i].is_paired) {
            return true;
        }
    }
    return false;
}

/**
 * @brief Reset all boot-scan tracking state
 */
static void connect_logic_reset_boot_scan_state(void) {
    s_boot_scan_active = false;
    s_boot_scan_start_tick = 0;
    s_boot_connect_in_progress = false;
    for (int i = 0; i < 3; i++) {
        s_boot_scan_slot_pending[i] = false;
        s_boot_scan_slot_found[i] = false;
        s_boot_scan_slot_not_found[i] = false;
        s_slot_is_connecting[i] = false;
    }
}

/**
 * @brief Mark a camera slot as found during boot scan
 * 
 * Called by BLE layer when a paired camera is discovered during boot scan
 * 
 * @param slot_index Camera slot index (0-2)
 */
void connect_logic_mark_slot_found(int slot_index) {
    if (slot_index < 0 || slot_index >= 3) {
        ESP_LOGW(TAG, "Invalid slot index %d in mark_slot_found", slot_index);
        return;
    }
    
    if (s_boot_scan_active && s_boot_scan_slot_pending[slot_index]) {
        s_boot_scan_slot_found[slot_index] = true;
        s_boot_scan_slot_pending[slot_index] = false;
        ESP_LOGI(TAG, "Boot scan: Slot %d marked as found", slot_index);
        
        // Trigger UI update to show found_icon for this slot
        extern ui_state_t g_ui_state;
        extern camera_state_t g_camera_states[NUM_CAMERAS];
        if (slot_index < NUM_CAMERAS) {
            g_ui_state.display_needs_update = true;
            ESP_LOGI(TAG, "Boot scan: Triggered UI update for slot %d (found state changed)", slot_index);
        }
    }
}

/**
 * @brief Check if all pending slots have been found
 * 
 * @return true if all pending slots are found, false otherwise
 */
static bool connect_logic_all_slots_found(void) {
    for (int i = 0; i < 3; i++) {
        if (s_boot_scan_slot_pending[i]) {
            return false;  // Still waiting for this slot
        }
    }
    return true;  // All pending slots found
}

/**
 * @brief Start single boot scan for all paired camera slots
 * 
 * Initiates a single BLE scan that searches for all paired cameras simultaneously.
 * Sets connect_state to BLE_SEARCHING during the scan so UI can show scanning_icon.
 * 
 * @return 0 on success, -1 on failure or no paired slots
 */
int connect_logic_start_boot_scan(void) {
    // Check if any slot is paired
    if (!connect_logic_has_any_paired_slot()) {
        ESP_LOGI(TAG, "No paired slots found, skipping boot scan");
        connect_state = BLE_INIT_COMPLETE;
        return -1;
    }
    
    // Reset all camera connection states to "paired but not connected" before boot scan
    // This ensures we start with a clean state and don't show stale connections
    // Note: Sleep state (is_sleeping, power_mode) is preserved during disconnect/reconnect.
    // The sleep state will be updated when new Camera Status Push (1D02) packets arrive.
    for (int i = 0; i < 3; i++) {
        if (g_camera_states[i].is_paired) {
            g_camera_states[i].connection_state = CAM_STATE_PAIRED_DISCONNECTED;
            g_camera_states[i].is_connected = false;
            if (g_camera_states[i].snapshot_pending) {
                ESP_LOGD(TAG, "Boot scan: Camera %d snapshot_pending cleared (reset before scan)", i);
            }
            g_camera_states[i].snapshot_pending = false;
            ESP_LOGI(TAG, "Boot scan: Reset slot %d to PAIRED_DISCONNECTED before scan", i);
        }
    }
    
    // Reset boot scan state
    connect_logic_reset_boot_scan_state();
    
    // Disconnect any stale BLE connections from previous sessions
    // This ensures we start with a clean state
    for (int i = 0; i < 3; i++) {
        if (ble_is_camera_connected(i)) {
            ESP_LOGI(TAG, "Boot scan: Disconnecting stale connection for slot %d", i);
            ble_disconnect(i);
        }
    }
    
    // Mark all paired slots as pending
    int pending_count = 0;
    for (int i = 0; i < 3; i++) {
        if (g_camera_states[i].is_paired) {
            s_boot_scan_slot_pending[i] = true;
            pending_count++;
            ESP_LOGI(TAG, "Boot scan: Slot %d marked as pending", i);
        }
    }
    
    if (pending_count == 0) {
        ESP_LOGI(TAG, "No pending slots, skipping boot scan");
        connect_state = BLE_INIT_COMPLETE;
        return -1;
    }
    
    // Set up BLE profiles with target MAC addresses for matching
    // This must be done before starting the scan so matching works
    for (int i = 0; i < 3; i++) {
        if (s_boot_scan_slot_pending[i]) {
            // Set target device info in BLE profile for this slot
            ble_set_target_device(i, g_camera_states[i].camera_name, g_camera_states[i].camera_mac);
            ESP_LOGI(TAG, "Boot scan: Set target for slot %d: %s, MAC: %02X:%02X:%02X:%02X:%02X:%02X",
                     i, g_camera_states[i].camera_name,
                     g_camera_states[i].camera_mac[0], g_camera_states[i].camera_mac[1],
                     g_camera_states[i].camera_mac[2], g_camera_states[i].camera_mac[3],
                     g_camera_states[i].camera_mac[4], g_camera_states[i].camera_mac[5]);
        }
    }
    
    // Set up autoconnect pending flags for BLE layer
    bool autoconnect_pending[BLE_MAX_CAMERAS] = {false, false, false};
    for (int i = 0; i < 3; i++) {
        autoconnect_pending[i] = s_boot_scan_slot_pending[i];
    }
    ble_set_autoconnect_pending(autoconnect_pending);
    
    // Set connect_state to BLE_SEARCHING so UI shows scanning_icon
    connect_state = BLE_SEARCHING;
    
    // Start the boot scan
    esp_err_t ret = ble_start_scan(SCAN_MODE_AUTOCONNECT_BOOT, -1, BOOT_SCAN_TIMEOUT_MS);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start boot scan: %s", esp_err_to_name(ret));
        connect_logic_reset_boot_scan_state();
        connect_state = BLE_INIT_COMPLETE;
        return -1;
    }
    
    // Record scan start time
    s_boot_scan_start_tick = xTaskGetTickCount() * portTICK_PERIOD_MS;
    s_boot_scan_active = true;
    
    ESP_LOGI(TAG, "Boot scan started for %d paired slot(s), timeout=%d ms", 
             pending_count, BOOT_SCAN_TIMEOUT_MS);
    ESP_LOGI(TAG, "connect_state set to BLE_SEARCHING - scanning_icon should be visible");
    
    // Force UI update to show scanning indicator immediately
    g_ui_state.display_needs_update = true;
    
    return 0;
}

/**
 * @brief Update boot scan state and handle timeout/connection initiation
 * 
 * This function should be called periodically (every 100-500ms) during boot scan.
 * It checks for timeout, stops the scan when all slots are found or timeout expires,
 * and initiates connections for all found slots.
 * 
 * @return 0 if scan is still active, 1 if scan completed, -1 if not active
 */
int connect_logic_update_boot_scan(void) {
    if (!s_boot_scan_active) {
        return -1;  // Boot scan not active
    }
    
    uint32_t current_tick = xTaskGetTickCount() * portTICK_PERIOD_MS;
    uint32_t elapsed_ms = current_tick - s_boot_scan_start_tick;
    
    // Check if all slots are found
    bool all_found = connect_logic_all_slots_found();
    
    // Check if timeout expired
    bool timeout_expired = (elapsed_ms >= BOOT_SCAN_TIMEOUT_MS);
    
    // Check if scan is still running
    bool scan_still_running = ble_is_scanning();
    
    // If all found or timeout expired, stop the scan
    if ((all_found || timeout_expired) && scan_still_running) {
        ESP_LOGI(TAG, "Boot scan stopping: all_found=%d, timeout=%d, elapsed=%lu ms", 
                 all_found, timeout_expired, elapsed_ms);
        ble_stop_scan();
    }
    
    // If scan has stopped (either by us or by BLE layer), process results
    if (!scan_still_running || (!all_found && timeout_expired)) {
        // Scan is complete - process results
        s_boot_scan_active = false;
        
        // Leave BLE_SEARCHING state
        connect_state = BLE_INIT_COMPLETE;
        
        // Log results and mark cameras as not found
        int found_count = 0;
        int not_found_count = 0;
        for (int i = 0; i < 3; i++) {
            if (s_boot_scan_slot_found[i]) {
                found_count++;
                ESP_LOGI(TAG, "Boot scan: Slot %d found", i);
            } else if (s_boot_scan_slot_pending[i]) {
                not_found_count++;
                s_boot_scan_slot_not_found[i] = true;  // Mark as not found during boot scan
                ESP_LOGW(TAG, "Boot scan: Slot %d not found", i);
            }
        }
        
        ESP_LOGI(TAG, "Boot scan completed: %d found, %d not found", found_count, not_found_count);
        
        // Reset connection state for cameras that were not found
        // They should be in "paired but not connected" state
        // Note: Sleep state (is_sleeping, power_mode) is preserved - only connection state is reset.
        // When camera reconnects, status push will update sleep state based on actual power_mode.
        for (int i = 0; i < 3; i++) {
            if (g_camera_states[i].is_paired && !s_boot_scan_slot_found[i]) {
                g_camera_states[i].connection_state = CAM_STATE_PAIRED_DISCONNECTED;
                g_camera_states[i].is_connected = false;
                if (g_camera_states[i].snapshot_pending) {
                    ESP_LOGD(TAG, "Boot scan: Camera %d snapshot_pending cleared (slot not found)", i);
                }
                g_camera_states[i].snapshot_pending = false;
                ESP_LOGI(TAG, "Boot scan: Slot %d not found, set to PAIRED_DISCONNECTED", i);
            }
        }
        
        // Set boot connect flag if any cameras were found
        if (found_count > 0) {
            s_boot_connect_in_progress = true;
            ESP_LOGI(TAG, "Boot connect phase starting for %d cameras", found_count);
        }
        
        // Trigger connection initiation in UI layer
        extern void ui_initiate_boot_scan_connections(void);
        ui_initiate_boot_scan_connections();
        
        return 1;  // Scan completed
    }
    
    return 0;  // Scan still active
}

/**
 * @brief Check if boot scan is currently active
 * 
 * @return true if boot scan is active, false otherwise
 */
bool connect_logic_is_boot_scan_active(void) {
    return s_boot_scan_active;
}

/**
 * @brief Check if boot-time connection phase is in progress
 * 
 * Returns true when the system is connecting to cameras found during boot scan.
 * This phase starts after the boot scan ends and lasts until all boot-initiated
 * connection attempts complete (regardless of success/failure).
 * 
 * @return true if boot connect phase is in progress, false otherwise
 */
bool connect_logic_is_boot_connect_in_progress(void) {
    return s_boot_connect_in_progress;
}

/**
 * @brief Clear the boot connect in progress flag
 * 
 * Called after all boot-initiated connection attempts have completed.
 * This allows the UI to show normal button mappings.
 */
void connect_logic_clear_boot_connect_flag(void) {
    s_boot_connect_in_progress = false;
    ESP_LOGI(TAG, "Boot connect phase completed");
}

/**
 * @brief Stop boot scan immediately and initiate connections to found cameras
 * 
 * This function stops the active boot scan and immediately begins connecting
 * to all cameras that were found during the scan. Called when user presses
 * Button C during an active boot scan.
 * 
 * Note: We do NOT mark all found cameras as "connecting" here. Instead, we let
 * ui_initiate_boot_scan_connections() set the connecting state for each camera
 * sequentially as it connects them one by one. This ensures:
 * - Only the camera currently being connected shows the connecting_icon
 * - Other found cameras keep showing the found_icon until their turn
 */
void connect_logic_stop_boot_scan_and_connect_found(void) {
    if (!s_boot_scan_active) {
        ESP_LOGI(TAG, "Boot scan not active, nothing to stop");
        return;
    }
    
    ESP_LOGI(TAG, "Stopping boot scan early (user requested)");
    
    // Stop the BLE scan immediately
    ble_stop_scan();
    
    // Mark boot scan as inactive
    s_boot_scan_active = false;
    
    // Leave BLE_SEARCHING state
    connect_state = BLE_INIT_COMPLETE;
    
    // Count found slots (for logging) but do NOT set connecting state
    // The connecting state will be set by ui_initiate_boot_scan_connections()
    // for each camera sequentially as it connects them
    int found_count = 0;
    for (int i = 0; i < 3; i++) {
        if (g_camera_states[i].is_paired && 
            s_boot_scan_slot_found[i] && 
            !g_camera_states[i].is_connected) {
            found_count++;
            ESP_LOGI(TAG, "Boot scan stop: Slot %d found, will connect", i);
        }
    }
    
    // Set boot connect flag if any cameras were found
    if (found_count > 0) {
        s_boot_connect_in_progress = true;
        ESP_LOGI(TAG, "Boot connect phase starting for %d cameras (user abort)", found_count);
    }
    
    // Initiate connections for all found slots
    // This will set connecting state for each camera one at a time
    extern void ui_initiate_boot_scan_connections(void);
    ui_initiate_boot_scan_connections();
    
    // Trigger UI refresh
    extern ui_state_t g_ui_state;
    g_ui_state.display_needs_update = true;
    
    ESP_LOGI(TAG, "Boot scan stopped, connections initiated for found cameras");
}

/**
 * @brief Get boot scan status for a specific slot
 * 
 * @param slot_index Camera slot index (0-2)
 * @return true if slot was found during boot scan, false otherwise
 */
bool connect_logic_is_slot_found(int slot_index) {
    if (slot_index < 0 || slot_index >= 3) {
        return false;
    }
    return s_boot_scan_slot_found[slot_index];
}

/**
 * @brief Check if a camera slot was not found during boot scan
 * 
 * This is used to prevent background reconnection from attempting to reconnect
 * cameras that were not available during boot scan (e.g., they were turned off).
 * 
 * @param slot_index Camera slot index (0-2)
 * @return true if slot was not found during boot scan, false otherwise
 */
bool connect_logic_was_slot_not_found_during_boot(int slot_index) {
    if (slot_index < 0 || slot_index >= 3) {
        return false;
    }
    return s_boot_scan_slot_not_found[slot_index];
}

/**
 * @brief Clear the "not found during boot" flag for a camera slot
 * 
 * This should be called when a camera is manually connected (e.g., by user action)
 * so that it can be reconnected by background reconnection if it disconnects later.
 * 
 * @param slot_index Camera slot index (0-2)
 */
void connect_logic_clear_slot_not_found_flag(int slot_index) {
    if (slot_index >= 0 && slot_index < 3) {
        s_boot_scan_slot_not_found[slot_index] = false;
        ESP_LOGI(TAG, "Cleared 'not found during boot' flag for slot %d", slot_index);
    }
}

/**
 * @brief Mark a camera slot as not found during boot scan
 * 
 * This sets the "not found during boot" flag for a camera slot, which prevents
 * background reconnection from attempting to reconnect this camera automatically.
 * The camera was not available at boot time, so it's likely turned off.
 * 
 * @param slot_index Camera slot index (0-2)
 */
void connect_logic_mark_slot_not_found_during_boot(int slot_index) {
    if (slot_index >= 0 && slot_index < 3) {
        s_boot_scan_slot_not_found[slot_index] = true;
        ESP_LOGI(TAG, "Marked slot %d as 'not found during boot'", slot_index);
    }
}

/**
 * @brief Check if a camera slot has an active connection attempt in progress
 * 
 * @param slot_index Camera slot index (0-2)
 * @return true if connection attempt is in progress, false otherwise
 */
bool connect_logic_slot_is_connecting(int slot_index) {
    if (slot_index < 0 || slot_index >= 3) {
        return false;
    }
    return s_slot_is_connecting[slot_index];
}

/**
 * @brief Set the connecting state for a camera slot
 * 
 * This function allows the UI layer to set the connecting flag before
 * calling connect_logic_ble_connect(), ensuring the UI can show the
 * connecting icon immediately.
 * 
 * @param slot_index Camera slot index (0-2)
 * @param is_connecting true to mark slot as connecting, false to clear
 */
void connect_logic_set_slot_connecting(int slot_index, bool is_connecting) {
    if (slot_index >= 0 && slot_index < 3) {
        s_slot_is_connecting[slot_index] = is_connecting;
    }
}