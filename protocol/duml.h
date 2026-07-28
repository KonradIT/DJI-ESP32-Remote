/*
 * DJI DUML frame codec — used by the Osmo Nano (and other DUML cameras)
 * over BLE GATT: frames are written to characteristic 0xFFF5 and received
 * as notifications on 0xFFF4.
 *
 * Frame layout (verified against DJI Mimo captures, see Osmosis MEDIA_PROTOCOL.md):
 *
 *   [0]     0x55 SOF
 *   [1]     length low byte           (length = whole frame incl. CRC16)
 *   [2]     (version << 2) | length high 2 bits   — version is 1
 *   [3]     CRC8 over bytes 0..2      (poly 0x8C reflected, seed 0x77)
 *   [4]     source address            ((id << 5) | type; App = 0x02)
 *   [5]     destination address
 *   [6:8]   sequence number, BIG-endian on the BLE transport
 *   [8]     cmd_type flags            (0x00 push / 0x40 needs-ack / 0x80 is-ack)
 *   [9]     command set
 *   [10]    command id
 *   [11:-2] payload
 *   [-2:]   CRC16 over bytes 0..len-2, little-endian (poly 0x8408, seed 0x3692)
 */

#ifndef DUML_H
#define DUML_H

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

#define DUML_SOF              0x55
#define DUML_VERSION          0x04    /* byte 2 upper bits: version 1 << 2 */
#define DUML_FRAME_OVERHEAD   13      /* 11-byte header + 2-byte CRC16 */
#define DUML_MAX_FRAME_LEN    0x3FF   /* 10-bit length field */

/* Device addresses: (id << 5) | type */
#define DUML_ADDR_CAMERA      0x01    /* type 0x01, id 0 */
#define DUML_ADDR_APP         0x02    /* type 0x02, id 0 — us */
#define DUML_ADDR_GIMBAL      0x03
#define DUML_ADDR_BATTERY     0x05
#define DUML_ADDR_WIFI        0x07
#define DUML_ADDR_DM368       0x08
#define DUML_ADDR_DM368_1     0x28    /* type 0x08, id 1 — sends 0x00/0x99 config items */
#define DUML_ADDR_DM368_2     0x48    /* type 0x08, id 2 — sends 0x00/0x81 device info  */
#define DUML_ADDR_DM368_4     0x88    /* type 0x08, id 4 — target of Mimo's 0x00/0x32   */
#define DUML_ADDR_SESSION     0xF0    /* type 0x10, id 7 — session endpoint (keepalive) */
#define DUML_ADDR_SYSTEM      0x1C    /* type 0x1C, id 0 — system endpoint (wake) */

/* cmd_type flags (byte 8) */
#define DUML_CMD_NO_ACK       0x00
#define DUML_CMD_NEED_ACK     0x40
#define DUML_CMD_IS_ACK       0x80

typedef struct {
    uint8_t  src;
    uint8_t  dst;
    uint16_t seq;
    uint8_t  cmd_type;
    uint8_t  cmd_set;
    uint8_t  cmd_id;
    const uint8_t *payload;   /* points into the caller's receive buffer */
    uint16_t payload_len;
} duml_frame_t;

uint8_t  duml_crc8(const uint8_t *data, size_t len);
uint16_t duml_crc16(const uint8_t *data, size_t len);

/*
 * Build a frame into `out`. Returns the frame length, or 0 if `out_cap` is
 * too small. `payload` may be NULL when `payload_len` is 0.
 */
size_t duml_build(uint8_t *out, size_t out_cap,
                  uint8_t dst, uint16_t seq, uint8_t cmd_type,
                  uint8_t cmd_set, uint8_t cmd_id,
                  const uint8_t *payload, size_t payload_len);

/*
 * As duml_build(), but with an explicit source address instead of the default
 * DUML_ADDR_APP. Needed when answering a camera-originated request: the reply
 * must swap the request's addresses, so its source is whatever address the
 * camera used to address US (which is not always 0x02).
 */
size_t duml_build_from(uint8_t *out, size_t out_cap,
                       uint8_t src, uint8_t dst, uint16_t seq, uint8_t cmd_type,
                       uint8_t cmd_set, uint8_t cmd_id,
                       const uint8_t *payload, size_t payload_len);

/*
 * Parse and CRC-check one frame starting at data[0]. Returns true and fills
 * `frame` (payload pointing into `data`) on success. `len` may be longer than
 * the frame; use duml_frame_len() to step through a buffer of several frames.
 */
bool duml_parse(const uint8_t *data, size_t len, duml_frame_t *frame);

/*
 * Total length of the frame starting at data[0], or 0 if data doesn't start
 * with a plausible frame header (needs at least 3 bytes).
 */
size_t duml_frame_len(const uint8_t *data, size_t len);

#ifdef __cplusplus
}
#endif

#endif /* DUML_H */
