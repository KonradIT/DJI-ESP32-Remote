/*
 * DJI DUML frame codec — see duml.h for the frame layout.
 *
 * CRC parameters were verified against eight known-good frames from DJI Mimo
 * captures (Osmosis MEDIA_PROTOCOL.md): CRC8 seed 0x77 poly 0x8C (reflected
 * 0x31), CRC16 seed 0x3692 poly 0x8408 (reflected 0x1021, KERMIT).
 */

#include "duml.h"
#include <string.h>

uint8_t duml_crc8(const uint8_t *data, size_t len)
{
    uint8_t crc = 0x77;
    for (size_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (int b = 0; b < 8; b++) {
            crc = (crc & 1) ? (crc >> 1) ^ 0x8C : crc >> 1;
        }
    }
    return crc;
}

uint16_t duml_crc16(const uint8_t *data, size_t len)
{
    uint16_t crc = 0x3692;
    for (size_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (int b = 0; b < 8; b++) {
            crc = (crc & 1) ? (crc >> 1) ^ 0x8408 : crc >> 1;
        }
    }
    return crc;
}

size_t duml_build(uint8_t *out, size_t out_cap,
                  uint8_t dst, uint16_t seq, uint8_t cmd_type,
                  uint8_t cmd_set, uint8_t cmd_id,
                  const uint8_t *payload, size_t payload_len)
{
    return duml_build_from(out, out_cap, DUML_ADDR_APP, dst, seq, cmd_type,
                           cmd_set, cmd_id, payload, payload_len);
}

size_t duml_build_from(uint8_t *out, size_t out_cap,
                       uint8_t src, uint8_t dst, uint16_t seq, uint8_t cmd_type,
                       uint8_t cmd_set, uint8_t cmd_id,
                       const uint8_t *payload, size_t payload_len)
{
    size_t frame_len = DUML_FRAME_OVERHEAD + payload_len;
    if (frame_len > DUML_MAX_FRAME_LEN || frame_len > out_cap) {
        return 0;
    }

    out[0] = DUML_SOF;
    out[1] = (uint8_t)(frame_len & 0xFF);
    out[2] = (uint8_t)(DUML_VERSION | ((frame_len >> 8) & 0x03));
    out[3] = duml_crc8(out, 3);
    out[4] = src;
    out[5] = dst;
    /*
     * Sequence byte order. Mimo's frames read 1b cb / 1c cb / 1d cb — byte6
     * increments, byte7 stays 0xcb. That is consistent with EITHER a
     * little-endian u16 (0xcb1b, 0xcb1c…) or a big-endian one stepping by
     * 0x100 (0x1bcb, 0x1ccb…), so the capture alone cannot settle it.
     * The field is echoed verbatim by the camera, so functionally it only has
     * to round-trip — which it does either way. Kept big-endian because that
     * is what the one run which ever got a pairing reply used.
     */
    out[6] = (uint8_t)(seq >> 8);
    out[7] = (uint8_t)(seq & 0xFF);
    out[8] = cmd_type;
    out[9] = cmd_set;
    out[10] = cmd_id;
    if (payload_len > 0) {
        memcpy(&out[11], payload, payload_len);
    }

    uint16_t crc = duml_crc16(out, frame_len - 2);
    out[frame_len - 2] = (uint8_t)(crc & 0xFF);
    out[frame_len - 1] = (uint8_t)(crc >> 8);
    return frame_len;
}

size_t duml_frame_len(const uint8_t *data, size_t len)
{
    if (len < 3 || data[0] != DUML_SOF) {
        return 0;
    }
    return (size_t)data[1] | ((size_t)(data[2] & 0x03) << 8);
}

bool duml_parse(const uint8_t *data, size_t len, duml_frame_t *frame)
{
    if (len < DUML_FRAME_OVERHEAD || data[0] != DUML_SOF) {
        return false;
    }
    size_t frame_len = duml_frame_len(data, len);
    if (frame_len < DUML_FRAME_OVERHEAD || frame_len > len) {
        return false;
    }
    if (duml_crc8(data, 3) != data[3]) {
        return false;
    }
    uint16_t crc = (uint16_t)data[frame_len - 2] | ((uint16_t)data[frame_len - 1] << 8);
    if (duml_crc16(data, frame_len - 2) != crc) {
        return false;
    }

    frame->src         = data[4];
    frame->dst         = data[5];
    frame->seq         = ((uint16_t)data[6] << 8) | data[7];
    frame->cmd_type    = data[8];
    frame->cmd_set     = data[9];
    frame->cmd_id      = data[10];
    frame->payload     = &data[11];
    frame->payload_len = (uint16_t)(frame_len - DUML_FRAME_OVERHEAD);
    return true;
}
