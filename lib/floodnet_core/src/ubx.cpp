#include <floodnet/ubx.hpp>

namespace floodnet {

void ubx_checksum(const uint8_t *data, size_t len, uint8_t *ck_a, uint8_t *ck_b) {
    uint8_t a = 0;
    uint8_t b = 0;
    for (size_t i = 0; i < len; ++i) {
        a = static_cast<uint8_t>(a + data[i]);
        b = static_cast<uint8_t>(b + a);
    }
    *ck_a = a;
    *ck_b = b;
}

size_t ubx_pmreq_backup(uint8_t *out, size_t out_len) {
    if (out == nullptr || out_len < UBX_PMREQ_BACKUP_SIZE) {
        return 0;
    }

    static const uint8_t PMREQ_CLASS = 0x02;
    static const uint8_t PMREQ_ID = 0x41;
    static const uint8_t PAYLOAD_LEN = 16;
    static const uint8_t FLAG_BACKUP = 0x02;
    static const uint8_t WAKE_UARTRX = 0x08;

    for (size_t i = 0; i < UBX_PMREQ_BACKUP_SIZE; ++i) {
        out[i] = 0;
    }
    out[0] = 0xB5;
    out[1] = 0x62;
    out[2] = PMREQ_CLASS;
    out[3] = PMREQ_ID;
    out[4] = PAYLOAD_LEN;  // little-endian u16; high byte stays 0
    // Payload starts at 6: version 0, reserved[3], duration 0 = indefinite.
    out[14] = FLAG_BACKUP;   // flags, bits 0-7
    out[18] = WAKE_UARTRX;   // wakeupSources, bits 0-7

    ubx_checksum(&out[2], 4 + PAYLOAD_LEN, &out[22], &out[23]);
    return UBX_PMREQ_BACKUP_SIZE;
}

}  // namespace floodnet
