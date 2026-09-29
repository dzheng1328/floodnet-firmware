#include <floodnet/packet.hpp>

namespace floodnet {
namespace {

void put_u16(uint8_t *p, uint16_t v) {
    p[0] = static_cast<uint8_t>(v & 0xFF);
    p[1] = static_cast<uint8_t>((v >> 8) & 0xFF);
}

void put_u32(uint8_t *p, uint32_t v) {
    p[0] = static_cast<uint8_t>(v & 0xFF);
    p[1] = static_cast<uint8_t>((v >> 8) & 0xFF);
    p[2] = static_cast<uint8_t>((v >> 16) & 0xFF);
    p[3] = static_cast<uint8_t>((v >> 24) & 0xFF);
}

uint16_t get_u16(const uint8_t *p) {
    return static_cast<uint16_t>(p[0] | (static_cast<uint16_t>(p[1]) << 8));
}

uint32_t get_u32(const uint8_t *p) {
    return static_cast<uint32_t>(p[0]) | (static_cast<uint32_t>(p[1]) << 8) |
           (static_cast<uint32_t>(p[2]) << 16) | (static_cast<uint32_t>(p[3]) << 24);
}

}  // namespace

uint16_t crc16_ccitt(const uint8_t *data, size_t len) {
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < len; ++i) {
        crc ^= static_cast<uint16_t>(data[i]) << 8;
        for (int bit = 0; bit < 8; ++bit) {
            if (crc & 0x8000) {
                crc = static_cast<uint16_t>((crc << 1) ^ 0x1021);
            } else {
                crc = static_cast<uint16_t>(crc << 1);
            }
        }
    }
    return crc;
}

size_t encode_packet(const Packet &p, uint8_t *out, size_t out_len) {
    if (out == nullptr || out_len < PACKET_SIZE) {
        return 0;
    }

    out[0] = PACKET_MAGIC;
    out[1] = PACKET_VERSION;
    put_u16(&out[2], p.node_id);
    put_u32(&out[4], p.seq);
    out[8] = p.ttl;
    out[9] = p.flags;

    put_u32(&out[10], p.record.gps.time_ms);
    put_u32(&out[14], static_cast<uint32_t>(p.record.gps.lat_1e7));
    put_u32(&out[18], static_cast<uint32_t>(p.record.gps.lon_1e7));
    put_u32(&out[22], static_cast<uint32_t>(p.record.gps.alt_mm));
    out[26] = p.record.gps.satellites;
    out[27] = p.record.gps.valid ? 1 : 0;

    put_u32(&out[28], p.record.imu.time_ms);
    put_u16(&out[32], static_cast<uint16_t>(p.record.imu.yaw_cd));
    put_u16(&out[34], static_cast<uint16_t>(p.record.imu.pitch_cd));
    put_u16(&out[36], static_cast<uint16_t>(p.record.imu.roll_cd));
    out[38] = p.record.imu.valid ? 1 : 0;

    put_u16(&out[39], p.record.diag.drops);
    put_u16(&out[41], p.record.diag.crc_errors);

    put_u16(&out[43], crc16_ccitt(out, 43));
    return PACKET_SIZE;
}

size_t encode_packet_v2(const Packet &p, uint8_t *out, size_t out_len) {
    if (out == nullptr || out_len < PACKET_SIZE) {
        return 0;
    }

    bool imu_valid = p.record.imu.valid;
    int32_t skew = 0;
    if (imu_valid) {
        // Wrap-safe for the same reason pairing is: unsigned subtraction,
        // then a signed reinterpretation.
        skew = static_cast<int32_t>(p.record.imu.time_ms - p.record.gps.time_ms);
        if (skew < INT16_MIN || skew > INT16_MAX) {
            imu_valid = false;
            skew = 0;
        }
    }

    uint8_t flags = static_cast<uint8_t>(p.flags & ~FLAG_VALIDITY_MASK);
    if (p.record.gps.valid) {
        flags = static_cast<uint8_t>(flags | FLAG_GPS_VALID);
    }
    if (imu_valid) {
        flags = static_cast<uint8_t>(flags | FLAG_IMU_VALID);
    }

    out[0] = PACKET_MAGIC;
    out[1] = PACKET_VERSION_V2;
    put_u16(&out[2], p.node_id);
    put_u32(&out[4], p.seq);
    out[8] = p.ttl;
    out[9] = flags;

    put_u32(&out[10], p.record.gps.time_ms);
    put_u32(&out[14], static_cast<uint32_t>(p.record.gps.lat_1e7));
    put_u32(&out[18], static_cast<uint32_t>(p.record.gps.lon_1e7));
    put_u32(&out[22], static_cast<uint32_t>(p.record.gps.alt_mm));
    out[26] = p.record.gps.satellites;

    put_u16(&out[27], static_cast<uint16_t>(static_cast<int16_t>(skew)));
    put_u16(&out[29], static_cast<uint16_t>(imu_valid ? p.record.imu.yaw_cd : 0));
    put_u16(&out[31], static_cast<uint16_t>(imu_valid ? p.record.imu.pitch_cd : 0));
    put_u16(&out[33], static_cast<uint16_t>(imu_valid ? p.record.imu.roll_cd : 0));

    put_u16(&out[35], p.record.diag.drops);
    put_u16(&out[37], p.boot_count);
    put_u16(&out[39], p.tx_timeouts);
    put_u16(&out[41], p.battery_mv);

    put_u16(&out[43], crc16_ccitt(out, 43));
    return PACKET_SIZE;
}

size_t encode_packet_as(WireVersion version, const Packet &p, uint8_t *out, size_t out_len) {
    return version == WireVersion::V2 ? encode_packet_v2(p, out, out_len)
                                      : encode_packet(p, out, out_len);
}

namespace {

void decode_v1(const uint8_t *in, Packet *p) {
    p->flags = in[9];

    p->record.gps.time_ms = get_u32(&in[10]);
    p->record.gps.lat_1e7 = static_cast<int32_t>(get_u32(&in[14]));
    p->record.gps.lon_1e7 = static_cast<int32_t>(get_u32(&in[18]));
    p->record.gps.alt_mm = static_cast<int32_t>(get_u32(&in[22]));
    p->record.gps.satellites = in[26];
    p->record.gps.valid = in[27] != 0;

    p->record.imu.time_ms = get_u32(&in[28]);
    p->record.imu.yaw_cd = static_cast<int16_t>(get_u16(&in[32]));
    p->record.imu.pitch_cd = static_cast<int16_t>(get_u16(&in[34]));
    p->record.imu.roll_cd = static_cast<int16_t>(get_u16(&in[36]));
    p->record.imu.valid = in[38] != 0;

    p->record.diag.drops = get_u16(&in[39]);
    p->record.diag.crc_errors = get_u16(&in[41]);
}

void decode_v2(const uint8_t *in, Packet *p) {
    p->flags = static_cast<uint8_t>(in[9] & ~FLAG_VALIDITY_MASK);

    p->record.gps.time_ms = get_u32(&in[10]);
    p->record.gps.lat_1e7 = static_cast<int32_t>(get_u32(&in[14]));
    p->record.gps.lon_1e7 = static_cast<int32_t>(get_u32(&in[18]));
    p->record.gps.alt_mm = static_cast<int32_t>(get_u32(&in[22]));
    p->record.gps.satellites = in[26];
    p->record.gps.valid = (in[9] & FLAG_GPS_VALID) != 0;

    const int16_t skew = static_cast<int16_t>(get_u16(&in[27]));
    p->record.imu.valid = (in[9] & FLAG_IMU_VALID) != 0;
    p->record.imu.time_ms =
        p->record.imu.valid
            ? p->record.gps.time_ms + static_cast<uint32_t>(static_cast<int32_t>(skew))
            : 0;
    p->record.imu.yaw_cd = static_cast<int16_t>(get_u16(&in[29]));
    p->record.imu.pitch_cd = static_cast<int16_t>(get_u16(&in[31]));
    p->record.imu.roll_cd = static_cast<int16_t>(get_u16(&in[33]));

    p->record.diag.drops = get_u16(&in[35]);
    p->record.diag.crc_errors = 0;  // not carried: always zero in every milestone
    p->boot_count = get_u16(&in[37]);
    p->tx_timeouts = get_u16(&in[39]);
    p->battery_mv = get_u16(&in[41]);
}

}  // namespace

bool decode_packet(const uint8_t *in, size_t len, Packet *out) {
    if (in == nullptr || out == nullptr || len != PACKET_SIZE) {
        return false;
    }
    if (in[0] != PACKET_MAGIC) {
        return false;
    }
    if (in[1] != PACKET_VERSION && in[1] != PACKET_VERSION_V2) {
        return false;
    }
    if (get_u16(&in[43]) != crc16_ccitt(in, 43)) {
        return false;
    }

    Packet p;
    p.node_id = get_u16(&in[2]);
    p.seq = get_u32(&in[4]);
    p.ttl = in[8];
    if (in[1] == PACKET_VERSION) {
        decode_v1(in, &p);
    } else {
        decode_v2(in, &p);
    }

    *out = p;
    return true;
}

}  // namespace floodnet
