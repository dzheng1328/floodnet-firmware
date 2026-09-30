#include <floodnet/gateway_format.hpp>

#include <stdio.h>

namespace floodnet {

namespace {
size_t written(int n, size_t out_len) {
    if (n < 0 || static_cast<size_t>(n) >= out_len) {
        return 0;
    }
    return static_cast<size_t>(n);
}
}  // namespace

size_t format_rec_line(const Packet &p, int16_t rssi, char *out, size_t out_len) {
    // Field order is the milestone 1 gateway's, with the v0x02 fields appended.
    const int n = snprintf(
        out, out_len, "REC,%u,%lu,%lu,%ld,%ld,%ld,%u,%d,%d,%d,%u,%u,%d,%d,%d,%u,%u,%u",
        static_cast<unsigned>(p.node_id), static_cast<unsigned long>(p.seq),
        static_cast<unsigned long>(p.record.gps.time_ms), static_cast<long>(p.record.gps.lat_1e7),
        static_cast<long>(p.record.gps.lon_1e7), static_cast<long>(p.record.gps.alt_mm),
        static_cast<unsigned>(p.record.gps.satellites), static_cast<int>(p.record.imu.yaw_cd),
        static_cast<int>(p.record.imu.pitch_cd), static_cast<int>(p.record.imu.roll_cd),
        static_cast<unsigned>(p.record.diag.drops), static_cast<unsigned>(p.record.diag.crc_errors),
        static_cast<int>(rssi), p.record.gps.valid ? 1 : 0, p.record.imu.valid ? 1 : 0,
        static_cast<unsigned>(p.boot_count), static_cast<unsigned>(p.tx_timeouts),
        static_cast<unsigned>(p.battery_mv));
    return written(n, out_len);
}

size_t format_decode_error_line(uint16_t count, char *out, size_t out_len) {
    return written(snprintf(out, out_len, "ERR,decode,%u", static_cast<unsigned>(count)),
                   out_len);
}

}  // namespace floodnet
