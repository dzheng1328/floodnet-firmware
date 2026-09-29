#ifndef FLOODNET_UBX_HPP
#define FLOODNET_UBX_HPP

#include <stddef.h>
#include <stdint.h>

namespace floodnet {

/// UBX-RXM-PMREQ, 16-byte payload form, framed.
const size_t UBX_PMREQ_BACKUP_SIZE = 24;

/// 8-bit Fletcher checksum over `data`, which is the UBX frame from the class
/// byte through the end of the payload.
void ubx_checksum(const uint8_t *data, size_t len, uint8_t *ck_a, uint8_t *ck_b);

/// Builds the frame that puts a u-blox M8 receiver into software backup
/// indefinitely, woken by activity on its UART RX line. Backup keeps ephemeris
/// in battery-backed RAM, which is what makes the next wake a 1 s hot start
/// rather than a 26 s cold one (NEO-M8 data sheet UBX-15031086, TTFF).
/// Returns bytes written, or 0 if `out` is too small.
///
/// Pure so it can be tested on the host; the Teensy GPS driver writes the
/// bytes. Whether the module on a real board honours it is unverified: no
/// hardware was available for milestone 3.
size_t ubx_pmreq_backup(uint8_t *out, size_t out_len);

}  // namespace floodnet

#endif  // FLOODNET_UBX_HPP
