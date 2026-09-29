#ifndef FLOODNET_PACKET_HPP
#define FLOODNET_PACKET_HPP

#include <stddef.h>
#include <stdint.h>

#include <floodnet/sample.hpp>

namespace floodnet {

const uint8_t PACKET_MAGIC = 0xFD;
const uint8_t PACKET_VERSION = 0x01;
const uint8_t PACKET_VERSION_V2 = 0x02;

/// Which layout encode_packet_as() writes. decode_packet() reads either.
enum class WireVersion : uint8_t { V1 = 0x01, V2 = 0x02 };

/// v0x02 only: the low two bits of `flags` carry validity. The upper six are
/// the caller's, as every bit of `flags` is in v0x01.
const uint8_t FLAG_GPS_VALID = 0x01;
const uint8_t FLAG_IMU_VALID = 0x02;
const uint8_t FLAG_VALIDITY_MASK = 0x03;

/// Exact on-the-wire size, for both versions. Fixed, so a receiver never has
/// to frame by length. v0x02 deliberately stays at 45: at SF12 with low data
/// rate optimisation, anything from 46 to 50 bytes costs a further 262 ms of
/// airtime per packet. See the milestone 3 design doc, "Wire format v0x02".
const size_t PACKET_SIZE = 45;

/// One transmitted observation plus the routing fields relays need.
struct Packet {
    uint16_t node_id = 0;
    uint32_t seq = 0;
    uint8_t ttl = 0;
    uint8_t flags = 0;
    SensorRecord record;

    /// v0x02 only; zero when a v0x01 packet is decoded.
    uint16_t boot_count = 0;   ///< increments on every boot, survives resets
    uint16_t tx_timeouts = 0;  ///< saturating; transmissions abandoned unconfirmed
    uint16_t battery_mv = 0;
};

/// CRC16-CCITT-FALSE: poly 0x1021, init 0xFFFF, no reflection, no final XOR.
uint16_t crc16_ccitt(const uint8_t *data, size_t len);

/// Serialises `p` as v0x01. Returns bytes written, or 0 if `out` is too small.
size_t encode_packet(const Packet &p, uint8_t *out, size_t out_len);

/// Serialises `p` as v0x02. Returns bytes written, or 0 if `out` is too small.
/// An IMU sample whose time differs from the fix by more than an int16 of
/// milliseconds is sent as absent, with zeroed angles: pairing bounds a real
/// pair to 250 ms, so a wider gap was never a pair.
size_t encode_packet_v2(const Packet &p, uint8_t *out, size_t out_len);

size_t encode_packet_as(WireVersion version, const Packet &p, uint8_t *out, size_t out_len);

/// Validates magic, version (0x01 or 0x02), length and CRC, then fills `*out`.
/// Returns false and leaves `*out` untouched if any check fails.
bool decode_packet(const uint8_t *in, size_t len, Packet *out);

}  // namespace floodnet

#endif  // FLOODNET_PACKET_HPP
