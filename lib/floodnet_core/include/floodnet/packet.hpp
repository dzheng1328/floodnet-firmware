#ifndef FLOODNET_PACKET_HPP
#define FLOODNET_PACKET_HPP

#include <stddef.h>
#include <stdint.h>

#include <floodnet/sample.hpp>

namespace floodnet {

const uint8_t PACKET_MAGIC = 0xFD;
const uint8_t PACKET_VERSION = 0x01;

/// Exact on-the-wire size. Fixed, so a receiver never has to frame by length.
const size_t PACKET_SIZE = 45;

/// One transmitted observation plus the routing fields relays need.
struct Packet {
    uint16_t node_id = 0;
    uint32_t seq = 0;
    uint8_t ttl = 0;
    uint8_t flags = 0;
    SensorRecord record;
};

/// CRC16-CCITT-FALSE: poly 0x1021, init 0xFFFF, no reflection, no final XOR.
uint16_t crc16_ccitt(const uint8_t *data, size_t len);

/// Serialises `p` into `out`. Returns bytes written, or 0 if `out` is too small.
size_t encode_packet(const Packet &p, uint8_t *out, size_t out_len);

/// Validates magic, version, length and CRC, then fills `*out`.
/// Returns false and leaves `*out` untouched if any check fails.
bool decode_packet(const uint8_t *in, size_t len, Packet *out);

}  // namespace floodnet

#endif  // FLOODNET_PACKET_HPP
