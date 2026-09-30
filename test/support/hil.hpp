#ifndef FLOODNET_TEST_HIL_HPP
#define FLOODNET_TEST_HIL_HPP

#include <math.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include <floodnet/packet.hpp>

#include "noisy_channel.hpp"

namespace floodnet {

// Frozen in the milestone 4 design doc, "Frozen sweep", before any of this ran.
// Changing any of these after seeing output needs a dated "Revisions" entry.
const double kBerPoints[] = {0.0, 1e-6, 1e-5, 1e-4, 3e-4, 1e-3, 3e-3, 1e-2};
const size_t kBerPointCount = sizeof(kBerPoints) / sizeof(kBerPoints[0]);
const uint32_t kLinkFrames = 100000;
const uint32_t kIntegrityFrames = 1000000;
const double kIntegrityBer = 1e-2;
const uint32_t kHilRunMs = 24UL * 3600000UL;

struct LinkResult {
    uint32_t frames;
    uint32_t accepted;
    uint32_t corrupted;   ///< at least one bit flipped
    uint32_t undetected;  ///< accepted although its bytes differ from what was sent
};

/// Frame `i` of a link sweep: a v0x02 packet with a valid fix and seq = i.
inline void build_link_frame(uint32_t i, uint8_t *out) {
    Packet p;
    p.node_id = 0x0042;
    p.seq = i;
    p.ttl = 3;
    p.record.gps.time_ms = i * 1000u;
    p.record.gps.lat_1e7 = 481173000;
    p.record.gps.lon_1e7 = 115166667;
    p.record.gps.alt_mm = 545400;
    p.record.gps.satellites = 8;
    p.record.gps.valid = true;
    p.boot_count = 1;
    encode_packet_as(WireVersion::V2, p, out, PACKET_SIZE);
}

/// Corruption the checks let through: the receiver accepted bytes that are
/// not the bytes that were sent.
inline bool is_undetected(const uint8_t *sent, const uint8_t *received, size_t len,
                          bool accepted) {
    return accepted && memcmp(sent, received, len) != 0;
}

inline LinkResult run_link(double p, uint32_t frames, uint64_t seed) {
    NoisyChannel channel(p, seed);
    LinkResult r = {frames, 0, 0, 0};
    uint8_t sent[PACKET_SIZE];
    uint8_t received[PACKET_SIZE];
    for (uint32_t i = 0; i < frames; ++i) {
        build_link_frame(i, sent);
        memcpy(received, sent, PACKET_SIZE);
        const size_t flips = channel.corrupt(received, PACKET_SIZE);
        Packet decoded;
        const bool accepted = decode_packet(received, PACKET_SIZE, &decoded);
        if (flips > 0) {
            ++r.corrupted;
        }
        if (accepted) {
            ++r.accepted;
        }
        if (is_undetected(sent, received, PACKET_SIZE, accepted)) {
            ++r.undetected;
        }
    }
    return r;
}

/// Probability that no bit of a frame flips.
inline double expected_ratio(double p) { return pow(1.0 - p, PACKET_SIZE * 8.0); }

/// Measured count within three standard deviations of the binomial mean.
inline bool link_pass(uint32_t accepted, uint32_t frames, double q) {
    const double n = static_cast<double>(frames);
    const double mean = n * q;
    const double sd = sqrt(n * q * (1.0 - q));
    return fabs(static_cast<double>(accepted) - mean) <= 3.0 * sd;
}

/// The count a 16-bit check with a uniformly distributed residue would pass,
/// plus three standard deviations and one. An approximation stated in advance.
inline double integrity_bound(uint32_t corrupted) {
    const double lambda = static_cast<double>(corrupted) / 65536.0;
    return lambda + 3.0 * sqrt(lambda) + 1.0;
}

inline bool integrity_pass(uint32_t undetected, uint32_t corrupted) {
    return static_cast<double>(undetected) <= integrity_bound(corrupted);
}

}  // namespace floodnet

#endif  // FLOODNET_TEST_HIL_HPP
