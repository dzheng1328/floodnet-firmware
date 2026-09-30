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

/// A link sweep through any channel with `size_t corrupt(uint8_t *, size_t)`.
template <typename Channel>
inline LinkResult run_link_with(Channel &channel, uint32_t frames) {
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

inline LinkResult run_link(double p, uint32_t frames, uint64_t seed) {
    NoisyChannel channel(p, seed);
    return run_link_with(channel, frames);
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

// Frozen in the milestone 6 design doc, "Frozen constants", before any of
// this ran. The BER points are kBerPoints without its first entry, 0.
const double kBurstLengths[] = {2.0, 4.0, 8.0, 16.0, 32.0};
const size_t kBurstLengthCount = sizeof(kBurstLengths) / sizeof(kBurstLengths[0]);
const uint32_t kBurstLinkFrames = 100000;
const uint32_t kBurstIntegrityFrames = 1000000;
const double kBurstIntegrityBer = 1e-2;
const double kBurstIntegrityLength = 32.0;

/// The span a 16-bit polynomial code is guaranteed to catch.
const size_t kMaxBurstSpan = 16;
/// Bytes 43 and 44 carry the CRC, stored little-endian by put_u16().
const size_t kCrcFieldFirstBit = 43 * 8;

/// A burst belongs to the crc_field region when any flipped bit, and so its
/// last one, lies in bytes 43 or 44.
inline bool in_crc_field(size_t last_flipped_bit) { return last_flipped_bit >= kCrcFieldFirstBit; }

/// True when `sent` with the bits set in `errors` flipped is accepted by
/// decode_packet(): corruption that got through.
inline bool pattern_undetected(const uint8_t *sent, const uint8_t *errors) {
    uint8_t received[PACKET_SIZE];
    for (size_t i = 0; i < PACKET_SIZE; ++i) {
        received[i] = static_cast<uint8_t>(sent[i] ^ errors[i]);
    }
    Packet decoded;
    const bool accepted = decode_packet(received, PACKET_SIZE, &decoded);
    return is_undetected(sent, received, PACKET_SIZE, accepted);
}

struct ExhaustiveResult {
    uint64_t data_patterns;
    uint64_t data_undetected;
    uint64_t crc_patterns;
    uint64_t crc_undetected;
};

/// Every error pattern spanning 1 to kMaxBurstSpan bits, at every position
/// in the frame, in transmission order (byte 0 first, most significant bit
/// first). For a span w >= 2 the first and last bits are set and the w - 2
/// bits between them take every value. `missed(errors, first, last)` says
/// whether the receiver let that pattern through.
template <typename Missed>
inline ExhaustiveResult for_each_short_burst(Missed missed) {
    ExhaustiveResult r = {0, 0, 0, 0};
    const size_t bits = PACKET_SIZE * 8;
    uint8_t errors[PACKET_SIZE];
    for (size_t w = 1; w <= kMaxBurstSpan; ++w) {
        const uint32_t interiors = w >= 2 ? (1u << (w - 2)) : 1u;
        for (size_t a = 0; a + w <= bits; ++a) {
            const size_t last = a + w - 1;
            const bool crc_field = in_crc_field(last);
            for (uint32_t m = 0; m < interiors; ++m) {
                memset(errors, 0, sizeof errors);
                errors[a / 8] |= static_cast<uint8_t>(0x80u >> (a % 8));
                errors[last / 8] |= static_cast<uint8_t>(0x80u >> (last % 8));
                for (size_t j = 0; j + 2 < w; ++j) {
                    if ((m >> j) & 1u) {
                        const size_t bit = a + 1 + j;
                        errors[bit / 8] |= static_cast<uint8_t>(0x80u >> (bit % 8));
                    }
                }
                const bool miss = missed(static_cast<const uint8_t *>(errors), a, last);
                if (crc_field) {
                    ++r.crc_patterns;
                    r.crc_undetected += miss ? 1 : 0;
                } else {
                    ++r.data_patterns;
                    r.data_undetected += miss ? 1 : 0;
                }
            }
        }
    }
    return r;
}

/// The exhaustive short-burst check against the real decoder.
inline ExhaustiveResult run_exhaustive_bursts(const uint8_t *sent) {
    return for_each_short_burst(
        [sent](const uint8_t *errors, size_t, size_t) { return pattern_undetected(sent, errors); });
}

}  // namespace floodnet

#endif  // FLOODNET_TEST_HIL_HPP
