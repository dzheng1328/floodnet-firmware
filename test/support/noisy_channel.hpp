#ifndef FLOODNET_TEST_NOISY_CHANNEL_HPP
#define FLOODNET_TEST_NOISY_CHANNEL_HPP

#include <math.h>
#include <stddef.h>
#include <stdint.h>

namespace floodnet {

/// Frozen in the milestone 4 design doc: ASCII "FloodNet".
const uint64_t kChannelSeed = 0x466C6F6F644E6574ULL;

/// A binary symmetric channel: every bit of a frame flips independently with
/// probability p. Seeded, so a run is reproducible bit for bit.
///
/// It draws the gap to the next flipped bit from a geometric distribution
/// instead of drawing once per bit, so a sweep of a million frames stays
/// fast. At p = 0 it neither touches the frame nor draws, so installing it
/// at p = 0 cannot perturb anything.
class NoisyChannel {
  public:
    NoisyChannel(double p, uint64_t seed) : p_(p), state_(seed), draws_(0) {}

    /// Flips bits of `frame` in place and returns how many it flipped.
    size_t corrupt(uint8_t *frame, size_t len) {
        if (p_ <= 0.0 || len == 0) {
            return 0;
        }
        const size_t bits = len * 8;
        if (p_ >= 1.0) {
            for (size_t i = 0; i < len; ++i) {
                frame[i] = static_cast<uint8_t>(~frame[i]);
            }
            return bits;
        }
        size_t flips = 0;
        size_t pos = gap();
        while (pos < bits) {
            frame[pos / 8] ^= static_cast<uint8_t>(0x80u >> (pos % 8));
            ++flips;
            pos += 1 + gap();
        }
        return flips;
    }

    uint64_t draws() const { return draws_; }
    double p() const { return p_; }

  private:
    /// splitmix64.
    uint64_t next_u64() {
        ++draws_;
        uint64_t z = (state_ += 0x9E3779B97F4A7C15ULL);
        z = (z ^ (z >> 30)) * 0xBF58476D1CE4E5B9ULL;
        z = (z ^ (z >> 27)) * 0x94D049BB133111EBULL;
        return z ^ (z >> 31);
    }

    /// Uniform on (0, 1], so log() below is always finite.
    double uniform() {
        return (static_cast<double>(next_u64() >> 11) + 1.0) * (1.0 / 9007199254740992.0);
    }

    /// Unflipped bits before the next flip: P(gap = k) = (1 - p)^k p.
    /// Capped far above any frame length so the conversion cannot overflow.
    size_t gap() {
        const double g = floor(log(uniform()) / log1p(-p_));
        return g > 1e9 ? static_cast<size_t>(1000000000) : static_cast<size_t>(g);
    }

    double p_;
    uint64_t state_;
    uint64_t draws_;
};

}  // namespace floodnet

#endif  // FLOODNET_TEST_NOISY_CHANNEL_HPP
