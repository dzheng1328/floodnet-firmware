#ifndef FLOODNET_TEST_BURST_CHANNEL_HPP
#define FLOODNET_TEST_BURST_CHANNEL_HPP

#include <math.h>
#include <stddef.h>
#include <stdint.h>

namespace floodnet {

/// A two-state Gilbert channel, reset at every frame. Frozen in the
/// milestone 6 design doc, "Channel model".
///
/// In the good state no bit flips; in the bad state each bit flips with
/// probability kBadFlip. The state changes once per bit: good to bad with
/// probability s, bad to good with probability r = 1 / L. Each frame starts
/// in the bad state with the long-run probability pi_B = p / kBadFlip, so
/// every bit of every frame flips with probability exactly p, and nothing
/// carries over from one frame to the next.
///
/// Run lengths are drawn geometrically rather than stepping bit by bit, so
/// a million-frame run stays fast. At p = 0 it neither touches the frame
/// nor draws.
class BurstChannel {
  public:
    static constexpr double kBadFlip = 0.5;

    /// `p` is the average bit-error rate and `mean_burst_bits` is L, the
    /// mean bad-state run. The channel hits p only where valid() holds:
    /// L >= 1 and 0 <= p <= L / (2 (L + 1)), or p = 0.5, where every bit is
    /// in the bad state. Outside that, s would exceed 1.
    BurstChannel(double p, double mean_burst_bits, uint64_t seed)
        : p_(p),
          mean_burst_bits_(mean_burst_bits),
          pi_bad_(p / kBadFlip),
          always_bad_(pi_bad_ >= 1.0),
          leave_bad_(1.0 / mean_burst_bits),
          enter_bad_(always_bad_ ? 1.0 : leave_bad_ * pi_bad_ / (1.0 - pi_bad_)),
          state_(seed),
          draws_(0),
          frames_(0) {}

    /// Flips bits of `frame` in place and returns how many it flipped.
    size_t corrupt(uint8_t *frame, size_t len) {
        ++frames_;
        if (p_ <= 0.0 || len == 0) {
            return 0;
        }
        const size_t bits = len * 8;
        size_t flips = 0;
        size_t pos = 0;
        bool bad = always_bad_ || uniform() <= pi_bad_;
        while (pos < bits) {
            if (!bad) {
                pos += run_length(enter_bad_);
                bad = true;
                continue;
            }
            size_t end = bits;
            if (!always_bad_) {
                const size_t run = run_length(leave_bad_);
                end = run < bits - pos ? pos + run : bits;
            }
            flips += flip_half(frame, pos, end);
            pos = end;
            bad = false;
        }
        return flips;
    }

    /// Whether (p, L) lies in the domain where every bit flips with
    /// probability exactly p.
    bool valid() const {
        return p_ >= 0.0 && p_ <= 0.5 && mean_burst_bits_ >= 1.0 &&
               (always_bad_ || enter_bad_ <= 1.0);
    }

    uint64_t draws() const { return draws_; }
    /// Calls to corrupt(): frames that passed through the channel.
    uint64_t frames() const { return frames_; }
    double p() const { return p_; }

  private:
    /// splitmix64, as NoisyChannel.
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

    /// Bits spent in the current state, counting this one, when each bit
    /// leaves it with probability q: P(run = k) = (1 - q)^(k - 1) q, k >= 1.
    /// Capped far above any frame length so the conversion cannot overflow.
    size_t run_length(double q) {
        if (q >= 1.0) {
            return 1;
        }
        const double g = floor(log(uniform()) / log1p(-q));
        return g > 1e9 ? static_cast<size_t>(1000000001) : 1 + static_cast<size_t>(g);
    }

    /// Flips each bit in [from, to) with probability 1/2, one draw per 64 bits.
    size_t flip_half(uint8_t *frame, size_t from, size_t to) {
        size_t flips = 0;
        uint64_t coins = 0;
        unsigned left = 0;
        for (size_t i = from; i < to; ++i) {
            if (left == 0) {
                coins = next_u64();
                left = 64;
            }
            if (coins & 1u) {
                frame[i / 8] ^= static_cast<uint8_t>(0x80u >> (i % 8));
                ++flips;
            }
            coins >>= 1;
            --left;
        }
        return flips;
    }

    double p_;
    double mean_burst_bits_;
    double pi_bad_;
    bool always_bad_;
    double leave_bad_;
    double enter_bad_;
    uint64_t state_;
    uint64_t draws_;
    uint64_t frames_;
};

}  // namespace floodnet

#endif  // FLOODNET_TEST_BURST_CHANNEL_HPP
