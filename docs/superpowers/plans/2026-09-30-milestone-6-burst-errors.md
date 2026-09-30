# Milestone 6: Burst Bit Errors Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Test the CRC and the link under bursty bit errors within a frame, with every criterion frozen before the first run.

**Architecture:** A seeded two-state Gilbert channel (`test/support/burst_channel.hpp`) with `NoisyChannel`'s interface, a channel-generic `run_link_with()` and an exhaustive short-burst enumerator in `test/support/hil.hpp`, and a new runner `test/test_hil_burst/` that prints `BURST_*` lines.
Milestone 4's `test_hil` and its committed output are not touched.

**Tech Stack:** C++17, PlatformIO `native` env, Unity.

**Spec:** `docs/superpowers/specs/2026-09-30-milestone-6-burst-errors-design.md`

## Global Constraints

- Frozen: BER points 1e-6, 1e-5, 1e-4, 3e-4, 1e-3, 3e-3, 1e-2; *L* in {2, 4, 8, 16, 32}; `h` = 0.5; 100 000 frames per (*p*, *L*); integrity 1 000 000 frames at *p* = 1e-2, *L* = 32; seed `0x466C6F6F644E6574`.
- Exhaustive check: every span of 1 to 16 bits; `data` = every flipped bit in bits 0 to 343 (10 813 439 patterns); `crc_field` = at least one flipped bit in bits 344 to 359 (524 288 patterns); both pass only at 0 undetected.
- Integrity pass: undetected at most `integrity_bound(corrupted)` from `test/support/hil.hpp`.
- Nothing measured is tuned after it is seen; any change to a frozen value needs a dated entry in the spec's "Revisions".
- The CRC byte order is not changed, whatever `crc_field` shows.
- Milestone 4's `LINK` and `INTEGRITY` lines must stay byte-identical.
- Every harness assertion is shown to fail on a deliberately broken local edit, then the edit is reverted (project `CLAUDE.md`).
- This build of Unity has double precision disabled: compare doubles with `TEST_ASSERT_TRUE`, never `TEST_ASSERT_EQUAL_DOUBLE`.
- No em dashes anywhere. Commit messages carry no co-author trailer.

## Review Focus

- Very small *p* (1e-6): good-state runs are hundreds of thousands of bits long; the run-length draw must not overflow and a frame must never report more than 360 flips. Pinned in Task 1.
- *p* = 0.5: `pi_B` = 1 and the enter-bad probability divides by zero; the channel must treat every bit as bad. Pinned in Task 1.
- The region boundary: a burst whose last flipped bit is 343 is `data`, one whose last bit is 344 is `crc_field`. Pinned in Task 2.
- The first bit of a frame: if the initial state is not drawn from the long-run mix, bit 0's flip rate falls to near 0. Pinned by the per-position test in Task 1.
- The refactor of `run_link()` must not move milestone 4's numbers. Pinned against the committed file in Task 2.

---

### Task 1: `BurstChannel`

**Files:**
- Create: `test/support/burst_channel.hpp`
- Test: `test/test_burst_channel/test_main.cpp`

**Interfaces:**
- Consumes: `kChannelSeed` from `test/support/noisy_channel.hpp`.
- Produces: `class floodnet::BurstChannel` with `BurstChannel(double p, double mean_burst_bits, uint64_t seed)`, `size_t corrupt(uint8_t *frame, size_t len)`, `uint64_t draws() const`, `uint64_t frames() const`, `double p() const`, `static constexpr double kBadFlip = 0.5`.

- [ ] **Step 1: Write the failing tests**

Create `test/test_burst_channel/test_main.cpp`:

```cpp
#include <math.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include <unity.h>

#include "../support/burst_channel.hpp"
#include "../support/noisy_channel.hpp"

using namespace floodnet;

void setUp(void) {}
void tearDown(void) {}

static const size_t kLen = 45;
static const size_t kBits = kLen * 8;
static const uint32_t kFrames = 100000;

static void fill(uint8_t *frame) {
    for (size_t i = 0; i < kLen; ++i) {
        frame[i] = static_cast<uint8_t>(i * 37 + 11);
    }
}

// Frozen in the milestone 6 design doc, "Testing": 5% of the target.
static bool within_5_percent(double measured, double target) {
    return fabs(measured - target) <= 0.05 * target;
}

void test_zero_p_changes_nothing_and_draws_nothing(void) {
    BurstChannel ch(0.0, 8.0, kChannelSeed);
    uint8_t frame[kLen];
    uint8_t original[kLen];
    fill(frame);
    memcpy(original, frame, kLen);
    for (int i = 0; i < 1000; ++i) {
        TEST_ASSERT_EQUAL_UINT32(0, ch.corrupt(frame, kLen));
    }
    TEST_ASSERT_EQUAL_MEMORY(original, frame, kLen);
    TEST_ASSERT_EQUAL_UINT64(0, ch.draws());
    TEST_ASSERT_EQUAL_UINT64(1000, ch.frames());
}

void test_average_flip_rate_is_p(void) {
    const double p = 1e-3;
    BurstChannel ch(p, 8.0, kChannelSeed);
    uint8_t frame[kLen];
    uint64_t flips = 0;
    for (uint32_t i = 0; i < kFrames; ++i) {
        fill(frame);
        flips += ch.corrupt(frame, kLen);
    }
    const double rate = static_cast<double>(flips) / (static_cast<double>(kFrames) * kBits);
    TEST_ASSERT_TRUE_MESSAGE(within_5_percent(rate, p), "average flip rate is not p");
}

// Counts flips per bit position by comparing each corrupted frame against
// the original, so the count does not depend on corrupt()'s return value.
static void per_position_rates(double p, double L, double *rates) {
    BurstChannel ch(p, L, kChannelSeed);
    uint8_t original[kLen];
    uint8_t frame[kLen];
    fill(original);
    uint32_t counts[kBits];
    memset(counts, 0, sizeof counts);
    for (uint32_t i = 0; i < kFrames; ++i) {
        memcpy(frame, original, kLen);
        ch.corrupt(frame, kLen);
        for (size_t b = 0; b < kBits; ++b) {
            const uint8_t mask = static_cast<uint8_t>(0x80u >> (b % 8));
            if ((frame[b / 8] ^ original[b / 8]) & mask) {
                ++counts[b];
            }
        }
    }
    for (size_t b = 0; b < kBits; ++b) {
        rates[b] = static_cast<double>(counts[b]) / kFrames;
    }
}

void test_every_bit_position_flips_at_rate_p(void) {
    double rates[kBits];
    per_position_rates(0.25, 4.0, rates);
    for (size_t b = 0; b < kBits; ++b) {
        char msg[64];
        snprintf(msg, sizeof msg, "bit %u flips at %.4f", static_cast<unsigned>(b), rates[b]);
        TEST_ASSERT_TRUE_MESSAGE(within_5_percent(rates[b], 0.25), msg);
    }
}

void test_half_p_makes_every_bit_bad(void) {
    double rates[kBits];
    per_position_rates(0.5, 8.0, rates);
    for (size_t b = 0; b < kBits; ++b) {
        char msg[64];
        snprintf(msg, sizeof msg, "bit %u flips at %.4f", static_cast<unsigned>(b), rates[b]);
        TEST_ASSERT_TRUE_MESSAGE(within_5_percent(rates[b], 0.5), msg);
    }
}

void test_errors_cluster(void) {
    const double p = 1e-2;
    BurstChannel ch(p, 32.0, kChannelSeed);
    uint8_t frame[kLen];
    uint64_t corrupted = 0;
    uint64_t flips = 0;
    for (uint32_t i = 0; i < kFrames; ++i) {
        fill(frame);
        const size_t f = ch.corrupt(frame, kLen);
        if (f > 0) {
            ++corrupted;
            flips += f;
        }
    }
    const double independent_corrupted_share = 1.0 - pow(1.0 - p, static_cast<double>(kBits));
    const double independent_flips_per_corrupted = kBits * p / independent_corrupted_share;
    const double share = static_cast<double>(corrupted) / kFrames;
    const double per_corrupted = static_cast<double>(flips) / static_cast<double>(corrupted);
    TEST_ASSERT_TRUE_MESSAGE(share < independent_corrupted_share,
                             "bursts did not corrupt fewer frames");
    TEST_ASSERT_TRUE_MESSAGE(per_corrupted > independent_flips_per_corrupted,
                             "bursts did not put more flips in each corrupted frame");
}

void test_tiny_p_stays_bounded(void) {
    BurstChannel ch(1e-6, 2.0, kChannelSeed);
    uint8_t frame[kLen];
    uint64_t flips = 0;
    for (uint32_t i = 0; i < kFrames; ++i) {
        fill(frame);
        const size_t f = ch.corrupt(frame, kLen);
        TEST_ASSERT_TRUE_MESSAGE(f <= kBits, "more flips than bits");
        flips += f;
    }
    // Expected 36 flips over 36 000 000 bits; the bound only catches a
    // run length that overflowed into a huge bad run.
    TEST_ASSERT_TRUE_MESSAGE(flips > 0 && flips < 1000, "tiny p flip count out of range");
}

void test_same_seed_gives_same_flips(void) {
    BurstChannel a(1e-2, 16.0, kChannelSeed);
    BurstChannel b(1e-2, 16.0, kChannelSeed);
    uint8_t fa[kLen];
    uint8_t fb[kLen];
    for (int i = 0; i < 10000; ++i) {
        fill(fa);
        fill(fb);
        TEST_ASSERT_EQUAL_UINT32(a.corrupt(fa, kLen), b.corrupt(fb, kLen));
        TEST_ASSERT_EQUAL_MEMORY(fa, fb, kLen);
    }
    TEST_ASSERT_EQUAL_UINT64(a.draws(), b.draws());
}

void test_return_value_counts_flipped_bits(void) {
    BurstChannel ch(1e-2, 8.0, kChannelSeed);
    uint8_t original[kLen];
    uint8_t frame[kLen];
    fill(original);
    for (int i = 0; i < 10000; ++i) {
        memcpy(frame, original, kLen);
        const size_t f = ch.corrupt(frame, kLen);
        size_t diff = 0;
        for (size_t j = 0; j < kLen; ++j) {
            diff += static_cast<size_t>(__builtin_popcount(frame[j] ^ original[j]));
        }
        TEST_ASSERT_EQUAL_UINT32(diff, f);
    }
}

int main(int, char **) {
    UNITY_BEGIN();
    RUN_TEST(test_zero_p_changes_nothing_and_draws_nothing);
    RUN_TEST(test_average_flip_rate_is_p);
    RUN_TEST(test_every_bit_position_flips_at_rate_p);
    RUN_TEST(test_half_p_makes_every_bit_bad);
    RUN_TEST(test_errors_cluster);
    RUN_TEST(test_tiny_p_stays_bounded);
    RUN_TEST(test_same_seed_gives_same_flips);
    RUN_TEST(test_return_value_counts_flipped_bits);
    return UNITY_END();
}
```

- [ ] **Step 2: Run to verify it fails**

Run: `pio test -e native -f test_burst_channel`
Expected: build FAILS with `'../support/burst_channel.hpp' file not found`.

- [ ] **Step 3: Implement**

Create `test/support/burst_channel.hpp`:

```cpp
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

    /// `p` in [0, 0.5], the average bit-error rate; at 0.5 every bit is in
    /// the bad state. `mean_burst_bits` is L, the mean bad-state run, >= 1.
    BurstChannel(double p, double mean_burst_bits, uint64_t seed)
        : p_(p),
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
```

`flip_half` uses one coin per bit and `kBadFlip` is fixed at 0.5 by the spec; if `kBadFlip` ever changes, `flip_half` must change with it.

- [ ] **Step 4: Run to verify it passes**

Run: `pio test -e native -f test_burst_channel`
Expected: 8 tests PASS.

- [ ] **Step 5: Deliberate-break checks**

Make each edit, run `pio test -e native -f test_burst_channel`, confirm the named test fails, then revert with `git checkout test/support/burst_channel.hpp`:

1. `bool bad = always_bad_ || uniform() <= pi_bad_;` to `bool bad = always_bad_;` (always start good): `test_every_bit_position_flips_at_rate_p` fails at bit 0.
2. `1 + static_cast<size_t>(g)` to `static_cast<size_t>(g) + 2` (off-by-one run length): `test_every_bit_position_flips_at_rate_p` or `test_average_flip_rate_is_p` fails.
3. Change `if (!always_bad_) {` in `corrupt()` to `if (true) {`, so an always-bad channel still ends its bad runs and takes one-bit good runs: `test_half_p_makes_every_bit_bad` fails, with rates near 0.44.

Known gap, stated rather than papered over: no test here pins the bit order within a byte. Swapping `0x80u >> (i % 8)` for `0x01u << (i % 8)` passes every test, because each position's rate is the same either way. The expression is copied verbatim from `NoisyChannel::corrupt()`, and the README states that the order is shared by inspection, not by a test.

- [ ] **Step 6: Commit**

```bash
git add test/support/burst_channel.hpp test/test_burst_channel/test_main.cpp
git commit -m "test: add a seeded two-state burst channel for milestone 6"
```

---

### Task 2: channel-generic link runs and the exhaustive short-burst check

**Files:**
- Modify: `test/support/hil.hpp` (the `run_link()` function, and new declarations after `integrity_pass()`)
- Test: `test/test_hil_criteria/test_main.cpp`

**Interfaces:**
- Consumes: `NoisyChannel` (`noisy_channel.hpp`), `decode_packet()`, `PACKET_SIZE`, `is_undetected()`, `build_link_frame()`.
- Produces:
  - `template <typename Channel> LinkResult run_link_with(Channel &channel, uint32_t frames)`
  - `const size_t kMaxBurstSpan = 16;`, `const size_t kCrcFieldFirstBit = 43 * 8;`
  - `inline bool in_crc_field(size_t last_flipped_bit)`
  - `inline bool pattern_undetected(const uint8_t *sent, const uint8_t *errors)`
  - `struct ExhaustiveResult { uint64_t data_patterns; uint64_t data_undetected; uint64_t crc_patterns; uint64_t crc_undetected; };`
  - `inline ExhaustiveResult run_exhaustive_bursts(const uint8_t *sent)`
  - Frozen constants `kBurstLengths[]`, `kBurstLengthCount`, `kBurstLinkFrames`, `kBurstIntegrityFrames`, `kBurstIntegrityBer`, `kBurstIntegrityLength`.

- [ ] **Step 1: Write the failing tests**

Add to `test/test_hil_criteria/test_main.cpp`, before `int main`:

```cpp
// Pinned to docs/results/milestone-4-hil-regression.txt: the refactor of
// run_link() must not move milestone 4's numbers.
void test_run_link_reproduces_milestone_4_rows(void) {
    const LinkResult a = run_link(1e-3, kLinkFrames, kChannelSeed);
    TEST_ASSERT_EQUAL_UINT32(69617, a.accepted);
    TEST_ASSERT_EQUAL_UINT32(0, a.undetected);
    const LinkResult b = run_link(1e-2, kLinkFrames, kChannelSeed);
    TEST_ASSERT_EQUAL_UINT32(2741, b.accepted);
    TEST_ASSERT_EQUAL_UINT32(2, b.undetected);
}

// x^16 + x^12 + x^5 + 1 placed in the data bytes is a multiple of the CRC
// polynomial, so the CRC cannot see it.
void test_crc_polynomial_pattern_is_undetected(void) {
    uint8_t sent[PACKET_SIZE];
    build_link_frame(0, sent);
    uint8_t errors[PACKET_SIZE];
    memset(errors, 0, sizeof errors);
    const size_t start = 100;
    const size_t offsets[] = {0, 4, 11, 16};
    for (size_t i = 0; i < 4; ++i) {
        const size_t bit = start + offsets[i];
        errors[bit / 8] |= static_cast<uint8_t>(0x80u >> (bit % 8));
    }
    TEST_ASSERT_TRUE(pattern_undetected(sent, errors));
}

void test_single_bit_pattern_is_detected(void) {
    uint8_t sent[PACKET_SIZE];
    build_link_frame(0, sent);
    uint8_t errors[PACKET_SIZE];
    memset(errors, 0, sizeof errors);
    errors[25] = 0x10;
    TEST_ASSERT_FALSE(pattern_undetected(sent, errors));
}

void test_crc_field_starts_at_byte_43(void) {
    TEST_ASSERT_EQUAL_UINT32(344, kCrcFieldFirstBit);
    TEST_ASSERT_FALSE(in_crc_field(343));
    TEST_ASSERT_TRUE(in_crc_field(344));
    TEST_ASSERT_TRUE(in_crc_field(359));
}
```

And register them in `main`:

```cpp
    RUN_TEST(test_run_link_reproduces_milestone_4_rows);
    RUN_TEST(test_crc_polynomial_pattern_is_undetected);
    RUN_TEST(test_single_bit_pattern_is_detected);
    RUN_TEST(test_crc_field_starts_at_byte_43);
```

- [ ] **Step 2: Run to verify it fails**

Run: `pio test -e native -f test_hil_criteria`
Expected: build FAILS on `pattern_undetected`, `kCrcFieldFirstBit` and `in_crc_field` not declared.

- [ ] **Step 3: Implement**

In `test/support/hil.hpp`, replace the whole `run_link()` function with:

```cpp
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
```

After `integrity_pass()`, add:

```cpp
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
/// bits between them take every value.
inline ExhaustiveResult run_exhaustive_bursts(const uint8_t *sent) {
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
                const bool missed = pattern_undetected(sent, errors);
                if (crc_field) {
                    ++r.crc_patterns;
                    r.crc_undetected += missed ? 1 : 0;
                } else {
                    ++r.data_patterns;
                    r.data_undetected += missed ? 1 : 0;
                }
            }
        }
    }
    return r;
}
```

The interior loop bound `j + 2 < w` gives `w - 2` interior bits.

- [ ] **Step 4: Run to verify it passes**

Run: `pio test -e native -f test_hil_criteria`
Expected: all tests PASS, including the 4 new ones.

- [ ] **Step 5: Confirm milestone 4 is unchanged**

Run: `pio test -e native -f test_hil -v 2>&1 | grep -E '^(LINK|INTEGRITY),' > /tmp/m4-now.txt; grep -E '^(LINK|INTEGRITY),' docs/results/milestone-4-hil-regression.txt | diff - /tmp/m4-now.txt && echo IDENTICAL`
Expected: `IDENTICAL`.

- [ ] **Step 6: Deliberate-break checks**

Make each edit, run `pio test -e native -f test_hil_criteria`, confirm the named test fails, then revert with `git checkout test/support/hil.hpp`:

1. In `run_link_with`, change `build_link_frame(i, sent)` to `build_link_frame(i + 1, sent)`: `test_run_link_reproduces_milestone_4_rows` fails.
2. In `pattern_undetected`, change `sent[i] ^ errors[i]` to `sent[i]`: `test_crc_polynomial_pattern_is_undetected` fails.
3. Change `kCrcFieldFirstBit = 43 * 8` to `42 * 8`: `test_crc_field_starts_at_byte_43` fails.

- [ ] **Step 7: Commit**

```bash
git add test/support/hil.hpp test/test_hil_criteria/test_main.cpp
git commit -m "test: make link runs channel-generic and add the exhaustive short-burst check"
```

---

### Task 3: the milestone 6 runner, and its one kept run

**Files:**
- Create: `test/test_hil_burst/test_main.cpp`
- Create: `docs/results/milestone-6-burst-errors.txt` (the first complete run, unchanged)

**Interfaces:**
- Consumes: everything Task 1 and Task 2 produce, plus `kBerPoints`, `kBerPointCount`, `kChannelSeed`, `expected_ratio()`, `integrity_bound()`, `integrity_pass()`, `build_link_frame()`.
- Produces: the `BURST_LINK`, `BURST_EXHAUSTIVE`, `BURST_INTEGRITY` and `BURST_REG` lines the README quotes.

- [ ] **Step 1: Write the runner**

Create `test/test_hil_burst/test_main.cpp`:

```cpp
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>

#include <unity.h>

#include "../support/burst_channel.hpp"
#include "../support/hil.hpp"
#include "../support/noisy_channel.hpp"

using namespace floodnet;

void setUp(void) {}
void tearDown(void) {}

// Frozen in the milestone 6 design doc. The runner asserts only that each
// run did what it was told; whether a row passes is printed, never asserted.

static unsigned g_passed = 0;
static const unsigned kGraded = 3;

void test_burst_link_group(void) {
    for (size_t i = 1; i < kBerPointCount; ++i) {
        const double p = kBerPoints[i];
        for (size_t k = 0; k < kBurstLengthCount; ++k) {
            const double L = kBurstLengths[k];
            BurstChannel channel(p, L, kChannelSeed);
            const LinkResult r = run_link_with(channel, kBurstLinkFrames);
            TEST_ASSERT_EQUAL_UINT32(kBurstLinkFrames, r.frames);
            TEST_ASSERT_EQUAL_UINT64(kBurstLinkFrames, channel.frames());
            printf("BURST_LINK,%g,%g,%u,%u,%.6f,%.6f,%u,%u\n", p, L, r.frames, r.accepted,
                   static_cast<double>(r.accepted) / r.frames, expected_ratio(p), r.corrupted,
                   r.undetected);
        }
    }
}

/// Patterns of span 1 to kMaxBurstSpan lying wholly within the first
/// `n_bits` bits, from the closed form rather than from the enumerator.
static uint64_t bursts_within(uint64_t n_bits) {
    uint64_t count = n_bits;
    for (uint64_t w = 2; w <= kMaxBurstSpan; ++w) {
        count += (n_bits + 1 - w) << (w - 2);
    }
    return count;
}

void test_exhaustive_group(void) {
    uint8_t sent[PACKET_SIZE];
    build_link_frame(0, sent);
    const ExhaustiveResult r = run_exhaustive_bursts(sent);
    const uint64_t data_expected = bursts_within(kCrcFieldFirstBit);
    const uint64_t crc_expected = bursts_within(PACKET_SIZE * 8) - data_expected;
    TEST_ASSERT_EQUAL_UINT64(10813439, data_expected);
    TEST_ASSERT_EQUAL_UINT64(524288, crc_expected);
    TEST_ASSERT_EQUAL_UINT64(data_expected, r.data_patterns);
    TEST_ASSERT_EQUAL_UINT64(crc_expected, r.crc_patterns);
    const bool data_pass = r.data_undetected == 0;
    const bool crc_pass = r.crc_undetected == 0;
    g_passed += (data_pass ? 1 : 0) + (crc_pass ? 1 : 0);
    printf("BURST_EXHAUSTIVE,data,%llu,%llu,%d\n",
           static_cast<unsigned long long>(r.data_patterns),
           static_cast<unsigned long long>(r.data_undetected), data_pass ? 1 : 0);
    printf("BURST_EXHAUSTIVE,crc_field,%llu,%llu,%d\n",
           static_cast<unsigned long long>(r.crc_patterns),
           static_cast<unsigned long long>(r.crc_undetected), crc_pass ? 1 : 0);
}

void test_burst_integrity_group(void) {
    BurstChannel channel(kBurstIntegrityBer, kBurstIntegrityLength, kChannelSeed);
    const LinkResult r = run_link_with(channel, kBurstIntegrityFrames);
    TEST_ASSERT_EQUAL_UINT32(kBurstIntegrityFrames, r.frames);
    TEST_ASSERT_TRUE_MESSAGE(r.corrupted > 0, "integrity run corrupted nothing");
    const bool pass = integrity_pass(r.undetected, r.corrupted);
    g_passed += pass ? 1 : 0;
    printf("BURST_INTEGRITY,%g,%g,%u,%u,%u,%.3f,%d\n", kBurstIntegrityBer,
           kBurstIntegrityLength, r.frames, r.corrupted, r.undetected,
           integrity_bound(r.corrupted), pass ? 1 : 0);
}

void test_summary(void) { printf("BURST_REG,all,-,%u,%u\n", g_passed, kGraded); }

int main(int, char **) {
    UNITY_BEGIN();
    RUN_TEST(test_burst_link_group);
    RUN_TEST(test_exhaustive_group);
    RUN_TEST(test_burst_integrity_group);
    RUN_TEST(test_summary);
    return UNITY_END();
}
```

- [ ] **Step 2: Deliberate-break checks, before the kept run**

These use throwaway runs whose output is not kept or read beyond the failing assertion.
For each, make the edit, run `pio test -e native -f test_hil_burst 2>&1 | grep -E 'FAIL|Expected'`, confirm the named assertion fails, and revert:

1. In `run_exhaustive_bursts`, change `a + w <= bits` to `a + w < bits`: the `crc_patterns` count assertion fails.
2. In `run_exhaustive_bursts`, change `(1u << (w - 2))` to `(1u << (w - 3))` for `w >= 3` by writing `w >= 3 ? (1u << (w - 3)) : 1u`: the `data_patterns` count assertion fails.
3. In `test_burst_integrity_group`, construct the channel with `0.0` instead of `kBurstIntegrityBer`: "integrity run corrupted nothing" fails.

Revert with `git checkout test/support/hil.hpp` and by undoing the runner edit. `git diff` must show only the new runner file as untracked.

- [ ] **Step 3: Run once and keep the output**

Run: `pio test -e native -f test_hil_burst -v 2>&1 | tee docs/results/milestone-6-burst-errors.txt | grep -E '^BURST_|PASS|FAIL'`
Expected: 4 tests PASS; 35 `BURST_LINK`, 2 `BURST_EXHAUSTIVE`, 1 `BURST_INTEGRITY` and 1 `BURST_REG` lines. Record the wall-clock time from the `Took` line.

This is the only run whose output is kept.
Whatever the `crc_field` and integrity rows show, they are not re-run, and nothing is edited to change them.

- [ ] **Step 4: Commit**

```bash
git add test/test_hil_burst/test_main.cpp docs/results/milestone-6-burst-errors.txt
git commit -m "test: run the frozen burst-error scenarios and keep the first run"
```

---

### Task 4: publish the results

**Files:**
- Modify: `README.md` (Status, a new `### Milestone 6: burst bit errors` section after `### Milestone 5: board target`, the "Milestone 4 link regression" limitation "Bit errors are independent", and a new `### Milestone 6 burst errors` limitations section after "Milestone 5 board target")
- Modify: `docs/superpowers/specs/2026-09-30-milestone-6-burst-errors-design.md` (Status, Revisions)
- Modify: `CLAUDE.md` only if the work found a new mistake worth guarding

**Interfaces:**
- Consumes: `docs/results/milestone-6-burst-errors.txt` (Task 3). Every number written comes from that file, verbatim; nothing is rounded toward a preferred value.

- [ ] **Step 1: Update the spec**

Set `Status: implemented`.
Under "Revisions", add a dated entry for any change made during implementation to a frozen value, or leave "None yet." replaced with "None." if there were none.

- [ ] **Step 2: Write the README section**

Structure, matching the milestone 4 section:

1. One paragraph: what milestone 6 tests and that it is simulated, pointing at the spec.
2. `#### The channel`: the Gilbert model, `h` = 0.5, *L* as mean bad-run length, the per-frame reset from the long-run mix, and that *p* is set, not measured.
3. `#### A property of the wire format`: the little-endian CRC paragraph from the spec, in past tense, followed by what the `crc_field` row found.
4. `#### Raw output`: every `BURST_` line from the results file, verbatim, in a `text` block, then the four line formats.
5. `#### Exhaustive short-burst check`: the two rows as a table, and a plain statement of the result. If `crc_field` failed, state it as the finding, give the count, and state the proposed fix (CRC most significant byte first in a future packet version) and that it was not made.
6. `#### Burst link sweep`: a table of *p* by *L* acceptance ratios from the `BURST_LINK` lines, with the independent ratio column, and one or two sentences on what the rows show about corrupted-frame share. Note that every row reuses `kChannelSeed`, as milestone 4 did.
7. `#### Burst integrity`: the row, its bound, and the pass or fail.
8. `#### Regression`: `BURST_REG`, stated as separate from milestone 4's 19 of 21.

Add to Status, after the milestone 5 line: one sentence naming milestone 6 and its `BURST_REG` result.
Change the milestone 4 limitation "Bit errors are independent." to point at milestone 6 for the burst case.
Add `### Milestone 6 burst errors` under Known limitations with the spec's "Known limitations, stated in advance", plus the run time of `test_hil_burst`.

- [ ] **Step 3: Check the numbers**

Run: `grep -E '^BURST_' docs/results/milestone-6-burst-errors.txt | while read -r line; do grep -qF "$line" README.md || echo "MISSING: $line"; done`
Expected: no output.

- [ ] **Step 4: Full suite and firmware build**

Run: `pio test -e native 2>&1 | tail -2` and `pio run -e node_polling -e node_interrupt -e node_duty_cycled -e gateway 2>&1 | tail -6`
Expected: every native test case passes; 4 environments SUCCESS.

- [ ] **Step 5: Commit**

```bash
git add README.md docs/superpowers/specs/2026-09-30-milestone-6-burst-errors-design.md
git commit -m "docs: publish the milestone 6 burst-error results"
```
