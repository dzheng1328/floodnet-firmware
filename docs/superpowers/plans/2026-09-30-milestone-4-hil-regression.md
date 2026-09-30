# Milestone 4 Link Regression and Signal-Integrity Tool Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Build a host-side link regression and signal-integrity tool that sweeps a bit-error channel between the node and the gateway, reports link delivery, undetected corruption, end-to-end record delivery per build and a regression pass rate, and publishes the first run verbatim.

**Architecture:** A seeded `NoisyChannel` flips bits in a frame. A link sweep feeds encoded frames through it into the real `decode_packet()`. `PowerRig` gains an optional channel between its radio and its gateway decode, plus a count of records queued across boots. A Unity test suite, `test_hil`, runs the frozen scenario table and prints `LINK`, `INTEGRITY`, `TRANSPARENT`, `FAULT`, `E2E` and `REG` lines. No file under `lib/` or `src/` changes.

**Tech Stack:** C++17 (`-std=gnu++17 -Wall -Wextra`), PlatformIO `native` env, Unity.

**Spec:** `docs/superpowers/specs/2026-09-30-milestone-4-hil-regression-design.md`

## Global Constraints

- Build flags stay `-std=gnu++17 -Wall -Wextra`; new code compiles without warnings.
- No file under `lib/` or `src/` changes in this milestone.
- BER points: 0, 1e-6, 1e-5, 1e-4, 3e-4, 1e-3, 3e-3, 1e-2.
- Frame length `PACKET_SIZE` = 45 bytes = 360 bits.
- Link frames per BER point: 100 000. Integrity frames at p = 1e-2: 1 000 000.
- End-to-end, transparency and fault runs: 24 simulated hours each.
- Builds: `interrupt` and `duty_cycled` only.
- PRNG: splitmix64, seed `0x466C6F6F644E6574`. Every channel instance starts from this seed.
- Link pass: `|accepted - N q| <= 3 sqrt(N q (1 - q))`, `q = (1 - p)^360`.
- Integrity pass: `undetected <= lambda + 3 sqrt(lambda) + 1`, `lambda = corrupted / 65536`.
- Graded rows: 8 link + 1 integrity + 2 transparency + 10 fault = 21. End-to-end rows are reported, not graded.
- The milestone 1 and 2 benchmark rows in `README.md` and the milestone 3 `LIFE`/`DOWNTIME` lines in `docs/results/milestone-3-power-bench.txt` must reproduce byte for byte after every task that touches `test/support/`.
- Benchmark guard: `diff <(grep -o '^BENCH,[^ ]*' README.md) <(pio test -e native -f test_benchmark -v 2>&1 | grep -o 'BENCH,[^ ]*' | head -12)` prints nothing.
- Power-bench guard: `diff <(grep -E '^(LIFE|DOWNTIME),' docs/results/milestone-3-power-bench.txt) <(pio test -e native -f test_power_bench -v 2>&1 | grep -E '^(LIFE|DOWNTIME),')` prints nothing.
- Every harness assertion must be shown to fail on a deliberately broken local edit, then the edit reverted (project `CLAUDE.md`).
- Nothing measured may be tuned after it is seen. A surprising result is reported, not adjusted.
- Never use the em dash character; use "-". Markdown: one sentence per physical line.
- Commit messages use `feat:`/`fix:`/`test:`/`docs:`/`refactor:` prefixes and carry no co-author trailer.

## Review Focus

1. p = 0 must change nothing: a channel at p = 0 must not touch the frame or draw from the generator, or the transparency rows and seeded reproducibility break. Pinned in Task 1 (`draws() == 0`) and Task 3 (record lists identical).
2. Very small p (1e-6) makes geometric gaps enormous; the gap arithmetic must not overflow or loop. Pinned in Task 1 (`test_tiny_p_flip_count_matches_expectation`).
3. A duty-cycled reboot resets the sampler's sequence counter; the end-to-end denominator must still count the records queued before the reboot. Pinned in Task 3 (`test_records_queued_survives_a_reboot`).
4. A corrupted frame that still passes the CRC must be counted as undetected, and one that fails must not. Pinned in Task 2 (`test_crc_matched_corruption_is_undetected`, `test_plain_corruption_is_rejected`).
5. The interrupt build sends v0x01 frames; the channel must corrupt and reject those too. Pinned in Task 3 (`test_interrupt_build_frames_are_corrupted_by_the_channel`).

---

### Task 1: `NoisyChannel`

**Files:**
- Create: `test/support/noisy_channel.hpp`
- Test: `test/test_noisy_channel/test_main.cpp`

**Interfaces:**
- Consumes: nothing.
- Produces:
  - `const uint64_t kChannelSeed = 0x466C6F6F644E6574ULL;`
  - `class NoisyChannel { NoisyChannel(double p, uint64_t seed); size_t corrupt(uint8_t *frame, size_t len); uint64_t draws() const; double p() const; };`

- [ ] **Step 1: Write the failing tests**

Create `test/test_noisy_channel/test_main.cpp`:

```cpp
#include <math.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include <unity.h>

#include "../support/noisy_channel.hpp"

using namespace floodnet;

void setUp(void) {}
void tearDown(void) {}

static const size_t kLen = 45;
static const uint32_t kFrames = 100000;

static void fill(uint8_t *frame) {
    for (size_t i = 0; i < kLen; ++i) {
        frame[i] = static_cast<uint8_t>(i * 37 + 11);
    }
}

void test_zero_p_changes_nothing_and_draws_nothing(void) {
    NoisyChannel ch(0.0, kChannelSeed);
    uint8_t frame[kLen];
    uint8_t original[kLen];
    fill(frame);
    memcpy(original, frame, kLen);
    for (int i = 0; i < 1000; ++i) {
        TEST_ASSERT_EQUAL_UINT32(0, ch.corrupt(frame, kLen));
    }
    TEST_ASSERT_EQUAL_MEMORY(original, frame, kLen);
    TEST_ASSERT_EQUAL_UINT64(0, ch.draws());
}

void test_p_one_flips_every_bit(void) {
    NoisyChannel ch(1.0, kChannelSeed);
    uint8_t frame[kLen];
    fill(frame);
    uint8_t expected[kLen];
    for (size_t i = 0; i < kLen; ++i) {
        expected[i] = static_cast<uint8_t>(~frame[i]);
    }
    TEST_ASSERT_EQUAL_UINT32(kLen * 8, ch.corrupt(frame, kLen));
    TEST_ASSERT_EQUAL_MEMORY(expected, frame, kLen);
}

static void assert_flip_count_within_3_sigma(double p) {
    NoisyChannel ch(p, kChannelSeed);
    uint8_t frame[kLen];
    uint64_t flips = 0;
    for (uint32_t i = 0; i < kFrames; ++i) {
        fill(frame);
        flips += ch.corrupt(frame, kLen);
    }
    const double n = static_cast<double>(kFrames) * kLen * 8;
    const double mean = n * p;
    const double sd = sqrt(n * p * (1.0 - p));
    TEST_ASSERT_TRUE_MESSAGE(fabs(static_cast<double>(flips) - mean) <= 3.0 * sd,
                             "flip count outside 3 sigma of n p");
}

void test_flip_rate_matches_p(void) { assert_flip_count_within_3_sigma(1e-3); }

void test_tiny_p_flip_count_matches_expectation(void) { assert_flip_count_within_3_sigma(1e-6); }

void test_same_seed_reproduces_the_same_corruption(void) {
    NoisyChannel a(1e-2, kChannelSeed);
    NoisyChannel b(1e-2, kChannelSeed);
    uint8_t fa[kLen];
    uint8_t fb[kLen];
    for (int i = 0; i < 1000; ++i) {
        fill(fa);
        fill(fb);
        TEST_ASSERT_EQUAL_UINT32(a.corrupt(fa, kLen), b.corrupt(fb, kLen));
        TEST_ASSERT_EQUAL_MEMORY(fa, fb, kLen);
    }
}

int main(int, char **) {
    UNITY_BEGIN();
    RUN_TEST(test_zero_p_changes_nothing_and_draws_nothing);
    RUN_TEST(test_p_one_flips_every_bit);
    RUN_TEST(test_flip_rate_matches_p);
    RUN_TEST(test_tiny_p_flip_count_matches_expectation);
    RUN_TEST(test_same_seed_reproduces_the_same_corruption);
    return UNITY_END();
}
```

- [ ] **Step 2: Run to verify it fails**

Run: `pio test -e native -f test_noisy_channel`
Expected: compile failure, `noisy_channel.hpp: No such file or directory`.

- [ ] **Step 3: Implement**

Create `test/support/noisy_channel.hpp`:

```cpp
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
```

- [ ] **Step 4: Run to verify it passes**

Run: `pio test -e native -f test_noisy_channel`
Expected: 5 tests PASS, no warnings.

- [ ] **Step 5: Show the assertions can fail**

Temporarily change `if (p_ <= 0.0 || len == 0)` to `if (len == 0)` and rerun; `test_zero_p_changes_nothing_and_draws_nothing` must FAIL on `draws()`.
Temporarily change `pos += 1 + gap();` to `pos += 2 + gap();`; `test_flip_rate_matches_p` must FAIL, because the flip rate drops below p.
Revert both edits and rerun to PASS.

- [ ] **Step 6: Commit**

```bash
git add test/support/noisy_channel.hpp test/test_noisy_channel/test_main.cpp
git commit -m "test: add a seeded bit-error channel for the link regression tool"
```

---

### Task 2: Link sweep, undetected-corruption detector and pass criteria

**Files:**
- Create: `test/support/hil.hpp`
- Test: `test/test_hil_criteria/test_main.cpp`

**Interfaces:**
- Consumes: `NoisyChannel`, `kChannelSeed` (Task 1); `Packet`, `encode_packet_as`, `decode_packet`, `crc16_ccitt`, `PACKET_SIZE`, `WireVersion` (`<floodnet/packet.hpp>`).
- Produces:
  - `const double kBerPoints[8]; const size_t kBerPointCount = 8;`
  - `const uint32_t kLinkFrames = 100000; const uint32_t kIntegrityFrames = 1000000; const double kIntegrityBer = 1e-2; const uint32_t kHilRunMs = 86400000UL;`
  - `struct LinkResult { uint32_t frames; uint32_t accepted; uint32_t corrupted; uint32_t undetected; };`
  - `void build_link_frame(uint32_t i, uint8_t *out);` (writes `PACKET_SIZE` bytes)
  - `bool is_undetected(const uint8_t *sent, const uint8_t *received, size_t len, bool accepted);`
  - `LinkResult run_link(double p, uint32_t frames, uint64_t seed);`
  - `double expected_ratio(double p);`
  - `bool link_pass(uint32_t accepted, uint32_t frames, double q);`
  - `double integrity_bound(uint32_t corrupted);`
  - `bool integrity_pass(uint32_t undetected, uint32_t corrupted);`

- [ ] **Step 1: Write the failing tests**

Create `test/test_hil_criteria/test_main.cpp`:

```cpp
#include <math.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include <unity.h>

#include <floodnet/packet.hpp>

#include "../support/hil.hpp"

using namespace floodnet;

void setUp(void) {}
void tearDown(void) {}

void test_link_frame_decodes(void) {
    uint8_t frame[PACKET_SIZE];
    build_link_frame(7, frame);
    Packet p;
    TEST_ASSERT_TRUE(decode_packet(frame, PACKET_SIZE, &p));
    TEST_ASSERT_EQUAL_UINT32(7, p.seq);
}

void test_crc_matched_corruption_is_undetected(void) {
    uint8_t sent[PACKET_SIZE];
    uint8_t rx[PACKET_SIZE];
    build_link_frame(7, sent);
    memcpy(rx, sent, PACKET_SIZE);
    rx[20] ^= 0x01;
    const uint16_t crc = crc16_ccitt(rx, 43);
    rx[43] = static_cast<uint8_t>(crc & 0xFF);
    rx[44] = static_cast<uint8_t>(crc >> 8);
    Packet p;
    const bool accepted = decode_packet(rx, PACKET_SIZE, &p);
    TEST_ASSERT_TRUE(accepted);
    TEST_ASSERT_TRUE(is_undetected(sent, rx, PACKET_SIZE, accepted));
}

void test_plain_corruption_is_rejected(void) {
    uint8_t sent[PACKET_SIZE];
    uint8_t rx[PACKET_SIZE];
    build_link_frame(7, sent);
    memcpy(rx, sent, PACKET_SIZE);
    rx[20] ^= 0x01;
    Packet p;
    const bool accepted = decode_packet(rx, PACKET_SIZE, &p);
    TEST_ASSERT_FALSE(accepted);
    TEST_ASSERT_FALSE(is_undetected(sent, rx, PACKET_SIZE, accepted));
}

void test_clean_frame_is_not_undetected(void) {
    uint8_t sent[PACKET_SIZE];
    build_link_frame(3, sent);
    TEST_ASSERT_FALSE(is_undetected(sent, sent, PACKET_SIZE, true));
}

void test_run_link_at_zero_p_accepts_everything(void) {
    const LinkResult r = run_link(0.0, 1000, kChannelSeed);
    TEST_ASSERT_EQUAL_UINT32(1000, r.frames);
    TEST_ASSERT_EQUAL_UINT32(1000, r.accepted);
    TEST_ASSERT_EQUAL_UINT32(0, r.corrupted);
    TEST_ASSERT_EQUAL_UINT32(0, r.undetected);
}

void test_run_link_at_p_one_accepts_nothing(void) {
    const LinkResult r = run_link(1.0, 1000, kChannelSeed);
    TEST_ASSERT_EQUAL_UINT32(1000, r.corrupted);
    TEST_ASSERT_EQUAL_UINT32(0, r.accepted);
}

void test_expected_ratio(void) {
    TEST_ASSERT_EQUAL_DOUBLE(1.0, expected_ratio(0.0));
    TEST_ASSERT_DOUBLE_WITHIN(1e-12, pow(0.999, 360.0), expected_ratio(1e-3));
}

void test_link_pass_is_exact_at_zero_p(void) {
    TEST_ASSERT_TRUE(link_pass(100000, 100000, 1.0));
    TEST_ASSERT_FALSE(link_pass(99999, 100000, 1.0));
}

void test_link_pass_uses_three_sigma(void) {
    // N = 10000, q = 0.5: mean 5000, sd 50, so 5150 passes and 5151 fails.
    TEST_ASSERT_TRUE(link_pass(5150, 10000, 0.5));
    TEST_ASSERT_FALSE(link_pass(5151, 10000, 0.5));
    TEST_ASSERT_TRUE(link_pass(4850, 10000, 0.5));
    TEST_ASSERT_FALSE(link_pass(4849, 10000, 0.5));
}

void test_integrity_bound(void) {
    TEST_ASSERT_EQUAL_DOUBLE(1.0, integrity_bound(0));
    // corrupted = 65536 * 4: lambda 4, bound 4 + 6 + 1 = 11.
    TEST_ASSERT_EQUAL_DOUBLE(11.0, integrity_bound(262144));
    TEST_ASSERT_TRUE(integrity_pass(11, 262144));
    TEST_ASSERT_FALSE(integrity_pass(12, 262144));
}

int main(int, char **) {
    UNITY_BEGIN();
    RUN_TEST(test_link_frame_decodes);
    RUN_TEST(test_crc_matched_corruption_is_undetected);
    RUN_TEST(test_plain_corruption_is_rejected);
    RUN_TEST(test_clean_frame_is_not_undetected);
    RUN_TEST(test_run_link_at_zero_p_accepts_everything);
    RUN_TEST(test_run_link_at_p_one_accepts_nothing);
    RUN_TEST(test_expected_ratio);
    RUN_TEST(test_link_pass_is_exact_at_zero_p);
    RUN_TEST(test_link_pass_uses_three_sigma);
    RUN_TEST(test_integrity_bound);
    return UNITY_END();
}
```

- [ ] **Step 2: Run to verify it fails**

Run: `pio test -e native -f test_hil_criteria`
Expected: compile failure, `hil.hpp: No such file or directory`.

- [ ] **Step 3: Implement**

Create `test/support/hil.hpp`:

```cpp
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
```

- [ ] **Step 4: Run to verify it passes**

Run: `pio test -e native -f test_hil_criteria`
Expected: 10 tests PASS, no warnings.

- [ ] **Step 5: Show the assertions can fail**

Temporarily change `is_undetected` to `return accepted;`; `test_clean_frame_is_not_undetected` must FAIL.
Temporarily change `3.0 * sd` to `4.0 * sd` in `link_pass`; `test_link_pass_uses_three_sigma` must FAIL.
Revert both and rerun to PASS.

- [ ] **Step 6: Commit**

```bash
git add test/support/hil.hpp test/test_hil_criteria/test_main.cpp
git commit -m "test: add the link sweep, undetected-corruption count and frozen pass criteria"
```

---

### Task 3: `PowerRig` channel hook and records-queued count

**Files:**
- Modify: `test/support/power_rig.hpp`
- Test: `test/test_rig_channel/test_main.cpp`

**Interfaces:**
- Consumes: `NoisyChannel`, `kChannelSeed` (Task 1); `PowerRig`, `run_experiment_2_fault`, `fault_case`, `kFaultRunSleepNA`, `kFaultRunCapacityNaMs` (existing, `test/support/power_experiments.hpp`); `DutyCycledNode::sampler()`, `InterruptSampler::next_seq()` (existing).
- Produces:
  - `void PowerRig::set_channel(NoisyChannel *channel);` (nullptr, the default, means no channel)
  - `uint32_t PowerRig::records_queued() const;` (sequence numbers issued over every boot)

- [ ] **Step 1: Write the failing tests**

Create `test/test_rig_channel/test_main.cpp`:

```cpp
#include <stddef.h>
#include <stdint.h>

#include <unity.h>

#include "../support/noisy_channel.hpp"
#include "../support/power_experiments.hpp"
#include "../support/power_rig.hpp"

using namespace floodnet;

void setUp(void) {}
void tearDown(void) {}

static const uint32_t kSixHoursMs = 6UL * 3600000UL;

static bool same_records(const PowerRig &a, const PowerRig &b) {
    const std::vector<GatewayRecord> &ra = a.records();
    const std::vector<GatewayRecord> &rb = b.records();
    if (ra.size() != rb.size()) {
        return false;
    }
    for (size_t i = 0; i < ra.size(); ++i) {
        if (ra[i].arrival_ms != rb[i].arrival_ms || ra[i].gps_valid != rb[i].gps_valid ||
            ra[i].boot_count != rb[i].boot_count || ra[i].tx_timeouts != rb[i].tx_timeouts) {
            return false;
        }
    }
    return true;
}

static void assert_zero_p_channel_is_transparent(Build build) {
    PowerRig plain(build, kFaultRunSleepNA, kFaultRunCapacityNaMs);
    plain.run_until(kSixHoursMs);
    NoisyChannel channel(0.0, kChannelSeed);
    PowerRig noisy(build, kFaultRunSleepNA, kFaultRunCapacityNaMs);
    noisy.set_channel(&channel);
    noisy.run_until(kSixHoursMs);
    TEST_ASSERT_TRUE(plain.records().size() > 0);
    TEST_ASSERT_TRUE(same_records(plain, noisy));
}

void test_zero_p_channel_is_transparent_interrupt(void) {
    assert_zero_p_channel_is_transparent(Build::Interrupt);
}

void test_zero_p_channel_is_transparent_duty_cycled(void) {
    assert_zero_p_channel_is_transparent(Build::DutyCycled);
}

void test_p_one_channel_delivers_nothing(void) {
    NoisyChannel channel(1.0, kChannelSeed);
    PowerRig rig(Build::DutyCycled, kFaultRunSleepNA, kFaultRunCapacityNaMs);
    rig.set_channel(&channel);
    rig.run_until(2UL * 3600000UL);
    TEST_ASSERT_TRUE(rig.radio().sent_count() > 0);
    TEST_ASSERT_EQUAL_UINT32(0, rig.deliveries().size());
}

void test_interrupt_build_frames_are_corrupted_by_the_channel(void) {
    NoisyChannel channel(1e-2, kChannelSeed);
    PowerRig rig(Build::Interrupt, kFaultRunSleepNA, kFaultRunCapacityNaMs);
    rig.set_channel(&channel);
    rig.run_until(2UL * 3600000UL);
    TEST_ASSERT_TRUE(rig.radio().sent_count() > 0);
    TEST_ASSERT_TRUE(rig.deliveries().size() < rig.radio().sent_count());
}

void test_records_queued_counts_every_queued_record(void) {
    PowerRig rig(Build::Interrupt, kFaultRunSleepNA, kFaultRunCapacityNaMs);
    rig.run_until(3600000UL);
    TEST_ASSERT_TRUE(rig.deliveries().size() > 0);
    TEST_ASSERT_TRUE(rig.records_queued() >= rig.deliveries().size());
}

void test_records_queued_survives_a_reboot(void) {
    PowerRig rig(Build::DutyCycled, kFaultRunSleepNA, kFaultRunCapacityNaMs);
    run_experiment_2_fault(rig, fault_case(FaultKind::Hang));
    TEST_ASSERT_EQUAL_UINT32(1, rig.reboots());
    TEST_ASSERT_TRUE(rig.records_queued() > rig.node().sampler().next_seq());
    TEST_ASSERT_TRUE(rig.records_queued() >= rig.deliveries().size());
}

int main(int, char **) {
    UNITY_BEGIN();
    RUN_TEST(test_zero_p_channel_is_transparent_interrupt);
    RUN_TEST(test_zero_p_channel_is_transparent_duty_cycled);
    RUN_TEST(test_p_one_channel_delivers_nothing);
    RUN_TEST(test_interrupt_build_frames_are_corrupted_by_the_channel);
    RUN_TEST(test_records_queued_counts_every_queued_record);
    RUN_TEST(test_records_queued_survives_a_reboot);
    return UNITY_END();
}
```

- [ ] **Step 2: Run to verify it fails**

Run: `pio test -e native -f test_rig_channel`
Expected: compile failure, `'class floodnet::PowerRig' has no member named 'set_channel'`.

- [ ] **Step 3: Implement**

In `test/support/power_rig.hpp`:

Add after `#include "hang_breaker.hpp"`:

```cpp
#include "noisy_channel.hpp"
```

Add to the end of the constructor's initialiser list, after `reboots_(0)`:

```cpp
          reboots_(0),
          channel_(nullptr),
          queued_before_boot_(0) {
```

Add these public members after `size_t reboots() const { return reboots_; }`:

```cpp
    /// Puts `channel` between the node's radio and the gateway's decode.
    /// nullptr, the default, is no channel: the delivery path is then exactly
    /// the milestone 3 path.
    void set_channel(NoisyChannel *channel) { channel_ = channel; }

    /// Sequence numbers the node issued, summed over every boot: every record
    /// it queued for transmission, whether or not it was ever sent.
    uint32_t records_queued() const {
        const uint32_t current = node_ ? node_->sampler().next_seq() : sampler_->next_seq();
        return queued_before_boot_ + current;
    }
```

In `reboot()`, add as the first statement after `++reboots_;`:

```cpp
        queued_before_boot_ += node_->sampler().next_seq();
```

Replace the decode in `collect_delivery()`:

```cpp
        Packet packet;
        if (!decode_packet(radio_.last_payload(), radio_.last_length(), &packet)) {
            return;
        }
```

with:

```cpp
        Packet packet;
        const uint8_t *payload = radio_.last_payload();
        size_t length = radio_.last_length();
        uint8_t received[kRigMaxFrame];
        if (channel_ != nullptr) {
            if (length > kRigMaxFrame) {
                length = kRigMaxFrame;
            }
            memcpy(received, payload, length);
            channel_->corrupt(received, length);
            payload = received;
        }
        if (!decode_packet(payload, length, &packet)) {
            return;
        }
```

Add `#include <string.h>` after `#include <stdint.h>`, and after `const uint8_t kRigTtl = 3;` add:

```cpp
/// FakeRadio keeps at most 64 payload bytes, so a channel copy never needs more.
const size_t kRigMaxFrame = 64;
```

Add the private members after `size_t reboots_;`:

```cpp
    NoisyChannel *channel_;
    uint32_t queued_before_boot_;
```

- [ ] **Step 4: Run to verify it passes**

Run: `pio test -e native -f test_rig_channel`
Expected: 6 tests PASS, no warnings.

- [ ] **Step 5: Run the guards and the full suite**

Run the benchmark guard and the power-bench guard from Global Constraints; both must print nothing.
Run: `pio test -e native`
Expected: every suite PASSES.

- [ ] **Step 6: Show the assertions can fail**

Temporarily delete the `queued_before_boot_ += ...` line; `test_records_queued_survives_a_reboot` must FAIL.
Temporarily make `set_channel` ignore its argument (empty body); `test_p_one_channel_delivers_nothing` must FAIL.
Revert both and rerun to PASS.

- [ ] **Step 7: Commit**

```bash
git add test/support/power_rig.hpp test/test_rig_channel/test_main.cpp
git commit -m "test: let the power rig put a noisy channel before the gateway and count queued records"
```

---

### Task 4: The `test_hil` regression runner

**Files:**
- Create: `test/test_hil/test_main.cpp`

**Interfaces:**
- Consumes: everything from Tasks 1-3; `kFaults`, `kFaultCount`, `run_experiment_2_fault`, `kFaultRunMs`, `kFaultRunSleepNA`, `kFaultRunCapacityNaMs` (existing).
- Produces: the `LINK`, `INTEGRITY`, `TRANSPARENT`, `FAULT`, `E2E` and `REG` lines in the spec's "Output format".

The runner asserts only harness validity (the run did what it was told), never a measured value or a pass criterion.
A row that fails its criterion prints `pass` = 0; it does not fail the test.

- [ ] **Step 1: Write the runner**

Create `test/test_hil/test_main.cpp`:

```cpp
#include <math.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>

#include <unity.h>

#include "../support/hil.hpp"
#include "../support/noisy_channel.hpp"
#include "../support/power_experiments.hpp"
#include "../support/power_rig.hpp"

using namespace floodnet;

void setUp(void) {}
void tearDown(void) {}

// Frozen in the milestone 4 design doc, "Frozen scenario table and pass
// criteria". The runner asserts only that each run did what it was told;
// whether a row passes is printed, never asserted.

static const Build kBuilds[] = {Build::Interrupt, Build::DutyCycled};
static const size_t kBuildCount = 2;

static const char *build_name(Build build) {
    return build == Build::Interrupt ? "interrupt" : "duty_cycled";
}

static unsigned g_link_passed = 0;
static unsigned g_integrity_passed = 0;
static unsigned g_transparency_passed[kBuildCount] = {0, 0};
static unsigned g_fault_passed[kBuildCount] = {0, 0};

void test_link_group(void) {
    for (size_t i = 0; i < kBerPointCount; ++i) {
        const double p = kBerPoints[i];
        const LinkResult r = run_link(p, kLinkFrames, kChannelSeed);
        TEST_ASSERT_EQUAL_UINT32(kLinkFrames, r.frames);
        const double q = expected_ratio(p);
        const bool pass = link_pass(r.accepted, r.frames, q);
        g_link_passed += pass ? 1 : 0;
        printf("LINK,%g,%u,%u,%.1f,%.6f,%.6f,%u,%d\n", p, r.frames, r.accepted,
               r.frames * q, static_cast<double>(r.accepted) / r.frames, q, r.undetected,
               pass ? 1 : 0);
    }
}

void test_integrity_group(void) {
    const LinkResult r = run_link(kIntegrityBer, kIntegrityFrames, kChannelSeed);
    TEST_ASSERT_EQUAL_UINT32(kIntegrityFrames, r.frames);
    TEST_ASSERT_TRUE_MESSAGE(r.corrupted > 0, "integrity run corrupted nothing");
    const bool pass = integrity_pass(r.undetected, r.corrupted);
    g_integrity_passed += pass ? 1 : 0;
    printf("INTEGRITY,%g,%u,%u,%u,%.3f,%d\n", kIntegrityBer, r.frames, r.corrupted,
           r.undetected, integrity_bound(r.corrupted), pass ? 1 : 0);
}

static bool same_records(const PowerRig &a, const PowerRig &b) {
    const std::vector<GatewayRecord> &ra = a.records();
    const std::vector<GatewayRecord> &rb = b.records();
    if (ra.size() != rb.size()) {
        return false;
    }
    for (size_t i = 0; i < ra.size(); ++i) {
        if (ra[i].arrival_ms != rb[i].arrival_ms || ra[i].gps_valid != rb[i].gps_valid ||
            ra[i].boot_count != rb[i].boot_count || ra[i].tx_timeouts != rb[i].tx_timeouts) {
            return false;
        }
    }
    return true;
}

void test_transparency_group(void) {
    for (size_t b = 0; b < kBuildCount; ++b) {
        PowerRig plain(kBuilds[b], kFaultRunSleepNA, kFaultRunCapacityNaMs);
        plain.run_until(kHilRunMs);
        NoisyChannel channel(0.0, kChannelSeed);
        PowerRig noisy(kBuilds[b], kFaultRunSleepNA, kFaultRunCapacityNaMs);
        noisy.set_channel(&channel);
        noisy.run_until(kHilRunMs);
        TEST_ASSERT_TRUE_MESSAGE(plain.clock().now_ms() >= kHilRunMs, "plain run ended early");
        TEST_ASSERT_TRUE_MESSAGE(noisy.clock().now_ms() >= kHilRunMs, "noisy run ended early");
        TEST_ASSERT_TRUE_MESSAGE(plain.records().size() > 0, "plain run delivered nothing");
        const bool identical = same_records(plain, noisy);
        g_transparency_passed[b] += identical ? 1 : 0;
        printf("TRANSPARENT,%s,%u,%u,%d,%d\n", build_name(kBuilds[b]),
               static_cast<unsigned>(noisy.records().size()),
               static_cast<unsigned>(plain.records().size()), identical ? 1 : 0,
               identical ? 1 : 0);
    }
}

void test_fault_group(void) {
    for (size_t b = 0; b < kBuildCount; ++b) {
        for (size_t f = 0; f < kFaultCount; ++f) {
            PowerRig rig(kBuilds[b], kFaultRunSleepNA, kFaultRunCapacityNaMs);
            run_experiment_2_fault(rig, kFaults[f]);
            TEST_ASSERT_TRUE_MESSAGE(rig.fault_started(0), "fault never started");
            const bool recovered = !rig.downtime(kFaultRunMs).down_at_end;
            g_fault_passed[b] += recovered ? 1 : 0;
            printf("FAULT,%s,%s,%d,%d\n", build_name(kBuilds[b]), kFaults[f].name,
                   recovered ? 1 : 0, recovered ? 1 : 0);
        }
    }
}

void test_e2e_group(void) {
    for (size_t b = 0; b < kBuildCount; ++b) {
        for (size_t i = 0; i < kBerPointCount; ++i) {
            NoisyChannel channel(kBerPoints[i], kChannelSeed);
            PowerRig rig(kBuilds[b], kFaultRunSleepNA, kFaultRunCapacityNaMs);
            rig.set_channel(&channel);
            rig.run_until(kHilRunMs);
            TEST_ASSERT_TRUE_MESSAGE(rig.clock().now_ms() >= kHilRunMs, "run ended early");
            const uint32_t queued = rig.records_queued();
            TEST_ASSERT_TRUE_MESSAGE(queued > 0, "node queued nothing");
            const unsigned accepted = static_cast<unsigned>(rig.deliveries().size());
            printf("E2E,%s,%g,%u,%u,%.6f\n", build_name(kBuilds[b]), kBerPoints[i], queued,
                   accepted, static_cast<double>(accepted) / queued);
        }
    }
}

void test_summary(void) {
    printf("REG,link,-,%u,%u\n", g_link_passed, static_cast<unsigned>(kBerPointCount));
    printf("REG,integrity,-,%u,1\n", g_integrity_passed);
    unsigned total = g_link_passed + g_integrity_passed;
    for (size_t b = 0; b < kBuildCount; ++b) {
        printf("REG,transparency,%s,%u,1\n", build_name(kBuilds[b]), g_transparency_passed[b]);
        printf("REG,fault,%s,%u,%u\n", build_name(kBuilds[b]), g_fault_passed[b],
               static_cast<unsigned>(kFaultCount));
        total += g_transparency_passed[b] + g_fault_passed[b];
    }
    printf("REG,all,-,%u,21\n", total);
}

int main(int, char **) {
    UNITY_BEGIN();
    RUN_TEST(test_link_group);
    RUN_TEST(test_integrity_group);
    RUN_TEST(test_transparency_group);
    RUN_TEST(test_fault_group);
    RUN_TEST(test_e2e_group);
    RUN_TEST(test_summary);
    return UNITY_END();
}
```

- [ ] **Step 2: Show the harness assertions can fail**

Temporarily change `run_link(p, kLinkFrames, kChannelSeed)` to `run_link(p, kLinkFrames - 1, kChannelSeed)`; `test_link_group` must FAIL.
Temporarily change `rig.run_until(kHilRunMs);` in `test_e2e_group` to `rig.run_until(kHilRunMs / 2);`; `test_e2e_group` must FAIL on "run ended early".
Revert both.
These edits must not be run with `-v` into the results file.

- [ ] **Step 3: Run once and keep the output**

Run: `pio test -e native -f test_hil -v 2>&1 | tee docs/results/milestone-4-hil-regression.txt | grep -E '^(LINK|INTEGRITY|TRANSPARENT|FAULT|E2E|REG),|PASS|FAIL'`
Expected: 6 tests PASS; 8 `LINK`, 1 `INTEGRITY`, 2 `TRANSPARENT`, 10 `FAULT`, 16 `E2E` and 7 `REG` lines. Record the wall-clock time.

This is the only run whose output is kept.
If a harness assertion fails, debug it with superpowers:systematic-debugging and rerun, listing every rerun and its reason in the commit message.
If a row prints `pass` = 0, that is a result: do not change any constant, criterion or code to move it.

- [ ] **Step 4: Run the whole suite**

Run: `pio test -e native`
Expected: every suite PASSES.
Run the benchmark guard and the power-bench guard; both print nothing.

- [ ] **Step 5: Commit**

```bash
git add test/test_hil/test_main.cpp docs/results/milestone-4-hil-regression.txt
git commit -m "test: run the frozen link regression and signal-integrity scenarios and keep the first run"
```

---

### Task 5: Publish the results

**Files:**
- Modify: `README.md`
- Modify: `docs/superpowers/specs/2026-09-30-milestone-4-hil-regression-design.md`

**Interfaces:**
- Consumes: `docs/results/milestone-4-hil-regression.txt` (Task 4). Every number written here comes from that file, verbatim; nothing is rounded toward a preferred value.

- [ ] **Step 1: Update the spec**

Change `Status: approved` to `Status: implemented`.
Under "Revisions", replace "None yet." with a dated entry for every change made during implementation, with its reason; if none, write "None."

- [ ] **Step 2: Update the README**

Edit, one sentence per line:

1. **Status:** milestone 4 adds the link regression and signal-integrity tool, host-side only.
2. **Results:** a new "Milestone 4: link regression and signal integrity" subsection with:
   - how to reproduce: `pio test -e native -f test_hil -v`, and the committed output path;
   - the channel model and what it does not model, in brief;
   - the raw `LINK`, `INTEGRITY`, `TRANSPARENT`, `FAULT`, `E2E` and `REG` lines, as printed;
   - a table per group derived from them;
   - what the numbers do and do not show, written from the data: that link delivery is a channel property and matches or does not match `(1 - p)^360`; the undetected count against its bound; which build delivers a larger share of what it queued and why, from the numbers (queue drops versus channel loss); and that the fault group re-checks milestone 3 results and adds no new evidence;
   - the regression pass count out of 21, stated plainly, including any failing row.
3. **Known limitations:** add independent (not burst) bit errors; LoRa coding rate and interleaving not modeled; the board target not built; the suite's new run time.
4. **Deferred to later milestones:** add the board target for the regression tool (Teensy node plus gateway over USB serial, attenuators optional), per the spec's "Target contract".

- [ ] **Step 3: Check the numbers**

Run: `grep -E '^(LINK|INTEGRITY|TRANSPARENT|FAULT|E2E|REG),' docs/results/milestone-4-hil-regression.txt | while read -r line; do grep -qF "$line" README.md || echo "MISSING: $line"; done`
Expected: no output.

- [ ] **Step 4: Check the writing rules**

Run: `grep -n $'\xe2\x80\x94' README.md docs/superpowers/specs/2026-09-30-milestone-4-hil-regression-design.md docs/superpowers/plans/2026-09-30-milestone-4-hil-regression.md`
Expected: no output.

- [ ] **Step 5: Final verification**

Run: `pio test -e native && pio run -e node_polling -e node_interrupt -e node_duty_cycled -e gateway`, then both guards.
Expected: every suite PASSES, all four images build, both guards print nothing.

- [ ] **Step 6: Commit**

```bash
git add README.md docs/superpowers/specs/2026-09-30-milestone-4-hil-regression-design.md
git commit -m "docs: publish the milestone 4 link regression results"
```
