# Milestone 5 Board Target Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Build the board target for the milestone 4 link regression tool: a `TX` line from the node, shared `REC`/`ERR,decode` formatting at the gateway, simulated node and gateway transcripts from `PowerRig`, and a Python `floodnet-hil` tool that captures and analyzes both boards' serial logs, cross-checked exactly against the simulation.

**Architecture:** `InterruptSampler` reports each transmit's end through an optional `ITxListener`; the board prints it, the simulation writes it to `node.log`. The gateway's line formatting moves into `lib/floodnet_core` so the board and the simulation print identical `REC` lines. A native test writes eight frozen scenarios' transcripts and `expected.json`; `pytest` runs the host tool on them and requires exact agreement.

**Tech Stack:** C++17 (`-std=gnu++17 -Wall -Wextra`), PlatformIO `native` and Teensy envs, Unity; Python >= 3.10, pyserial, pytest, ruff.

**Spec:** `docs/superpowers/specs/2026-09-30-milestone-5-board-target-design.md` (read its "Revisions": runs end quiet, and frames orphaned by a reboot)

## Global Constraints

- Build flags stay `-std=gnu++17 -Wall -Wextra`; new C++ compiles without warnings.
- Nothing under `lib/` may include `Arduino.h`, `Wire.h`, or `SPI.h`. No heap allocation in `lib/` or `src/`.
- `TX` line: `TX,<node_id>,<boot_count>,<seq>,<millis>,<ok|timeout>`.
- `REC` field order, unchanged: `REC,node_id,seq,time_ms,lat_1e7,lon_1e7,alt_mm,satellites,yaw_cd,pitch_cd,roll_cd,drops,crc_errors,rssi,gps_valid,imu_valid,boot_count,tx_timeouts,battery_mv`.
- Decode-error line: `ERR,decode,<count>`.
- Log lines: `<t_ms> <line>`.
- `HIL` line: `HIL,label,tx_ok,tx_timeout,accepted,delivery,decode_errors,short_frames,unmatched_rec,seq_gaps,rssi_min,rssi_median,rssi_max,malformed`.
- Transcripts go to `build/hil-transcripts/<scenario>/` (`node.log`, `gateway.log`, `expected.json`); `build/` is git-ignored.
- The eight frozen scenarios, channel seed `kChannelSeed`, faults at 2 h, interrupt runs 1 h, duty-cycled runs 24 h, exactly as the spec's table.
- The milestone 1 and 2 `BENCH` rows, the milestone 3 `LIFE`/`DOWNTIME` lines and the milestone 4 `test_hil` lines must reproduce byte for byte after every task that touches `src/sampler_interrupt.*`, `src/duty_cycled_node.*` or `test/support/`.
  - Benchmark guard: `diff <(grep -o '^BENCH,[^ ]*' README.md) <(pio test -e native -f test_benchmark -v 2>&1 | grep -o 'BENCH,[^ ]*' | head -12)` prints nothing.
  - Power-bench guard: `diff <(grep -E '^(LIFE|DOWNTIME),' docs/results/milestone-3-power-bench.txt) <(pio test -e native -f test_power_bench -v 2>&1 | grep -E '^(LIFE|DOWNTIME),')` prints nothing.
  - HIL guard: `diff <(grep -E '^(LINK|INTEGRITY|TRANSPARENT|FAULT|E2E|REG),' docs/results/milestone-4-hil-regression.txt) <(pio test -e native -f test_hil -v 2>&1 | grep -E '^(LINK|INTEGRITY|TRANSPARENT|FAULT|E2E|REG),')` prints nothing.
- Every harness assertion must be shown to fail on a deliberately broken local edit, then the edit reverted.
- Nothing measured is tuned after it is seen; if the cross-check disagrees, find the defect, never adjust a count.
- Never use the em dash character; use "-". Markdown: one sentence per physical line.
- Commit messages use `feat:`/`fix:`/`test:`/`docs:`/`refactor:` prefixes, no co-author trailer.
- Local Python: use a venv at `tools/hil/.venv` (git-ignored via `.venv/`).

## Review Focus

1. A log whose last line has no newline (a capture cut off mid-write) must count that line as malformed, not parse it. Pinned in Task 5 (`test_unterminated_final_line_is_malformed`).
2. A frame in flight when the watchdog reboots the node completes with no `TX` line; the rig must count it as orphaned so the cross-check still balances. Pinned in Task 3 (`test_listener_survives_a_reboot`) and Task 6 (hang scenario).
3. A run that stops between radio completion and the sampler observing it would leave a `REC` without a `TX`. Pinned in Task 3 (`test_run_until_quiet_ends_with_nothing_in_flight`).
4. A serial port that cannot be opened must fail with the port name, not a traceback. Pinned in Task 5 (`test_capture_reports_a_port_it_cannot_open`).
5. Repeated `REC` lines for one key and an `ERR` line of unknown kind must not inflate `accepted` or pass silently. Pinned in Task 5 (`test_duplicate_rec_counts_once`, `test_unknown_err_kind_is_malformed`).

---

### Task 1: Transmit listener in the sampler, and the node's `TX` line

**Files:**
- Create: `lib/floodnet_hal/include/floodnet/hal/tx_listener.hpp`
- Modify: `src/sampler_interrupt.hpp`, `src/sampler_interrupt.cpp`, `src/duty_cycled_node.hpp`, `src/main.cpp`
- Test: `test/test_tx_listener/test_main.cpp`

**Interfaces:**
- Produces:
  - `class ITxListener { virtual void on_tx_end(uint16_t node_id, uint16_t boot_count, uint32_t seq, uint32_t now_ms, bool completed) = 0; };`
  - `void InterruptSampler::set_tx_listener(ITxListener *listener);`
  - `void DutyCycledNode::set_tx_listener(ITxListener *listener);`

- [ ] **Step 1: Write the failing tests**

Create `test/test_tx_listener/test_main.cpp`:

```cpp
#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include <vector>

#include <unity.h>

#include <floodnet/hal/tx_listener.hpp>
#include <floodnet/packet.hpp>

#include "../support/fake_gps.hpp"
#include "../support/fake_imu.hpp"
#include "../support/fake_radio.hpp"
#include "../support/sim_clock.hpp"
#include "sampler_interrupt.hpp"

using namespace floodnet;

void setUp(void) {}
void tearDown(void) {}

static const double kGpsByteRate = 0.96;
static const char kSentence[] =
    "$GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,*47\r\n";
static const uint32_t kImuMs = 2;
static const uint32_t kRadioMs = 5;

struct Event {
    uint16_t node_id;
    uint16_t boot_count;
    uint32_t seq;
    uint32_t now_ms;
    bool completed;
};

struct Recorder : public ITxListener {
    std::vector<Event> events;
    void on_tx_end(uint16_t node_id, uint16_t boot_count, uint32_t seq, uint32_t now_ms,
                   bool completed) override {
        Event e = {node_id, boot_count, seq, now_ms, completed};
        events.push_back(e);
    }
};

struct Rig {
    SimClock clock;
    FakeGps gps;
    FakeImu imu;
    FakeRadio radio;
    InterruptSampler sampler;

    explicit Rig(WireVersion version)
        : clock(),
          gps(kSentence, kGpsByteRate, FakeGps::MAX_FIFO_DEPTH),
          imu(clock, kImuMs),
          radio(clock, kRadioMs),
          sampler(gps, imu, radio, clock, 0x0042, 3, version) {
        clock.add_observer(&gps);
        clock.add_observer(&imu);
        clock.add_observer(&radio);
    }

    void run_for(uint32_t duration_ms) {
        const uint32_t end = clock.now_ms() + duration_ms;
        while (clock.now_ms() < end) {
            sampler.step();
        }
    }
};

void test_completed_transmits_are_reported(void) {
    Rig rig(WireVersion::V1);
    Recorder rec;
    rig.sampler.set_tx_listener(&rec);
    rig.run_for(2000);
    TEST_ASSERT_TRUE(rec.events.size() > 0);
    TEST_ASSERT_EQUAL_UINT32(rig.sampler.packets_sent(), rec.events.size());
    for (size_t i = 0; i < rec.events.size(); ++i) {
        TEST_ASSERT_TRUE(rec.events[i].completed);
        TEST_ASSERT_EQUAL_UINT16(0x0042, rec.events[i].node_id);
        TEST_ASSERT_EQUAL_UINT16(0, rec.events[i].boot_count);
    }
}

void test_reported_seq_is_the_transmitted_frame(void) {
    Rig rig(WireVersion::V1);
    Recorder rec;
    rig.sampler.set_tx_listener(&rec);
    rig.run_for(2000);
    // Stop at a pass boundary where the last frame's end has been observed.
    while (rig.sampler.tx_in_flight()) {
        rig.sampler.step();
    }
    Packet last;
    TEST_ASSERT_TRUE(decode_packet(rig.radio.last_payload(), rig.radio.last_length(), &last));
    TEST_ASSERT_EQUAL_UINT32(last.seq, rec.events.back().seq);
}

void test_v2_reports_the_boot_count(void) {
    Rig rig(WireVersion::V2);
    Recorder rec;
    rig.sampler.set_tx_listener(&rec);
    rig.sampler.set_node_status(7, 3700);
    rig.run_for(2000);
    TEST_ASSERT_TRUE(rec.events.size() > 0);
    TEST_ASSERT_EQUAL_UINT16(7, rec.events.back().boot_count);
}

void test_timeout_is_reported_as_not_completed(void) {
    Rig rig(WireVersion::V1);
    Recorder rec;
    rig.sampler.set_tx_listener(&rec);
    rig.radio.lose_next_completion();
    rig.run_for(InterruptSampler::TX_TIMEOUT_MS + 1000);
    TEST_ASSERT_EQUAL_UINT16(1, rig.sampler.tx_timeouts());
    TEST_ASSERT_TRUE(rec.events.size() > 0);
    TEST_ASSERT_FALSE(rec.events[0].completed);
    TEST_ASSERT_TRUE(rec.events[0].now_ms >= InterruptSampler::TX_TIMEOUT_MS);
}

void test_now_ms_is_the_clock_at_the_end(void) {
    Rig rig(WireVersion::V1);
    Recorder rec;
    rig.sampler.set_tx_listener(&rec);
    rig.run_for(2000);
    TEST_ASSERT_TRUE(rec.events.size() > 1);
    TEST_ASSERT_TRUE(rec.events[0].now_ms >= kRadioMs);
    TEST_ASSERT_TRUE(rec.events.back().now_ms <= rig.clock.now_ms());
}

void test_no_listener_changes_nothing(void) {
    Rig plain(WireVersion::V1);
    Rig heard(WireVersion::V1);
    Recorder rec;
    heard.sampler.set_tx_listener(&rec);
    plain.run_for(5000);
    heard.run_for(5000);
    TEST_ASSERT_EQUAL_UINT32(plain.sampler.packets_sent(), heard.sampler.packets_sent());
    TEST_ASSERT_EQUAL_UINT32(plain.sampler.next_seq(), heard.sampler.next_seq());
    TEST_ASSERT_EQUAL_UINT32(plain.radio.last_length(), heard.radio.last_length());
    TEST_ASSERT_EQUAL_MEMORY(plain.radio.last_payload(), heard.radio.last_payload(),
                             plain.radio.last_length());
}

int main(int, char **) {
    UNITY_BEGIN();
    RUN_TEST(test_completed_transmits_are_reported);
    RUN_TEST(test_reported_seq_is_the_transmitted_frame);
    RUN_TEST(test_v2_reports_the_boot_count);
    RUN_TEST(test_timeout_is_reported_as_not_completed);
    RUN_TEST(test_now_ms_is_the_clock_at_the_end);
    RUN_TEST(test_no_listener_changes_nothing);
    return UNITY_END();
}
```

- [ ] **Step 2: Run to verify it fails**

Run: `pio test -e native -f test_tx_listener`
Expected: compile failure, `floodnet/hal/tx_listener.hpp` not found.

- [ ] **Step 3: Implement**

Create `lib/floodnet_hal/include/floodnet/hal/tx_listener.hpp`:

```cpp
#ifndef FLOODNET_HAL_TX_LISTENER_HPP
#define FLOODNET_HAL_TX_LISTENER_HPP

#include <stdint.h>

namespace floodnet {

/// Told when a transmit ends. `completed` is true when the radio confirmed
/// the frame and false when the sampler abandoned it after its timeout; an
/// abandoned frame may or may not have reached the air.
class ITxListener {
  public:
    virtual ~ITxListener() {}
    virtual void on_tx_end(uint16_t node_id, uint16_t boot_count, uint32_t seq, uint32_t now_ms,
                           bool completed) = 0;
};

}  // namespace floodnet

#endif  // FLOODNET_HAL_TX_LISTENER_HPP
```

In `src/sampler_interrupt.hpp`:
- add `#include <floodnet/hal/tx_listener.hpp>` after `#include <floodnet/hal/radio.hpp>`;
- add after `bool last_record_imu_valid() const { return last_record_imu_valid_; }`:

```cpp

    /// Told when each transmit ends, completed or abandoned. Default none, and
    /// with none installed the sampler behaves exactly as before.
    void set_tx_listener(ITxListener *listener) { tx_listener_ = listener; }
```

- add `void notify_tx_end(bool completed);` after `bool service_radio();`;
- add after `bool last_record_imu_valid_;`:

```cpp
    ITxListener *tx_listener_;
    uint32_t tx_seq_;   ///< seq of the frame in flight
    uint16_t tx_boot_;  ///< boot_count stamped into the frame in flight
```

In `src/sampler_interrupt.cpp`:
- extend the initialiser list: replace `last_record_imu_valid_(false) {}` with

```cpp
      last_record_imu_valid_(false),
      tx_listener_(nullptr),
      tx_seq_(0),
      tx_boot_(0) {}
```

- in `service_radio()`, replace

```cpp
    if (tx_in_flight_ && !busy) {
        tx_in_flight_ = false;
        ++packets_sent_;
        return true;
    }
```

with

```cpp
    if (tx_in_flight_ && !busy) {
        tx_in_flight_ = false;
        ++packets_sent_;
        notify_tx_end(true);
        return true;
    }
```

- in the timeout branch, replace

```cpp
        if (tx_timeouts_ != 0xFFFF) {
            ++tx_timeouts_;
        }
        return true;
```

with

```cpp
        if (tx_timeouts_ != 0xFFFF) {
            ++tx_timeouts_;
        }
        notify_tx_end(false);
        return true;
```

- replace

```cpp
    tx_in_flight_ = true;
    tx_started_ms_ = clock_.now_ms();
    return true;
}
```

with

```cpp
    tx_in_flight_ = true;
    tx_started_ms_ = clock_.now_ms();
    tx_seq_ = packet.seq;
    tx_boot_ = packet.boot_count;
    return true;
}

void InterruptSampler::notify_tx_end(bool completed) {
    if (tx_listener_ != nullptr) {
        tx_listener_->on_tx_end(node_id_, tx_boot_, tx_seq_, clock_.now_ms(), completed);
    }
}
```

In `src/duty_cycled_node.hpp`, add after `const InterruptSampler &sampler() const { return sampler_; }`:

```cpp

    /// Passed to the sampler, which does the transmitting.
    void set_tx_listener(ITxListener *listener) { sampler_.set_tx_listener(listener); }
```

In `src/main.cpp`, inside the anonymous namespace after the `#endif` that closes the sampler selection, add:

```cpp

#if defined(FLOODNET_NODE_DUTY_CYCLED) || defined(FLOODNET_SAMPLER_INTERRUPT)
/// Prints one line per transmit end, for the link regression tool's board
/// target: TX,<node_id>,<boot_count>,<seq>,<millis>,<ok|timeout>.
class SerialTxListener : public floodnet::ITxListener {
  public:
    void on_tx_end(uint16_t node_id, uint16_t boot_count, uint32_t seq, uint32_t now_ms,
                   bool completed) override {
        Serial.print("TX,");
        Serial.print(node_id);
        Serial.print(',');
        Serial.print(boot_count);
        Serial.print(',');
        Serial.print(seq);
        Serial.print(',');
        Serial.print(now_ms);
        Serial.print(',');
        Serial.println(completed ? "ok" : "timeout");
    }
};
SerialTxListener g_tx_listener;
#endif
```

In `setup()`, replace

```cpp
#if defined(FLOODNET_NODE_DUTY_CYCLED)
    g_node.begin();
#endif
```

with

```cpp
#if defined(FLOODNET_NODE_DUTY_CYCLED)
    g_node.set_tx_listener(&g_tx_listener);
    g_node.begin();
#elif defined(FLOODNET_SAMPLER_INTERRUPT)
    g_sampler.set_tx_listener(&g_tx_listener);
#endif
```

- [ ] **Step 4: Run to verify it passes**

Run: `pio test -e native -f test_tx_listener`
Expected: 6 tests PASS, no warnings.

- [ ] **Step 5: Guards, suite and firmware**

Run the benchmark, power-bench and HIL guards; each prints nothing.
Run: `pio test -e native` and `pio run -e node_polling -e node_interrupt -e node_duty_cycled -e gateway`
Expected: every suite PASSES; all four images build with no warnings from project files.

- [ ] **Step 6: Show the assertions can fail**

Temporarily delete `notify_tx_end(false);`; `test_timeout_is_reported_as_not_completed` must FAIL.
Temporarily delete `tx_seq_ = packet.seq;`; `test_reported_seq_is_the_transmitted_frame` must FAIL.
Revert both and rerun to PASS.

- [ ] **Step 7: Commit**

```bash
git add lib/floodnet_hal/include/floodnet/hal/tx_listener.hpp src/sampler_interrupt.hpp src/sampler_interrupt.cpp src/duty_cycled_node.hpp src/main.cpp test/test_tx_listener/test_main.cpp
git commit -m "feat: report each transmit's end, and print it as a TX line on the node"
```

---

### Task 2: Shared gateway line formatting

**Files:**
- Create: `lib/floodnet_core/include/floodnet/gateway_format.hpp`, `lib/floodnet_core/src/gateway_format.cpp`
- Modify: `src/gateway_main.cpp`
- Test: `test/test_gateway_format/test_main.cpp`

**Interfaces:**
- Consumes: `Packet` (`<floodnet/packet.hpp>`).
- Produces:
  - `const size_t REC_LINE_MAX = 256;`
  - `size_t format_rec_line(const Packet &p, int16_t rssi, char *out, size_t out_len);`
  - `size_t format_decode_error_line(uint16_t count, char *out, size_t out_len);`

- [ ] **Step 1: Write the failing tests**

Create `test/test_gateway_format/test_main.cpp`:

```cpp
#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include <unity.h>

#include <floodnet/gateway_format.hpp>
#include <floodnet/packet.hpp>

using namespace floodnet;

void setUp(void) {}
void tearDown(void) {}

static Packet full_packet() {
    Packet p;
    p.node_id = 0x0042;
    p.seq = 123456;
    p.record.gps.time_ms = 987654;
    p.record.gps.lat_1e7 = 481173000;
    p.record.gps.lon_1e7 = -115166667;
    p.record.gps.alt_mm = -1200;
    p.record.gps.satellites = 8;
    p.record.gps.valid = true;
    p.record.imu.yaw_cd = -4500;
    p.record.imu.pitch_cd = 1234;
    p.record.imu.roll_cd = -1;
    p.record.imu.valid = false;
    p.record.diag.drops = 7;
    p.record.diag.crc_errors = 0;
    p.boot_count = 3;
    p.tx_timeouts = 2;
    p.battery_mv = 3700;
    return p;
}

void test_rec_line_matches_the_gateway_field_order(void) {
    char line[REC_LINE_MAX];
    const size_t n = format_rec_line(full_packet(), -97, line, sizeof(line));
    TEST_ASSERT_EQUAL_STRING(
        "REC,66,123456,987654,481173000,-115166667,-1200,8,-4500,1234,-1,7,0,-97,1,0,3,2,3700",
        line);
    TEST_ASSERT_EQUAL_UINT32(strlen(line), n);
}

void test_rec_line_returns_zero_when_the_buffer_is_too_small(void) {
    char line[10];
    TEST_ASSERT_EQUAL_UINT32(0, format_rec_line(full_packet(), -97, line, sizeof(line)));
}

void test_decode_error_line(void) {
    char line[32];
    const size_t n = format_decode_error_line(5, line, sizeof(line));
    TEST_ASSERT_EQUAL_STRING("ERR,decode,5", line);
    TEST_ASSERT_EQUAL_UINT32(12, n);
}

void test_decode_error_line_returns_zero_when_the_buffer_is_too_small(void) {
    char line[4];
    TEST_ASSERT_EQUAL_UINT32(0, format_decode_error_line(5, line, sizeof(line)));
}

int main(int, char **) {
    UNITY_BEGIN();
    RUN_TEST(test_rec_line_matches_the_gateway_field_order);
    RUN_TEST(test_rec_line_returns_zero_when_the_buffer_is_too_small);
    RUN_TEST(test_decode_error_line);
    RUN_TEST(test_decode_error_line_returns_zero_when_the_buffer_is_too_small);
    return UNITY_END();
}
```

- [ ] **Step 2: Run to verify it fails**

Run: `pio test -e native -f test_gateway_format`
Expected: compile failure, `floodnet/gateway_format.hpp` not found.

- [ ] **Step 3: Implement**

Create `lib/floodnet_core/include/floodnet/gateway_format.hpp`:

```cpp
#ifndef FLOODNET_GATEWAY_FORMAT_HPP
#define FLOODNET_GATEWAY_FORMAT_HPP

#include <stddef.h>
#include <stdint.h>

#include <floodnet/packet.hpp>

namespace floodnet {

/// Enough for a REC line with every field at its widest.
const size_t REC_LINE_MAX = 256;

/// The gateway's REC line, without a line ending. The board prints it with
/// Serial.println and the simulation writes it to a transcript, so both
/// produce the same bytes. Returns the length written, or 0 if `out_len` is
/// too small.
size_t format_rec_line(const Packet &p, int16_t rssi, char *out, size_t out_len);

/// "ERR,decode,<count>", without a line ending. Returns the length written,
/// or 0 if `out_len` is too small.
size_t format_decode_error_line(uint16_t count, char *out, size_t out_len);

}  // namespace floodnet

#endif  // FLOODNET_GATEWAY_FORMAT_HPP
```

Create `lib/floodnet_core/src/gateway_format.cpp`:

```cpp
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
```

In `src/gateway_main.cpp`:
- add `#include <floodnet/gateway_format.hpp>` before `#include <floodnet/mesh.hpp>`;
- replace the whole body of `print_record()` (from `Serial.print("REC,");` through `Serial.println(p.battery_mv);`) with:

```cpp
    char line[floodnet::REC_LINE_MAX];
    if (floodnet::format_rec_line(p, rssi, line, sizeof(line)) > 0) {
        Serial.println(line);
    }
```

  and keep the comment block above it that explains the one-line-per-packet format;
- replace

```cpp
        Serial.print("ERR,decode,");
        Serial.println(g_decode_errors);
```

with

```cpp
        char line[32];
        if (floodnet::format_decode_error_line(g_decode_errors, line, sizeof(line)) > 0) {
            Serial.println(line);
        }
```

- [ ] **Step 4: Run to verify it passes**

Run: `pio test -e native -f test_gateway_format`
Expected: 4 tests PASS.
Run: `pio run -e gateway`
Expected: SUCCESS, no warnings from project files.

- [ ] **Step 5: Show the assertions can fail**

Temporarily swap `%d,%d,%d,%u,%u,%u` at the end of the REC format to `%d,%d,%d,%u,%u,%u,` (a trailing comma); `test_rec_line_matches_the_gateway_field_order` must FAIL. Revert and rerun to PASS.

- [ ] **Step 6: Commit**

```bash
git add lib/floodnet_core/include/floodnet/gateway_format.hpp lib/floodnet_core/src/gateway_format.cpp src/gateway_main.cpp test/test_gateway_format/test_main.cpp
git commit -m "refactor: format the gateway's REC and decode-error lines in shared code"
```

---

### Task 3: `PowerRig` support for transcripts

**Files:**
- Modify: `test/support/power_rig.hpp`
- Test: `test/test_rig_transcript/test_main.cpp`

**Interfaces:**
- Consumes: `ITxListener`, `InterruptSampler::set_tx_listener`, `DutyCycledNode::set_tx_listener` (Task 1); `format_rec_line`, `format_decode_error_line`, `REC_LINE_MAX` (Task 2).
- Produces:
  - `void PowerRig::set_tx_listener(ITxListener *listener);` (survives reboots)
  - `void PowerRig::set_gateway_log(std::function<void(uint32_t t_ms, const char *line)> sink);`
  - `void PowerRig::run_until_quiet(uint32_t end_ms);` (then one pass at a time until nothing is in flight, at most `kRigQuietLimitMs` = 10000 past `end_ms`)
  - `bool PowerRig::tx_in_flight() const;`
  - `size_t PowerRig::decode_failures() const;`
  - `size_t PowerRig::tx_orphaned() const;` (radio completions of frames started before a reboot)
  - `size_t PowerRig::unmatched_accepted() const;` (accepted orphans plus accepted frames whose key the channel altered)
  - `uint32_t PowerRig::tx_timeouts_total() const;` (sampler timeouts summed over every boot)

- [ ] **Step 1: Write the failing tests**

Create `test/test_rig_transcript/test_main.cpp`:

```cpp
#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include <set>
#include <string>
#include <vector>

#include <unity.h>

#include <floodnet/hal/tx_listener.hpp>

#include "../support/noisy_channel.hpp"
#include "../support/power_experiments.hpp"
#include "../support/power_rig.hpp"

using namespace floodnet;

void setUp(void) {}
void tearDown(void) {}

struct Recorder : public ITxListener {
    size_t completed = 0;
    size_t abandoned = 0;
    std::set<uint16_t> boots;
    void on_tx_end(uint16_t, uint16_t boot_count, uint32_t, uint32_t, bool done) override {
        (done ? completed : abandoned) += 1;
        boots.insert(boot_count);
    }
};

void test_run_until_quiet_ends_with_nothing_in_flight(void) {
    PowerRig rig(Build::Interrupt, kFaultRunSleepNA, kFaultRunCapacityNaMs);
    rig.run_until_quiet(600000UL);
    TEST_ASSERT_FALSE(rig.tx_in_flight());
    TEST_ASSERT_TRUE(rig.clock().now_ms() >= 600000UL);
}

void test_completed_listener_events_equal_radio_completions(void) {
    PowerRig rig(Build::Interrupt, kFaultRunSleepNA, kFaultRunCapacityNaMs);
    Recorder rec;
    rig.set_tx_listener(&rec);
    rig.run_until_quiet(600000UL);
    TEST_ASSERT_TRUE(rec.completed > 0);
    TEST_ASSERT_EQUAL_UINT32(rig.radio().sent_count() - rig.tx_orphaned(), rec.completed);
    TEST_ASSERT_EQUAL_UINT32(rig.tx_timeouts_total(), rec.abandoned);
}

void test_listener_survives_a_reboot(void) {
    PowerRig rig(Build::DutyCycled, kFaultRunSleepNA, kFaultRunCapacityNaMs);
    Recorder rec;
    rig.set_tx_listener(&rec);
    rig.add_fault(FaultKind::Hang, kFaultAtMs, 0);
    rig.run_until_quiet(kFaultRunMs);
    TEST_ASSERT_EQUAL_UINT32(1, rig.reboots());
    TEST_ASSERT_EQUAL_UINT32(2, rec.boots.size());
    TEST_ASSERT_EQUAL_UINT32(rig.radio().sent_count() - rig.tx_orphaned(), rec.completed);
}

void test_lost_completion_is_a_timeout(void) {
    PowerRig rig(Build::DutyCycled, kFaultRunSleepNA, kFaultRunCapacityNaMs);
    Recorder rec;
    rig.set_tx_listener(&rec);
    rig.add_fault(FaultKind::LostCompletion, kFaultAtMs, 0);
    rig.run_until_quiet(kFaultRunMs);
    TEST_ASSERT_EQUAL_UINT32(1, rig.tx_timeouts_total());
    TEST_ASSERT_EQUAL_UINT32(1, rec.abandoned);
}

void test_gateway_log_has_a_rec_line_per_delivery(void) {
    PowerRig rig(Build::DutyCycled, kFaultRunSleepNA, kFaultRunCapacityNaMs);
    std::vector<std::string> lines;
    rig.set_gateway_log([&lines](uint32_t, const char *line) { lines.push_back(line); });
    rig.run_until_quiet(6UL * 3600000UL);
    TEST_ASSERT_TRUE(rig.deliveries().size() > 0);
    TEST_ASSERT_EQUAL_UINT32(rig.deliveries().size(), lines.size());
    for (size_t i = 0; i < lines.size(); ++i) {
        TEST_ASSERT_EQUAL_INT(0, strncmp(lines[i].c_str(), "REC,", 4));
    }
}

void test_every_corrupted_frame_is_a_decode_error_line(void) {
    NoisyChannel channel(1.0, kChannelSeed);
    PowerRig rig(Build::DutyCycled, kFaultRunSleepNA, kFaultRunCapacityNaMs);
    rig.set_channel(&channel);
    std::vector<std::string> lines;
    rig.set_gateway_log([&lines](uint32_t, const char *line) { lines.push_back(line); });
    rig.run_until_quiet(2UL * 3600000UL);
    TEST_ASSERT_TRUE(rig.radio().sent_count() > 0);
    TEST_ASSERT_EQUAL_UINT32(rig.radio().sent_count(), rig.decode_failures());
    TEST_ASSERT_EQUAL_UINT32(rig.decode_failures(), lines.size());
    TEST_ASSERT_EQUAL_STRING("ERR,decode,1", lines[0].c_str());
}

int main(int, char **) {
    UNITY_BEGIN();
    RUN_TEST(test_run_until_quiet_ends_with_nothing_in_flight);
    RUN_TEST(test_completed_listener_events_equal_radio_completions);
    RUN_TEST(test_listener_survives_a_reboot);
    RUN_TEST(test_lost_completion_is_a_timeout);
    RUN_TEST(test_gateway_log_has_a_rec_line_per_delivery);
    RUN_TEST(test_every_corrupted_frame_is_a_decode_error_line);
    return UNITY_END();
}
```

- [ ] **Step 2: Run to verify it fails**

Run: `pio test -e native -f test_rig_transcript`
Expected: compile failure, `no member named 'set_tx_listener'` (or `run_until_quiet`).

- [ ] **Step 3: Implement**

In `test/support/power_rig.hpp`:

Add after `#include <memory>`:

```cpp
#include <functional>
```

Add after `#include <floodnet/downtime.hpp>`:

```cpp
#include <floodnet/gateway_format.hpp>
#include <floodnet/hal/tx_listener.hpp>
```

Add after `const size_t kRigMaxFrame = 64;`:

```cpp

/// How far past its end time run_until_quiet() may go to let a transmit in
/// flight finish. SF12 airtime is about 3 s and the sampler's timeout 5 s.
const uint32_t kRigQuietLimitMs = 10000;
```

Extend the constructor's initialiser list: replace

```cpp
          channel_(nullptr),
          queued_before_boot_(0) {
```

with

```cpp
          channel_(nullptr),
          queued_before_boot_(0),
          tx_listener_(nullptr),
          decode_failures_(0),
          orphans_(0),
          unmatched_accepted_(0),
          timeouts_before_boot_(0) {
```

Replace `run_until()`:

```cpp
    /// Runs until `end_ms` or until the battery is empty, whichever is first.
    void run_until(uint32_t end_ms) {
        end_.set_end(end_ms);
        while (clock_.now_ms() < end_ms && !power_.depleted()) {
            apply_faults();
            if (node_) {
                node_->step();
            } else {
                sampler_->step();
            }
            collect_delivery();
            if (node_ && watchdog_.expired()) {
                reboot();
            }
        }
    }
```

with

```cpp
    /// Runs until `end_ms` or until the battery is empty, whichever is first.
    void run_until(uint32_t end_ms) {
        end_.set_end(end_ms);
        while (clock_.now_ms() < end_ms && !power_.depleted()) {
            step_once();
        }
    }

    /// run_until(end_ms), then one pass at a time until no transmit is in
    /// flight, at most kRigQuietLimitMs past `end_ms`. A transcript that ends
    /// between the radio completing a frame and the sampler observing it
    /// would show a REC line with no TX line.
    void run_until_quiet(uint32_t end_ms) {
        run_until(end_ms);
        const uint32_t limit = end_ms + kRigQuietLimitMs;
        end_.set_end(limit);
        while (tx_in_flight() && clock_.now_ms() < limit && !power_.depleted()) {
            step_once();
        }
    }
```

Add after the `records_queued()` accessor:

```cpp

    /// Told of every transmit's end by whichever node is running, including
    /// one booted after a watchdog reset.
    void set_tx_listener(ITxListener *listener) {
        tx_listener_ = listener;
        if (node_) {
            node_->set_tx_listener(listener);
        } else {
            sampler_->set_tx_listener(listener);
        }
    }

    /// Receives the gateway's lines, exactly as the board would print them,
    /// with the rig's clock. RSSI is 0: nothing models it.
    void set_gateway_log(std::function<void(uint32_t t_ms, const char *line)> sink) {
        gateway_log_ = sink;
    }

    bool tx_in_flight() const {
        return node_ ? node_->sampler().tx_in_flight() : sampler_->tx_in_flight();
    }

    size_t decode_failures() const { return decode_failures_; }

    /// Radio completions of frames started by a node that has since been
    /// reset: no sampler is left to report them.
    size_t tx_orphaned() const { return orphans_; }

    /// Accepted frames that no TX line will match: orphans, and frames whose
    /// (node_id, boot_count, seq) the channel altered without the CRC noticing.
    size_t unmatched_accepted() const { return unmatched_accepted_; }

    uint32_t tx_timeouts_total() const {
        const uint32_t current =
            node_ ? node_->sampler().tx_timeouts() : sampler_->tx_timeouts();
        return timeouts_before_boot_ + current;
    }
```

In `boot()`, replace

```cpp
        node_.reset(new DutyCycledNode(gps_, imu_, radio_, clock_, power_, watchdog_, store_,
                                       config()));
        node_->begin();
```

with

```cpp
        node_.reset(new DutyCycledNode(gps_, imu_, radio_, clock_, power_, watchdog_, store_,
                                       config()));
        node_->set_tx_listener(tx_listener_);
        node_->begin();
```

In `reboot()`, after `queued_before_boot_ += node_->sampler().next_seq();` add:

```cpp
        timeouts_before_boot_ += node_->sampler().tx_timeouts();
```

Replace `collect_delivery()` with:

```cpp
    void step_once() {
        apply_faults();
        if (node_) {
            node_->step();
        } else {
            sampler_->step();
        }
        collect_delivery();
        if (node_ && watchdog_.expired()) {
            reboot();
        }
    }

    void collect_delivery() {
        if (radio_.sent_count() == last_sent_) {
            return;
        }
        last_sent_ = radio_.sent_count();

        // What was transmitted, before the channel: its key, and whether a
        // node that has since been reset started it.
        Packet sent;
        const bool sent_decodes = decode_packet(radio_.last_payload(), radio_.last_length(), &sent);
        const bool orphan = node_ && sent_decodes && sent.boot_count != node_->boot_count();
        if (orphan) {
            ++orphans_;
        }

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
            ++decode_failures_;
            if (gateway_log_) {
                char line[32];
                const uint16_t count =
                    decode_failures_ > 0xFFFF ? 0xFFFF : static_cast<uint16_t>(decode_failures_);
                if (format_decode_error_line(count, line, sizeof(line)) > 0) {
                    gateway_log_(clock_.now_ms(), line);
                }
            }
            return;
        }
        // The gateway's own dedup, so a key bug would show up as lost records.
        if (dedup_.seen(packet.node_id, packet.boot_count, packet.seq)) {
            return;
        }
        deliveries_.push_back(packet);
        const bool key_altered = sent_decodes && (packet.node_id != sent.node_id ||
                                                  packet.boot_count != sent.boot_count ||
                                                  packet.seq != sent.seq);
        if (orphan || key_altered) {
            ++unmatched_accepted_;
        }
        GatewayRecord record;
        record.arrival_ms = clock_.now_ms();
        record.gps_valid = packet.record.gps.valid;
        record.boot_count = packet.boot_count;
        record.tx_timeouts = packet.tx_timeouts;
        records_.push_back(record);
        if (gateway_log_) {
            char line[REC_LINE_MAX];
            if (format_rec_line(packet, 0, line, sizeof(line)) > 0) {
                gateway_log_(clock_.now_ms(), line);
            }
        }
    }
```

Add the private members after `uint32_t queued_before_boot_;`:

```cpp
    ITxListener *tx_listener_;
    std::function<void(uint32_t, const char *)> gateway_log_;
    size_t decode_failures_;
    size_t orphans_;
    size_t unmatched_accepted_;
    uint32_t timeouts_before_boot_;
```

- [ ] **Step 4: Run to verify it passes**

Run: `pio test -e native -f test_rig_transcript`
Expected: 6 tests PASS, no warnings.

- [ ] **Step 5: Guards and suite**

Run the benchmark, power-bench and HIL guards; each prints nothing.
Run: `pio test -e native`
Expected: every suite PASSES.

- [ ] **Step 6: Show the assertions can fail**

Temporarily delete `node_->set_tx_listener(tx_listener_);` in `boot()`; `test_listener_survives_a_reboot` must FAIL.
Temporarily change `run_until_quiet` to only call `run_until(end_ms);`; `test_run_until_quiet_ends_with_nothing_in_flight` or `test_completed_listener_events_equal_radio_completions` must FAIL (if neither fails, report it: the 600000 ms end did not land mid-transmit, and pick an end that does, recording the ruling).
Revert both and rerun to PASS.

- [ ] **Step 7: Commit**

```bash
git add test/support/power_rig.hpp test/test_rig_transcript/test_main.cpp
git commit -m "test: let the power rig report transmit ends and gateway lines for transcripts"
```

---

### Task 4: Simulated transcripts

**Files:**
- Create: `test/test_hil_transcripts/test_main.cpp`
- Modify: `.gitignore`

**Interfaces:**
- Consumes: everything from Task 3; `NoisyChannel`, `kChannelSeed`; `kFaultAtMs` (power_experiments).
- Produces: `build/hil-transcripts/<scenario>/node.log`, `gateway.log`, `expected.json` for the eight scenarios; `expected.json` keys `scenario`, `build`, `p`, `tx_ok`, `tx_timeout`, `tx_orphaned`, `rec_lines`, `unmatched_rec`, `decode_errors`.

- [ ] **Step 1: Write the transcript writer**

Append to `.gitignore`:

```text
build/
.venv/
```

Create `test/test_hil_transcripts/test_main.cpp`:

```cpp
#include <errno.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/stat.h>

#include <string>

#include <unity.h>

#include <floodnet/hal/tx_listener.hpp>

#include "../support/noisy_channel.hpp"
#include "../support/power_experiments.hpp"
#include "../support/power_rig.hpp"

using namespace floodnet;

void setUp(void) {}
void tearDown(void) {}

// Frozen in the milestone 5 design doc, "Frozen scenarios".
struct Scenario {
    const char *name;
    Build build;
    double p;
    bool has_fault;
    FaultKind fault;
    uint32_t run_ms;
};

static const uint32_t kHourMs = 3600000UL;
static const Scenario kScenarios[] = {
    {"interrupt_p0", Build::Interrupt, 0.0, false, FaultKind::Hang, kHourMs},
    {"interrupt_p1e-3", Build::Interrupt, 1e-3, false, FaultKind::Hang, kHourMs},
    {"interrupt_p1e-2", Build::Interrupt, 1e-2, false, FaultKind::Hang, kHourMs},
    {"duty_cycled_p0", Build::DutyCycled, 0.0, false, FaultKind::Hang, 24 * kHourMs},
    {"duty_cycled_p1e-3", Build::DutyCycled, 1e-3, false, FaultKind::Hang, 24 * kHourMs},
    {"duty_cycled_p1e-2", Build::DutyCycled, 1e-2, false, FaultKind::Hang, 24 * kHourMs},
    {"duty_cycled_lost_completion", Build::DutyCycled, 0.0, true, FaultKind::LostCompletion,
     24 * kHourMs},
    {"duty_cycled_hang", Build::DutyCycled, 0.0, true, FaultKind::Hang, 24 * kHourMs},
};
static const size_t kScenarioCount = sizeof(kScenarios) / sizeof(kScenarios[0]);

static std::string out_root() {
    const char *env = getenv("FLOODNET_TRANSCRIPT_DIR");
    return env != nullptr ? env : "build/hil-transcripts";
}

/// mkdir -p.
static bool make_dirs(const std::string &path) {
    for (size_t i = 1; i <= path.size(); ++i) {
        if (i == path.size() || path[i] == '/') {
            const std::string part = path.substr(0, i);
            if (mkdir(part.c_str(), 0755) != 0 && errno != EEXIST) {
                return false;
            }
        }
    }
    return true;
}

/// Writes TX lines exactly as the board's SerialTxListener prints them,
/// prefixed with the rig's clock.
struct FileTxLog : public ITxListener {
    FILE *file;
    size_t lines;
    explicit FileTxLog(FILE *f) : file(f), lines(0) {}
    void on_tx_end(uint16_t node_id, uint16_t boot_count, uint32_t seq, uint32_t now_ms,
                   bool completed) override {
        fprintf(file, "%lu TX,%u,%u,%lu,%lu,%s\n", static_cast<unsigned long>(now_ms),
                static_cast<unsigned>(node_id), static_cast<unsigned>(boot_count),
                static_cast<unsigned long>(seq), static_cast<unsigned long>(now_ms),
                completed ? "ok" : "timeout");
        ++lines;
    }
};

static void write_scenario(const Scenario &s) {
    const std::string dir = out_root() + "/" + s.name;
    TEST_ASSERT_TRUE_MESSAGE(make_dirs(dir), "cannot create the transcript directory");
    FILE *node = fopen((dir + "/node.log").c_str(), "w");
    FILE *gateway = fopen((dir + "/gateway.log").c_str(), "w");
    FILE *expected = fopen((dir + "/expected.json").c_str(), "w");
    TEST_ASSERT_NOT_NULL(node);
    TEST_ASSERT_NOT_NULL(gateway);
    TEST_ASSERT_NOT_NULL(expected);

    NoisyChannel channel(s.p, kChannelSeed);
    PowerRig rig(s.build, kFaultRunSleepNA, kFaultRunCapacityNaMs);
    rig.set_channel(&channel);
    FileTxLog tx_log(node);
    rig.set_tx_listener(&tx_log);
    size_t gateway_lines = 0;
    rig.set_gateway_log([gateway, &gateway_lines](uint32_t t_ms, const char *line) {
        fprintf(gateway, "%lu %s\n", static_cast<unsigned long>(t_ms), line);
        ++gateway_lines;
    });
    if (s.has_fault) {
        rig.add_fault(s.fault, kFaultAtMs, 0);
    }
    rig.run_until_quiet(s.run_ms);

    TEST_ASSERT_TRUE_MESSAGE(rig.clock().now_ms() >= s.run_ms, "run ended early");
    TEST_ASSERT_FALSE_MESSAGE(rig.tx_in_flight(), "run ended with a transmit in flight");
    TEST_ASSERT_TRUE_MESSAGE(tx_log.lines > 0, "node transmitted nothing");
    TEST_ASSERT_EQUAL_UINT32(rig.deliveries().size() + rig.decode_failures(), gateway_lines);
    if (s.has_fault) {
        TEST_ASSERT_TRUE_MESSAGE(rig.fault_started(0), "fault never started");
    }

    const size_t tx_ok = rig.radio().sent_count() - rig.tx_orphaned();
    fprintf(expected,
            "{\"scenario\": \"%s\", \"build\": \"%s\", \"p\": %g, \"tx_ok\": %lu, "
            "\"tx_timeout\": %lu, \"tx_orphaned\": %lu, \"rec_lines\": %lu, "
            "\"unmatched_rec\": %lu, \"decode_errors\": %lu}\n",
            s.name, s.build == Build::Interrupt ? "interrupt" : "duty_cycled", s.p,
            static_cast<unsigned long>(tx_ok), static_cast<unsigned long>(rig.tx_timeouts_total()),
            static_cast<unsigned long>(rig.tx_orphaned()),
            static_cast<unsigned long>(rig.deliveries().size()),
            static_cast<unsigned long>(rig.unmatched_accepted()),
            static_cast<unsigned long>(rig.decode_failures()));
    fclose(node);
    fclose(gateway);
    fclose(expected);
    printf("TRANSCRIPT,%s,%lu,%lu\n", s.name, static_cast<unsigned long>(tx_log.lines),
           static_cast<unsigned long>(gateway_lines));
}

void test_write_every_scenario(void) {
    for (size_t i = 0; i < kScenarioCount; ++i) {
        write_scenario(kScenarios[i]);
    }
}

int main(int, char **) {
    UNITY_BEGIN();
    RUN_TEST(test_write_every_scenario);
    return UNITY_END();
}
```

- [ ] **Step 2: Run it**

Run: `pio test -e native -f test_hil_transcripts -v 2>&1 | grep -E 'TRANSCRIPT,|PASS|FAIL'`
Expected: 1 test PASSES and 8 `TRANSCRIPT` lines.
Run: `ls build/hil-transcripts && head -3 build/hil-transcripts/duty_cycled_p0/node.log build/hil-transcripts/duty_cycled_p0/gateway.log && cat build/hil-transcripts/duty_cycled_hang/expected.json`
Expected: 8 directories at the repository root; `TX,...` and `REC,...` lines prefixed with times; valid JSON.
If the files land somewhere other than `build/hil-transcripts/` at the repository root (PlatformIO may run the test binary from another directory), record where, rule on the fix (for example setting `FLOODNET_TRANSCRIPT_DIR` from `platformio.ini` via `test_testing_command` or resolving an absolute path from `__FILE__`), and ledger it.

- [ ] **Step 3: Show the assertions can fail**

Temporarily replace `rig.run_until_quiet(s.run_ms);` with `rig.run_until(s.run_ms);`; the run must FAIL on "run ended with a transmit in flight" for at least one scenario (if none fails, report it and ledger it).
Temporarily remove `rig.set_tx_listener(&tx_log);`; the run must FAIL on "node transmitted nothing".
Revert both and rerun to PASS.

- [ ] **Step 4: Suite**

Run: `pio test -e native`
Expected: every suite PASSES.

- [ ] **Step 5: Commit**

```bash
git add .gitignore test/test_hil_transcripts/test_main.cpp
git commit -m "test: write simulated node and gateway transcripts for the board target"
```

---

### Task 5: The `floodnet-hil` host tool

**Files:**
- Create: `tools/hil/pyproject.toml`, `tools/hil/src/floodnet_hil/__init__.py`, `tools/hil/src/floodnet_hil/parse.py`, `tools/hil/src/floodnet_hil/analyze.py`, `tools/hil/src/floodnet_hil/capture.py`, `tools/hil/src/floodnet_hil/cli.py`
- Test: `tools/hil/tests/test_parse.py`, `tools/hil/tests/test_analyze.py`, `tools/hil/tests/test_capture.py`, `tools/hil/tests/test_cli.py`

**Interfaces:**
- Produces:
  - `parse.parse_log(text: str) -> ParsedLog`; `ParsedLog(tx: list[TxLine], rec: list[RecLine], decode_errors: int, short_frames: int, malformed: int)`; `TxLine.key`, `RecLine.key` are `(node_id, boot_count, seq)`.
  - `analyze.analyze(node: ParsedLog, gateway: ParsedLog, label: str) -> Summary`; `analyze.analyze_dir(run_dir: Path) -> Summary`; `Summary.line() -> str` (the `HIL,...` line); fields `tx_ok, tx_timeout, accepted, delivery, decode_errors, short_frames, unmatched_rec, seq_gaps, rssi_min, rssi_median, rssi_max, malformed`.
  - `capture.capture(ports: dict[str, object], out_dir: Path, seconds: float, clock=time.monotonic) -> dict[str, int]`.
  - `cli.main(argv: list[str] | None = None) -> int`; console script `floodnet-hil`.

- [ ] **Step 1: Package skeleton and venv**

Create `tools/hil/pyproject.toml`:

```toml
[build-system]
requires = ["setuptools>=68"]
build-backend = "setuptools.build_meta"

[project]
name = "floodnet-hil"
version = "0.1.0"
description = "Capture and analyze FloodNet node and gateway serial logs"
requires-python = ">=3.10"
dependencies = ["pyserial>=3.5"]

[project.optional-dependencies]
test = ["pytest>=8", "ruff>=0.5"]

[project.scripts]
floodnet-hil = "floodnet_hil.cli:main"

[tool.setuptools.packages.find]
where = ["src"]

[tool.ruff]
line-length = 100
target-version = "py310"

[tool.ruff.lint]
select = ["E", "F", "W", "I", "B", "UP"]

[tool.pytest.ini_options]
testpaths = ["tests"]
```

Create `tools/hil/src/floodnet_hil/__init__.py`:

```python
"""FloodNet board-target tools: capture and analyze node and gateway serial logs."""
```

Run: `python3 -m venv tools/hil/.venv && tools/hil/.venv/bin/pip install -q -e 'tools/hil[test]'`
Expected: installs without error.

- [ ] **Step 2: Write the failing parse tests**

Create `tools/hil/tests/test_parse.py`:

```python
from floodnet_hil.parse import parse_log

REC = "REC,66,5,1000,481173000,115166667,545400,8,0,0,0,0,0,-97,1,0,2,0,3700"


def test_tx_line():
    log = parse_log("100 TX,66,2,5,98,ok\n200 TX,66,2,6,198,timeout\n")
    assert [(t.key, t.completed, t.millis, t.t_ms) for t in log.tx] == [
        ((66, 2, 5), True, 98, 100),
        ((66, 2, 6), False, 198, 200),
    ]
    assert log.malformed == 0


def test_rec_line_key_and_rssi():
    log = parse_log(f"300 {REC}\n")
    assert len(log.rec) == 1
    assert log.rec[0].key == (66, 2, 5)
    assert log.rec[0].rssi == -97


def test_err_lines():
    log = parse_log("1 ERR,decode,1\n2 ERR,decode,2\n3 ERR,short,12,1\n")
    assert (log.decode_errors, log.short_frames, log.malformed) == (2, 1, 0)


def test_unknown_err_kind_is_malformed():
    assert parse_log("1 ERR,bogus,1\n").malformed == 1


def test_banners_are_ignored():
    log = parse_log("0 floodnet node: duty-cycled sampler\n1 radio: init failed\n")
    assert (len(log.tx), len(log.rec), log.malformed) == (0, 0, 0)


def test_bad_lines_are_malformed():
    text = "\n".join(
        [
            "x TX,66,2,5,98,ok",  # timestamp not a number
            "TX,66,2,5,98,ok",  # no timestamp
            "1 TX,66,2,5,98,maybe",  # unknown outcome
            "2 TX,66,2,5",  # too few fields
            "3 REC,66,5",  # too few fields
            "4 TX,66,two,5,98,ok",  # field not a number
        ]
    )
    assert parse_log(text + "\n").malformed == 6


def test_unterminated_final_line_is_malformed():
    log = parse_log("100 TX,66,2,5,98,ok\n200 TX,66,2,6,19")
    assert len(log.tx) == 1
    assert log.malformed == 1


def test_crlf_line_endings():
    log = parse_log("100 TX,66,2,5,98,ok\r\n")
    assert len(log.tx) == 1 and log.malformed == 0


def test_empty_log():
    log = parse_log("")
    assert (len(log.tx), len(log.rec), log.malformed) == (0, 0, 0)
```

Run: `tools/hil/.venv/bin/pytest tools/hil/tests/test_parse.py -q`
Expected: FAIL, `ModuleNotFoundError: No module named 'floodnet_hil.parse'`.

- [ ] **Step 3: Implement `parse.py`**

Create `tools/hil/src/floodnet_hil/parse.py`:

```python
"""Parse FloodNet node and gateway serial logs.

Each log line is "<t_ms> <line>": milliseconds since the capture started (the
simulation clock for simulated transcripts), then the line as the board
printed it.
"""

from __future__ import annotations

from dataclasses import dataclass, field

TX_FIELDS = 6
REC_FIELDS = 19


@dataclass(frozen=True)
class TxLine:
    t_ms: int
    node_id: int
    boot_count: int
    seq: int
    millis: int
    completed: bool

    @property
    def key(self) -> tuple[int, int, int]:
        return (self.node_id, self.boot_count, self.seq)


@dataclass(frozen=True)
class RecLine:
    t_ms: int
    node_id: int
    seq: int
    boot_count: int
    rssi: int

    @property
    def key(self) -> tuple[int, int, int]:
        return (self.node_id, self.boot_count, self.seq)


@dataclass
class ParsedLog:
    tx: list[TxLine] = field(default_factory=list)
    rec: list[RecLine] = field(default_factory=list)
    decode_errors: int = 0
    short_frames: int = 0
    malformed: int = 0


def _parse_line(line: str, out: ParsedLog) -> None:
    stamp, sep, payload = line.partition(" ")
    try:
        if not sep:
            raise ValueError("no timestamp")
        t_ms = int(stamp)
        fields = payload.split(",")
        kind = fields[0]
        if kind == "TX":
            if len(fields) != TX_FIELDS or fields[5] not in ("ok", "timeout"):
                raise ValueError("bad TX line")
            node_id, boot_count, seq, millis = (int(f) for f in fields[1:5])
            out.tx.append(TxLine(t_ms, node_id, boot_count, seq, millis, fields[5] == "ok"))
        elif kind == "REC":
            if len(fields) != REC_FIELDS:
                raise ValueError("bad REC line")
            values = [int(f) for f in fields[1:]]
            out.rec.append(
                RecLine(t_ms, node_id=values[0], seq=values[1], boot_count=values[15],
                        rssi=values[12])
            )
        elif kind == "ERR":
            if len(fields) >= 2 and fields[1] == "decode":
                out.decode_errors += 1
            elif len(fields) >= 2 and fields[1] == "short":
                out.short_frames += 1
            else:
                raise ValueError("unknown ERR line")
        # Anything else (a boot banner, "radio: init failed") is not data.
    except ValueError:
        out.malformed += 1


def parse_log(text: str) -> ParsedLog:
    """Parses a whole log. A final line with no newline was cut off mid-write,
    so it is counted as malformed rather than parsed."""
    out = ParsedLog()
    if not text:
        return out
    lines = text.split("\n")
    for line in lines[:-1]:
        line = line.rstrip("\r")
        if line:
            _parse_line(line, out)
    if lines[-1]:
        out.malformed += 1
    return out
```

Run: `tools/hil/.venv/bin/pytest tools/hil/tests/test_parse.py -q`
Expected: 9 passed.

- [ ] **Step 4: Write the failing analyze tests**

Create `tools/hil/tests/test_analyze.py`:

```python
from pathlib import Path

from floodnet_hil.analyze import analyze, analyze_dir
from floodnet_hil.parse import parse_log


def rec(t, node, seq, boot, rssi=-90):
    return f"{t} REC,{node},{seq},0,0,0,0,0,0,0,0,0,0,{rssi},1,0,{boot},0,0"


def tx(t, node, boot, seq, outcome="ok"):
    return f"{t} TX,{node},{boot},{seq},{t},{outcome}"


def run(node_lines, gateway_lines, label="run"):
    node = parse_log("".join(line + "\n" for line in node_lines))
    gateway = parse_log("".join(line + "\n" for line in gateway_lines))
    return analyze(node, gateway, label)


def test_matching_and_delivery():
    s = run(
        [tx(1, 66, 1, 0), tx(2, 66, 1, 1), tx(3, 66, 1, 2)],
        [rec(1, 66, 0, 1), rec(3, 66, 2, 1)],
    )
    assert (s.tx_ok, s.accepted, s.unmatched_rec) == (3, 2, 0)
    assert s.delivery == 2 / 3


def test_matching_across_a_reboot():
    s = run(
        [tx(1, 66, 1, 0), tx(2, 66, 2, 0)],
        [rec(1, 66, 0, 1), rec(2, 66, 0, 2)],
    )
    assert (s.accepted, s.unmatched_rec, s.seq_gaps) == (2, 0, 0)


def test_duplicate_rec_counts_once():
    s = run([tx(1, 66, 1, 0)], [rec(1, 66, 0, 1), rec(2, 66, 0, 1)])
    assert (s.accepted, s.unmatched_rec) == (1, 0)


def test_timeout_never_matches_and_is_counted_apart():
    s = run([tx(1, 66, 1, 0, "timeout"), tx(2, 66, 1, 1)], [rec(1, 66, 0, 1), rec(2, 66, 1, 1)])
    assert (s.tx_ok, s.tx_timeout, s.accepted, s.unmatched_rec) == (1, 1, 1, 1)


def test_unmatched_rec():
    s = run([tx(1, 66, 1, 0)], [rec(1, 66, 0, 1), rec(2, 66, 9, 1)])
    assert (s.accepted, s.unmatched_rec) == (1, 1)


def test_seq_gaps_per_boot():
    s = run(
        [],
        [rec(1, 66, 0, 1), rec(2, 66, 3, 1), rec(3, 66, 10, 2), rec(4, 66, 11, 2)],
    )
    assert s.seq_gaps == 2


def test_rssi_with_no_accepted_frames():
    s = run([tx(1, 66, 1, 0)], [])
    assert (s.rssi_min, s.rssi_median, s.rssi_max) == (None, None, None)


def test_rssi_with_one_accepted_frame():
    s = run([tx(1, 66, 1, 0)], [rec(1, 66, 0, 1, -80)])
    assert (s.rssi_min, s.rssi_median, s.rssi_max) == (-80, -80, -80)


def test_rssi_median_of_two_is_the_lower():
    s = run([tx(1, 66, 1, 0), tx(2, 66, 1, 1)], [rec(1, 66, 0, 1, -80), rec(2, 66, 1, 1, -60)])
    assert (s.rssi_min, s.rssi_median, s.rssi_max) == (-80, -80, -60)


def test_zero_tx_ok_prints_a_dash():
    s = run([], [], label="empty")
    assert s.delivery is None
    assert s.line() == "HIL,empty,0,0,0,-,0,0,0,0,-,-,-,0"


def test_summary_line():
    s = run(
        [tx(1, 66, 1, 0), tx(2, 66, 1, 1)],
        [rec(1, 66, 0, 1, -80), "5 ERR,decode,1", "6 ERR,short,12,1", "garbage"],
        label="bench",
    )
    assert s.line() == "HIL,bench,2,0,1,0.500000,1,1,0,0,-80,-80,-80,1"


def test_analyze_dir_reads_both_logs(tmp_path: Path):
    run_dir = tmp_path / "attenuator_20db"
    run_dir.mkdir()
    (run_dir / "node.log").write_text(tx(1, 66, 1, 0) + "\n")
    (run_dir / "gateway.log").write_text(rec(1, 66, 0, 1) + "\n")
    s = analyze_dir(run_dir)
    assert s.label == "attenuator_20db"
    assert (s.tx_ok, s.accepted) == (1, 1)
```

Run: `tools/hil/.venv/bin/pytest tools/hil/tests/test_analyze.py -q`
Expected: FAIL, `ModuleNotFoundError: No module named 'floodnet_hil.analyze'`.

- [ ] **Step 5: Implement `analyze.py`**

Create `tools/hil/src/floodnet_hil/analyze.py`:

```python
"""Turn a node log and a gateway log into one HIL summary line."""

from __future__ import annotations

from collections import defaultdict
from dataclasses import dataclass
from pathlib import Path

from floodnet_hil.parse import ParsedLog, RecLine, parse_log


@dataclass(frozen=True)
class Summary:
    label: str
    tx_ok: int
    tx_timeout: int
    accepted: int
    delivery: float | None
    decode_errors: int
    short_frames: int
    unmatched_rec: int
    seq_gaps: int
    rssi_min: int | None
    rssi_median: int | None
    rssi_max: int | None
    malformed: int

    def line(self) -> str:
        def opt(value: int | None) -> str:
            return "-" if value is None else str(value)

        delivery = "-" if self.delivery is None else f"{self.delivery:.6f}"
        return (
            f"HIL,{self.label},{self.tx_ok},{self.tx_timeout},{self.accepted},{delivery},"
            f"{self.decode_errors},{self.short_frames},{self.unmatched_rec},{self.seq_gaps},"
            f"{opt(self.rssi_min)},{opt(self.rssi_median)},{opt(self.rssi_max)},{self.malformed}"
        )


def _seq_gaps(keys: set[tuple[int, int, int]]) -> int:
    """Sequence numbers missing between the lowest and highest accepted,
    per (node_id, boot_count)."""
    seqs: dict[tuple[int, int], set[int]] = defaultdict(set)
    for node_id, boot_count, seq in keys:
        seqs[(node_id, boot_count)].add(seq)
    return sum(max(s) - min(s) + 1 - len(s) for s in seqs.values())


def analyze(node: ParsedLog, gateway: ParsedLog, label: str) -> Summary:
    ok_keys = {t.key for t in node.tx if t.completed}
    tx_ok = sum(1 for t in node.tx if t.completed)
    tx_timeout = sum(1 for t in node.tx if not t.completed)

    first_rec: dict[tuple[int, int, int], RecLine] = {}
    for r in gateway.rec:
        first_rec.setdefault(r.key, r)
    matched = [r for key, r in first_rec.items() if key in ok_keys]
    rssi = sorted(r.rssi for r in matched)

    return Summary(
        label=label,
        tx_ok=tx_ok,
        tx_timeout=tx_timeout,
        accepted=len(matched),
        delivery=len(matched) / tx_ok if tx_ok else None,
        decode_errors=gateway.decode_errors,
        short_frames=gateway.short_frames,
        unmatched_rec=len(first_rec) - len(matched),
        seq_gaps=_seq_gaps(set(first_rec)),
        rssi_min=rssi[0] if rssi else None,
        rssi_median=rssi[(len(rssi) - 1) // 2] if rssi else None,
        rssi_max=rssi[-1] if rssi else None,
        malformed=node.malformed + gateway.malformed,
    )


def analyze_dir(run_dir: Path) -> Summary:
    """Analyzes run_dir/node.log and run_dir/gateway.log; the label is the
    directory's name."""
    node = parse_log((run_dir / "node.log").read_text(encoding="utf-8", errors="replace"))
    gateway = parse_log((run_dir / "gateway.log").read_text(encoding="utf-8", errors="replace"))
    return analyze(node, gateway, run_dir.name)
```

Run: `tools/hil/.venv/bin/pytest tools/hil/tests/test_analyze.py -q`
Expected: 12 passed.

- [ ] **Step 6: Write the failing capture and CLI tests**

Create `tools/hil/tests/test_capture.py`:

```python
from pathlib import Path

from floodnet_hil.capture import capture


class FakePort:
    def __init__(self, chunks):
        self.chunks = list(chunks)

    @property
    def in_waiting(self):
        return len(self.chunks[0]) if self.chunks else 0

    def read(self, size):
        return self.chunks.pop(0) if self.chunks else b""


class FakeClock:
    def __init__(self, step):
        self.now = 0.0
        self.step = step

    def __call__(self):
        self.now += self.step
        return self.now


def test_capture_writes_complete_lines_with_times(tmp_path: Path):
    node = FakePort([b"TX,66,1,0,5,ok\r\nTX,66,1", b",1,9,ok\r\n", b"TX,66,1,2,1"])
    gateway = FakePort([b"floodnet gateway\r\n"])
    counts = capture({"node": node, "gateway": gateway}, tmp_path / "run", 1.0,
                     clock=FakeClock(0.01))
    assert counts == {"node": 2, "gateway": 1}
    node_lines = (tmp_path / "run" / "node.log").read_text().splitlines()
    assert [line.split(" ", 1)[1] for line in node_lines] == ["TX,66,1,0,5,ok", "TX,66,1,1,9,ok"]
    assert all(line.split(" ", 1)[0].isdigit() for line in node_lines)
    assert (tmp_path / "run" / "gateway.log").read_text().endswith("floodnet gateway\n")


def test_capture_stops_at_the_deadline(tmp_path: Path):
    endless = FakePort([b"x\n"] * 10000)
    clock = FakeClock(0.1)
    capture({"node": endless, "gateway": FakePort([])}, tmp_path / "run", 1.0, clock=clock)
    assert clock.now < 2.0
```

Create `tools/hil/tests/test_cli.py`:

```python
from pathlib import Path

import serial

from floodnet_hil import cli


def test_analyze_prints_the_hil_line(tmp_path: Path, capsys):
    run_dir = tmp_path / "bench"
    run_dir.mkdir()
    (run_dir / "node.log").write_text("1 TX,66,1,0,1,ok\n")
    (run_dir / "gateway.log").write_text(
        "2 REC,66,0,0,0,0,0,0,0,0,0,0,0,-90,1,0,1,0,0\n"
    )
    assert cli.main(["analyze", str(run_dir)]) == 0
    assert capsys.readouterr().out == "HIL,bench,1,0,1,1.000000,0,0,0,0,-90,-90,-90,0\n"


def test_capture_reports_a_port_it_cannot_open(tmp_path: Path, monkeypatch, capsys):
    def refuse(port, *args, **kwargs):
        raise serial.SerialException(f"could not open port {port}")

    monkeypatch.setattr(serial, "Serial", refuse)
    code = cli.main(
        ["capture", "--node", "/dev/nope", "--gateway", "/dev/nada", "--label", "x",
         "--seconds", "1", "--out", str(tmp_path)]
    )
    assert code == 2
    assert "/dev/nope" in capsys.readouterr().err
```

Run: `tools/hil/.venv/bin/pytest tools/hil/tests/test_capture.py tools/hil/tests/test_cli.py -q`
Expected: FAIL, `ModuleNotFoundError` for `floodnet_hil.capture` / `cli`.

- [ ] **Step 7: Implement `capture.py` and `cli.py`**

Create `tools/hil/src/floodnet_hil/capture.py`:

```python
"""Record both boards' serial output to log files, unchanged.

Only complete lines are written, each prefixed with milliseconds since the
capture started, so every log ends with a newline. A line still arriving when
the capture ends is dropped rather than written half-finished.
"""

from __future__ import annotations

import time
from collections.abc import Callable
from pathlib import Path


class _LineSplitter:
    def __init__(self) -> None:
        self._pending = b""

    def feed(self, data: bytes) -> list[str]:
        self._pending += data
        *complete, self._pending = self._pending.split(b"\n")
        return [line.rstrip(b"\r").decode("utf-8", errors="replace") for line in complete]


def capture(
    ports: dict[str, object],
    out_dir: Path,
    seconds: float,
    clock: Callable[[], float] = time.monotonic,
) -> dict[str, int]:
    """Reads every port until `seconds` have passed and writes out_dir/<name>.log
    per port. Returns the number of lines written per port."""
    out_dir.mkdir(parents=True, exist_ok=True)
    files = {
        name: open(out_dir / f"{name}.log", "w", encoding="utf-8", newline="\n")
        for name in ports
    }
    splitters = {name: _LineSplitter() for name in ports}
    counts = {name: 0 for name in ports}
    start = clock()
    try:
        while clock() - start < seconds:
            for name, port in ports.items():
                data = port.read(port.in_waiting or 1)
                if not data:
                    continue
                t_ms = int((clock() - start) * 1000)
                for line in splitters[name].feed(data):
                    files[name].write(f"{t_ms} {line}\n")
                    counts[name] += 1
    finally:
        for f in files.values():
            f.close()
    return counts
```

Create `tools/hil/src/floodnet_hil/cli.py`:

```python
"""floodnet-hil: capture both boards' serial logs, then analyze them."""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

from floodnet_hil.analyze import analyze_dir
from floodnet_hil.capture import capture


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(prog="floodnet-hil")
    sub = parser.add_subparsers(dest="command", required=True)

    cap = sub.add_parser("capture", help="record node and gateway serial output")
    cap.add_argument("--node", required=True, help="the node's serial port")
    cap.add_argument("--gateway", required=True, help="the gateway's serial port")
    cap.add_argument("--label", required=True, help="scenario name, e.g. attenuator_20db")
    cap.add_argument("--seconds", required=True, type=float)
    cap.add_argument("--baud", type=int, default=115200)
    cap.add_argument("--out", type=Path, default=Path("hil-captures"))

    ana = sub.add_parser("analyze", help="print the HIL summary line for a capture")
    ana.add_argument("run_dir", type=Path)
    return parser


def main(argv: list[str] | None = None) -> int:
    args = _parser().parse_args(argv)
    if args.command == "analyze":
        print(analyze_dir(args.run_dir).line())
        return 0

    import serial

    ports = {}
    for name, device in (("node", args.node), ("gateway", args.gateway)):
        try:
            ports[name] = serial.Serial(device, args.baud, timeout=0.05)
        except serial.SerialException as err:
            print(f"floodnet-hil: cannot open {device}: {err}", file=sys.stderr)
            return 2
    run_dir = args.out / args.label
    counts = capture(ports, run_dir, args.seconds)
    print(
        f"captured {counts['node']} node lines and {counts['gateway']} gateway lines "
        f"into {run_dir}"
    )
    return 0


if __name__ == "__main__":
    sys.exit(main())
```

Run: `tools/hil/.venv/bin/pytest tools/hil -q && tools/hil/.venv/bin/ruff check tools/hil`
Expected: 25 passed; ruff prints `All checks passed!`.

- [ ] **Step 8: Show the assertions can fail**

Temporarily change `if lines[-1]:` to `if False:` in `parse_log`; `test_unterminated_final_line_is_malformed` must FAIL.
Temporarily change `first_rec.setdefault(r.key, r)` to a list append that counts duplicates (`matched` built from `gateway.rec` directly); `test_duplicate_rec_counts_once` must FAIL.
Revert both and rerun to PASS.

- [ ] **Step 9: Commit**

```bash
git add tools/hil/pyproject.toml tools/hil/src tools/hil/tests
git commit -m "feat: add the floodnet-hil tool to capture and analyze node and gateway logs"
```

---

### Task 6: Cross-check against the simulation, and CI

**Files:**
- Create: `tools/hil/tests/test_crosscheck.py`
- Modify: `.github/workflows/ci.yml`

**Interfaces:**
- Consumes: `analyze_dir` (Task 5); `build/hil-transcripts/<scenario>/` (Task 4).

- [ ] **Step 1: Write the cross-check**

Create `tools/hil/tests/test_crosscheck.py`:

```python
"""The host tool against the simulation's own counts, for every frozen scenario.

The transcripts are written by `pio test -e native -f test_hil_transcripts`.
A difference means the tool or the firmware hook is wrong; no count is ever
adjusted to make them agree.
"""

import json
from pathlib import Path

import pytest

from floodnet_hil.analyze import analyze_dir
from floodnet_hil.parse import parse_log

REPO = Path(__file__).resolve().parents[3]
TRANSCRIPTS = REPO / "build" / "hil-transcripts"
SCENARIOS = [
    "interrupt_p0",
    "interrupt_p1e-3",
    "interrupt_p1e-2",
    "duty_cycled_p0",
    "duty_cycled_p1e-3",
    "duty_cycled_p1e-2",
    "duty_cycled_lost_completion",
    "duty_cycled_hang",
]


def load(name):
    run_dir = TRANSCRIPTS / name
    expected_path = run_dir / "expected.json"
    if not expected_path.exists():
        pytest.fail(
            f"missing {expected_path}: run `pio test -e native -f test_hil_transcripts` first"
        )
    return run_dir, json.loads(expected_path.read_text())


@pytest.mark.parametrize("name", SCENARIOS)
def test_analyze_matches_the_simulation(name):
    run_dir, expected = load(name)
    s = analyze_dir(run_dir)
    assert s.tx_ok == expected["tx_ok"]
    assert s.tx_timeout == expected["tx_timeout"]
    assert s.unmatched_rec == expected["unmatched_rec"]
    assert s.decode_errors == expected["decode_errors"]
    assert s.accepted == expected["rec_lines"] - expected["unmatched_rec"]
    assert s.malformed == 0


def test_lost_completion_scenario_produces_a_timeout():
    _, expected = load("duty_cycled_lost_completion")
    assert expected["tx_timeout"] >= 1


def test_hang_scenario_spans_two_boots():
    run_dir, _ = load("duty_cycled_hang")
    node = parse_log((run_dir / "node.log").read_text())
    assert len({t.boot_count for t in node.tx}) == 2


def test_noisy_scenarios_produce_decode_errors():
    _, expected = load("duty_cycled_p1e-2")
    assert expected["decode_errors"] > 0
```

- [ ] **Step 2: Run it**

Run: `pio test -e native -f test_hil_transcripts && tools/hil/.venv/bin/pytest tools/hil/tests/test_crosscheck.py -q`
Expected: 11 passed.
If a scenario disagrees, that is a defect in the tool, the hook or the rig: debug it with superpowers:systematic-debugging; never edit an expected count.

- [ ] **Step 3: Show the assertions can fail**

Temporarily change `if t.completed` in the `tx_ok` sum in `analyze.py` to count every TX line; the lost-completion case of `test_analyze_matches_the_simulation` must FAIL.
Temporarily delete `build/hil-transcripts/duty_cycled_p0/expected.json`; its case must FAIL with the "missing ... run `pio test`" message (not skip).
Revert the code, rerun `pio test -e native -f test_hil_transcripts` to restore the file, and rerun the cross-check to PASS.

- [ ] **Step 4: Add CI steps**

In `.github/workflows/ci.yml`, in the `test` job, after the `Run unit tests` step (`run: pio test -e native`), add:

```yaml
      - name: Install host tool
        run: pip install -e 'tools/hil[test]'
      - name: Lint host tool
        run: ruff check tools/hil
      - name: Test host tool against the simulated transcripts
        run: pytest tools/hil
```

- [ ] **Step 5: Run the whole thing the way CI will**

Run: `pio test -e native && tools/hil/.venv/bin/ruff check tools/hil && tools/hil/.venv/bin/pytest tools/hil -q`
Expected: every native suite PASSES; ruff clean; 36 passed.

- [ ] **Step 6: Commit**

```bash
git add tools/hil/tests/test_crosscheck.py .github/workflows/ci.yml
git commit -m "test: cross-check the host tool against the simulated transcripts, in CI"
```

---

### Task 7: Publish

**Files:**
- Create: `docs/results/milestone-5-crosscheck.txt`
- Modify: `README.md`, `docs/superpowers/specs/2026-09-30-milestone-5-board-target-design.md`

**Interfaces:**
- Consumes: the transcripts (Task 4) and `floodnet-hil analyze` (Task 5).

- [ ] **Step 1: Record the cross-check output**

Run:

```bash
for s in interrupt_p0 interrupt_p1e-3 interrupt_p1e-2 duty_cycled_p0 duty_cycled_p1e-3 duty_cycled_p1e-2 duty_cycled_lost_completion duty_cycled_hang; do
  tools/hil/.venv/bin/floodnet-hil analyze build/hil-transcripts/$s
done | tee docs/results/milestone-5-crosscheck.txt
```

Expected: 8 `HIL,...` lines, `malformed` 0 in each, and `rssi_*` 0 (the simulation prints RSSI 0).

- [ ] **Step 2: Update the spec**

Change `Status: approved` to `Status: implemented`.
Add to "Revisions" a dated entry for every change made during implementation, with its reason.

- [ ] **Step 3: Update the README**

Edit, one sentence per line:

1. **Status:** milestone 5 adds the board target for the link regression tool; built and cross-checked against simulated transcripts; no board has run it.
2. **Results:** a new "Milestone 5: board target" subsection with:
   - what the node and gateway print (the `TX`, `REC` and `ERR,decode` formats) and the `<t_ms> <line>` log format;
   - how to use it on two boards: flash `node_duty_cycled` (or `node_interrupt`) and `gateway`, then `floodnet-hil capture --node PORT --gateway PORT --label NAME --seconds N` and `floodnet-hil analyze hil-captures/NAME`;
   - the `HIL` line format and what each field means, including `unmatched_rec` and `seq_gaps`;
   - the cross-check: the eight scenarios, the raw `HIL` lines from `docs/results/milestone-5-crosscheck.txt` as printed, and the statement that each equals the simulation's own counts exactly;
   - plainly: these are simulated transcripts, not measurements, and the first real figures need two boards and one `capture` run.
3. **Known limitations:** no board has run any of this; RSSI is 0 in simulation; `capture` is tested only against fake ports; host timestamps are arrival times; a `timeout` frame is never counted as sent even if the gateway received it (it then shows in `unmatched_rec`); a frame in flight at a watchdog reset has no `TX` line on hardware either; `node_polling` prints no `TX` line.
4. **Deferred to later milestones:** replace the "board target" bullet with the remaining step, running `capture` on two boards and publishing the results; update "Sequence-gap counting" to say receiver-side counting now exists in `floodnet-hil analyze` (`seq_gaps`).

- [ ] **Step 4: Check the numbers and writing rules**

Run: `grep -E '^HIL,' docs/results/milestone-5-crosscheck.txt | while read -r line; do grep -qF "$line" README.md || echo "MISSING: $line"; done`
Expected: no output.
Run: `grep -n $'\xe2\x80\x94' README.md docs/superpowers/specs/2026-09-30-milestone-5-board-target-design.md docs/superpowers/plans/2026-09-30-milestone-5-board-target.md`
Expected: no output.

- [ ] **Step 5: Final verification**

Run: `pio test -e native && pio run -e node_polling -e node_interrupt -e node_duty_cycled -e gateway && tools/hil/.venv/bin/ruff check tools/hil && tools/hil/.venv/bin/pytest tools/hil -q`, then the three guards.
Expected: everything PASSES, all four images build, ruff clean, the guards print nothing.

- [ ] **Step 6: Commit**

```bash
git add docs/results/milestone-5-crosscheck.txt README.md docs/superpowers/specs/2026-09-30-milestone-5-board-target-design.md
git commit -m "docs: publish the milestone 5 board target and its cross-check"
```
