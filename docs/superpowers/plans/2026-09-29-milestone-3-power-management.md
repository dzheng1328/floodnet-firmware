# Milestone 3: Power Management and Fault Recovery Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add a duty-cycled node build (`node_duty_cycled`) driven by a pure node state machine, with sleep, a hardware watchdog, fault recovery, a 45-byte v0x02 wire format, and a host simulation that measures sensor downtime against the milestone 2 build.

**Architecture:** `NodeStateMachine` and `compute_downtime()` are pure functions in `lib/floodnet_core`. Three new HAL interfaces (`IPower`, `IWatchdog`, `IPersistentStore`) sit in `lib/floodnet_hal`. `DutyCycledNode` in `src/` composes the state machine with an additively extended `InterruptSampler`. The simulation gains powered fakes, energy accounting, and a `PowerRig` harness that runs both builds under scripted faults.

**Tech Stack:** C++17 (gnu++17), PlatformIO, Unity test framework on the `native` environment, Teensy 4.1 Arduino framework with the bundled RadioHead and Snooze libraries, Adafruit BNO055.

**Spec:** `docs/superpowers/specs/2026-09-29-milestone-3-power-management-design.md`. Read it before starting; this plan argues from it.

## Global Constraints

- Build flags stay `-std=gnu++17 -Wall -Wextra`; new code compiles without warnings.
- Nothing under `lib/` may include `Arduino.h`, `Wire.h`, or `SPI.h` (CI greps for it).
- No heap allocation in firmware code (`lib/`, `src/`). Tests and `test/support/` may use `std::vector` and `std::unique_ptr`.
- `PACKET_SIZE` stays 45 for both wire versions.
- Report interval 300 000 ms; down when the newest valid fix is more than 600 000 ms old.
- `ACQUIRE_TIMEOUT_MS` 60 000, `TX_TIMEOUT_MS` 5 000 (existing), `TRANSMIT_TIMEOUT_MS` 10 000, `WATCHDOG_TIMEOUT_MS` 90 000, `SLEEP_CHUNK_MS` 60 000, `RADIO_FAIL_LIMIT` 3, IMU fail limit 3.
- Currents, battery capacity and TTFF values are exactly the spec's "Energy accounting" table; each constant carries its citation in a comment.
- The milestone 1 and 2 benchmark rows in `README.md` must reproduce bit for bit after every task that touches `test/support/` or `src/sampler_interrupt.*`.
- Never use the em dash character anywhere; use "-".
- Markdown edits: one sentence per physical line.
- Commit messages use the repo's `feat:`/`fix:`/`test:`/`docs:`/`refactor:` prefixes, and carry no co-author trailer.
- Nothing measured may be tuned after it is seen. If a result looks wrong, stop and report it; do not change constants, schedules, or thresholds to move it.

## Review Focus

1. **`millis()` wrapping during a sleep or across a slot boundary** - the node must keep its 5-minute schedule through the 49.7-day rollover. Pinned by `test_schedule_survives_millis_wrap` in Task 4.
2. **A complete GPS sentence left in the receive buffer when the node sleeps** - on the next wake it must not be parsed and reported as a fresh fix. Pinned by `test_stale_sentence_from_before_sleep_is_not_reported` in Task 9.
3. **A node rebooting and restarting its sequence at 0** - the gateway must not discard its first packets as duplicates. Pinned by `test_same_sequence_after_a_reboot_relays` in Task 2 and `test_hang_is_reset_by_the_watchdog_and_reports_resume` in Task 9.
4. **The battery running out in the middle of a 60 s sleep jump** - the death time must be exact, not rounded to the end of the jump, or battery downtime is misattributed. Pinned by `test_depletion_time_is_exact_inside_a_long_tick` in Task 7.
5. **An IMU sample too far from its GPS fix to fit the v0x02 int16 skew field** - it must go out as "no IMU", never as a wrapped skew. Pinned by `test_v2_marks_imu_invalid_when_skew_does_not_fit` in Task 1.

---

## File Structure

| Path | Responsibility | Task |
|---|---|---|
| `lib/floodnet_core/include/floodnet/packet.hpp`, `src/packet.cpp` | v0x01 and v0x02 codecs | 1 |
| `lib/floodnet_core/include/floodnet/mesh.hpp`, `src/mesh.cpp` | dedup keyed on boot count | 2 |
| `src/gateway_main.cpp` | new dedup key, new REC fields | 2 |
| `lib/floodnet_core/include/floodnet/ubx.hpp`, `src/ubx.cpp` | UBX-RXM-PMREQ frame builder | 3 |
| `lib/floodnet_core/include/floodnet/node_state.hpp`, `src/node_state.cpp` | pure node state machine | 4 |
| `lib/floodnet_core/include/floodnet/downtime.hpp`, `src/downtime.cpp` | pure downtime calculation | 5 |
| `test/support/sim_clock.hpp`, `fake_gps.hpp`, `fake_imu.hpp`, `fake_radio.hpp`, `hang_breaker.hpp` | powered fakes, TTFF, fault hooks | 6 |
| `lib/floodnet_hal/include/floodnet/hal/power.hpp`, `watchdog.hpp`, `persistent_store.hpp` | new HAL interfaces | 7 |
| `test/support/current_model.hpp`, `fake_power.hpp`, `fake_watchdog.hpp`, `fake_persistent_store.hpp` | energy accounting and reset fakes | 7 |
| `src/sampler_interrupt.hpp`, `.cpp` | four additive members | 8 |
| `src/duty_cycled_node.hpp`, `.cpp`, `test/support/power_rig.hpp`, `platformio.ini` (native) | the node, and the harness both builds run in | 9 |
| `src/hal/teensy_*.hpp`, `src/main.cpp`, `platformio.ini`, `.github/workflows/ci.yml`, `docs/hardware.md` | Teensy implementations and the new build target | 10 |
| `test/test_power_bench/test_main.cpp` | the three experiments and the null control | 11 |
| `README.md`, the spec's status line | results and limitations | 12 |

Run every test command from the repository root, `/Users/dzheng/Documents/floodnet-firmware`.

---

### Task 1: Wire format v0x02

**Files:**
- Modify: `lib/floodnet_core/include/floodnet/packet.hpp`
- Modify: `lib/floodnet_core/src/packet.cpp`
- Test: `test/test_packet/test_main.cpp`

**Interfaces:**
- Consumes: nothing new.
- Produces:
  - `enum class WireVersion : uint8_t { V1 = 0x01, V2 = 0x02 };`
  - `const uint8_t PACKET_VERSION_V2 = 0x02;`, `FLAG_GPS_VALID = 0x01`, `FLAG_IMU_VALID = 0x02`, `FLAG_VALIDITY_MASK = 0x03`
  - `Packet` gains `uint16_t boot_count = 0; uint16_t tx_timeouts = 0; uint16_t battery_mv = 0;`
  - `size_t encode_packet_v2(const Packet &p, uint8_t *out, size_t out_len);`
  - `size_t encode_packet_as(WireVersion version, const Packet &p, uint8_t *out, size_t out_len);`
  - `decode_packet()` keeps its signature and accepts both versions.

- [ ] **Step 1: Write the failing tests**

Add to `test/test_packet/test_main.cpp`, after `make_packet()`:

```cpp
static Packet make_packet_v2(void) {
    Packet p = make_packet();
    p.boot_count = 0x0305;
    p.tx_timeouts = 9;
    p.battery_mv = 3712;
    return p;
}

void test_v2_golden_vector_pins_byte_layout(void) {
    uint8_t buf[PACKET_SIZE];
    TEST_ASSERT_EQUAL_UINT(PACKET_SIZE, encode_packet_v2(make_packet_v2(), buf, sizeof(buf)));

    // Computed independently of encode_packet_v2 (Python reference encoder
    // over the layout in the milestone 3 design doc). Flags 0x5A keeps its
    // upper six bits and gains both validity bits: 0x5B. Skew is 995 - 1000.
    static const uint8_t expected[PACKET_SIZE] = {
        0xFD, 0x02, 0x34, 0x12, 0xEF, 0xBE, 0xAD, 0xDE, 0x03, 0x5B,
        0xE8, 0x03, 0x00, 0x00, 0x08, 0x1E, 0xAE, 0x1C, 0x35, 0xB2,
        0x22, 0xF9, 0x78, 0x52, 0x08, 0x00, 0x08, 0xFB, 0xFF, 0x6C,
        0xEE, 0xFA, 0x00, 0xD4, 0xFE, 0x07, 0x00, 0x05, 0x03, 0x09,
        0x00, 0x80, 0x0E, 0x23, 0xD2,
    };
    TEST_ASSERT_EQUAL_UINT8_ARRAY(expected, buf, PACKET_SIZE);
}

void test_v2_round_trip_preserves_every_field(void) {
    Packet original = make_packet_v2();
    original.flags = 0x58;  // low two bits belong to validity in v0x02
    uint8_t buf[PACKET_SIZE];
    encode_packet_v2(original, buf, sizeof(buf));

    Packet decoded;
    TEST_ASSERT_TRUE(decode_packet(buf, sizeof(buf), &decoded));
    TEST_ASSERT_EQUAL_UINT16(original.node_id, decoded.node_id);
    TEST_ASSERT_EQUAL_UINT32(original.seq, decoded.seq);
    TEST_ASSERT_EQUAL_UINT8(original.ttl, decoded.ttl);
    TEST_ASSERT_EQUAL_UINT8(0x58, decoded.flags);
    TEST_ASSERT_EQUAL_UINT32(original.record.gps.time_ms, decoded.record.gps.time_ms);
    TEST_ASSERT_EQUAL_INT32(original.record.gps.lat_1e7, decoded.record.gps.lat_1e7);
    TEST_ASSERT_EQUAL_INT32(original.record.gps.lon_1e7, decoded.record.gps.lon_1e7);
    TEST_ASSERT_EQUAL_INT32(original.record.gps.alt_mm, decoded.record.gps.alt_mm);
    TEST_ASSERT_EQUAL_UINT8(original.record.gps.satellites, decoded.record.gps.satellites);
    TEST_ASSERT_TRUE(decoded.record.gps.valid);
    TEST_ASSERT_EQUAL_UINT32(original.record.imu.time_ms, decoded.record.imu.time_ms);
    TEST_ASSERT_EQUAL_INT16(original.record.imu.yaw_cd, decoded.record.imu.yaw_cd);
    TEST_ASSERT_EQUAL_INT16(original.record.imu.pitch_cd, decoded.record.imu.pitch_cd);
    TEST_ASSERT_EQUAL_INT16(original.record.imu.roll_cd, decoded.record.imu.roll_cd);
    TEST_ASSERT_TRUE(decoded.record.imu.valid);
    TEST_ASSERT_EQUAL_UINT16(original.record.diag.drops, decoded.record.diag.drops);
    TEST_ASSERT_EQUAL_UINT16(0, decoded.record.diag.crc_errors);  // not carried in v0x02
    TEST_ASSERT_EQUAL_UINT16(0x0305, decoded.boot_count);
    TEST_ASSERT_EQUAL_UINT16(9, decoded.tx_timeouts);
    TEST_ASSERT_EQUAL_UINT16(3712, decoded.battery_mv);
}

void test_v2_folds_validity_into_flag_bits(void) {
    Packet p = make_packet_v2();
    p.flags = 0;
    p.record.gps.valid = false;
    uint8_t buf[PACKET_SIZE];
    encode_packet_v2(p, buf, sizeof(buf));
    TEST_ASSERT_EQUAL_HEX8(FLAG_IMU_VALID, buf[9]);

    Packet decoded;
    TEST_ASSERT_TRUE(decode_packet(buf, sizeof(buf), &decoded));
    TEST_ASSERT_FALSE(decoded.record.gps.valid);
    TEST_ASSERT_TRUE(decoded.record.imu.valid);
    TEST_ASSERT_EQUAL_HEX8(0, decoded.flags);
}

void test_v2_marks_imu_invalid_when_skew_does_not_fit(void) {
    Packet p = make_packet_v2();
    p.record.imu.time_ms = p.record.gps.time_ms + 40000;  // beyond int16 milliseconds
    uint8_t buf[PACKET_SIZE];
    TEST_ASSERT_EQUAL_UINT(PACKET_SIZE, encode_packet_v2(p, buf, sizeof(buf)));

    Packet decoded;
    TEST_ASSERT_TRUE(decode_packet(buf, sizeof(buf), &decoded));
    TEST_ASSERT_FALSE(decoded.record.imu.valid);
    TEST_ASSERT_EQUAL_UINT32(0, decoded.record.imu.time_ms);
    TEST_ASSERT_EQUAL_INT16(0, decoded.record.imu.yaw_cd);
    TEST_ASSERT_EQUAL_INT16(0, decoded.record.imu.pitch_cd);
    TEST_ASSERT_EQUAL_INT16(0, decoded.record.imu.roll_cd);
}

void test_v2_negative_skew_survives_a_millis_wrap(void) {
    Packet p = make_packet_v2();
    p.record.gps.time_ms = 3;           // just after the rollover
    p.record.imu.time_ms = 0xFFFFFFFE;  // five ms earlier, just before it
    uint8_t buf[PACKET_SIZE];
    encode_packet_v2(p, buf, sizeof(buf));

    Packet decoded;
    TEST_ASSERT_TRUE(decode_packet(buf, sizeof(buf), &decoded));
    TEST_ASSERT_TRUE(decoded.record.imu.valid);
    TEST_ASSERT_EQUAL_UINT32(0xFFFFFFFE, decoded.record.imu.time_ms);
}

void test_decode_still_accepts_v1_and_zeroes_v2_fields(void) {
    uint8_t buf[PACKET_SIZE];
    encode_packet(make_packet_v2(), buf, sizeof(buf));
    TEST_ASSERT_EQUAL_HEX8(0x01, buf[1]);

    Packet decoded;
    TEST_ASSERT_TRUE(decode_packet(buf, sizeof(buf), &decoded));
    TEST_ASSERT_EQUAL_UINT16(0, decoded.boot_count);
    TEST_ASSERT_EQUAL_UINT16(0, decoded.tx_timeouts);
    TEST_ASSERT_EQUAL_UINT16(0, decoded.battery_mv);
    TEST_ASSERT_EQUAL_UINT16(2, decoded.record.diag.crc_errors);
}

void test_decode_rejects_unknown_version(void) {
    uint8_t buf[PACKET_SIZE];
    encode_packet_v2(make_packet_v2(), buf, sizeof(buf));
    buf[1] = 0x03;
    // Recompute the CRC so version is the only thing wrong.
    const uint16_t crc = crc16_ccitt(buf, 43);
    buf[43] = static_cast<uint8_t>(crc & 0xFF);
    buf[44] = static_cast<uint8_t>(crc >> 8);

    Packet decoded;
    TEST_ASSERT_FALSE(decode_packet(buf, sizeof(buf), &decoded));
}

void test_encode_packet_as_dispatches_on_version(void) {
    uint8_t buf[PACKET_SIZE];
    encode_packet_as(WireVersion::V1, make_packet_v2(), buf, sizeof(buf));
    TEST_ASSERT_EQUAL_HEX8(0x01, buf[1]);
    encode_packet_as(WireVersion::V2, make_packet_v2(), buf, sizeof(buf));
    TEST_ASSERT_EQUAL_HEX8(0x02, buf[1]);
}

void test_v2_encode_rejects_short_buffer(void) {
    uint8_t buf[PACKET_SIZE - 1];
    TEST_ASSERT_EQUAL_UINT(0, encode_packet_v2(make_packet_v2(), buf, sizeof(buf)));
}
```

Register each in `main()` after `RUN_TEST(test_decode_rejects_wrong_length);`:

```cpp
    RUN_TEST(test_v2_golden_vector_pins_byte_layout);
    RUN_TEST(test_v2_round_trip_preserves_every_field);
    RUN_TEST(test_v2_folds_validity_into_flag_bits);
    RUN_TEST(test_v2_marks_imu_invalid_when_skew_does_not_fit);
    RUN_TEST(test_v2_negative_skew_survives_a_millis_wrap);
    RUN_TEST(test_decode_still_accepts_v1_and_zeroes_v2_fields);
    RUN_TEST(test_decode_rejects_unknown_version);
    RUN_TEST(test_encode_packet_as_dispatches_on_version);
    RUN_TEST(test_v2_encode_rejects_short_buffer);
```

- [ ] **Step 2: Run the tests to verify they fail**

Run: `pio test -e native -f test_packet`
Expected: compile failure, `encode_packet_v2` / `boot_count` / `WireVersion` not declared.

- [ ] **Step 3: Implement**

Replace the declarations section of `packet.hpp` (everything between the includes and the closing namespace) with:

```cpp
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
```

In `packet.cpp`, keep the helpers, `crc16_ccitt()` and `encode_packet()` exactly as they are, and replace `decode_packet()` with the following, plus the two new encoders:

```cpp
size_t encode_packet_v2(const Packet &p, uint8_t *out, size_t out_len) {
    if (out == nullptr || out_len < PACKET_SIZE) {
        return 0;
    }

    bool imu_valid = p.record.imu.valid;
    int32_t skew = 0;
    if (imu_valid) {
        // Wrap-safe for the same reason pairing is: unsigned subtraction,
        // then a signed reinterpretation.
        skew = static_cast<int32_t>(p.record.imu.time_ms - p.record.gps.time_ms);
        if (skew < INT16_MIN || skew > INT16_MAX) {
            imu_valid = false;
            skew = 0;
        }
    }

    uint8_t flags = static_cast<uint8_t>(p.flags & ~FLAG_VALIDITY_MASK);
    if (p.record.gps.valid) {
        flags = static_cast<uint8_t>(flags | FLAG_GPS_VALID);
    }
    if (imu_valid) {
        flags = static_cast<uint8_t>(flags | FLAG_IMU_VALID);
    }

    out[0] = PACKET_MAGIC;
    out[1] = PACKET_VERSION_V2;
    put_u16(&out[2], p.node_id);
    put_u32(&out[4], p.seq);
    out[8] = p.ttl;
    out[9] = flags;

    put_u32(&out[10], p.record.gps.time_ms);
    put_u32(&out[14], static_cast<uint32_t>(p.record.gps.lat_1e7));
    put_u32(&out[18], static_cast<uint32_t>(p.record.gps.lon_1e7));
    put_u32(&out[22], static_cast<uint32_t>(p.record.gps.alt_mm));
    out[26] = p.record.gps.satellites;

    put_u16(&out[27], static_cast<uint16_t>(static_cast<int16_t>(skew)));
    put_u16(&out[29], static_cast<uint16_t>(imu_valid ? p.record.imu.yaw_cd : 0));
    put_u16(&out[31], static_cast<uint16_t>(imu_valid ? p.record.imu.pitch_cd : 0));
    put_u16(&out[33], static_cast<uint16_t>(imu_valid ? p.record.imu.roll_cd : 0));

    put_u16(&out[35], p.record.diag.drops);
    put_u16(&out[37], p.boot_count);
    put_u16(&out[39], p.tx_timeouts);
    put_u16(&out[41], p.battery_mv);

    put_u16(&out[43], crc16_ccitt(out, 43));
    return PACKET_SIZE;
}

size_t encode_packet_as(WireVersion version, const Packet &p, uint8_t *out, size_t out_len) {
    return version == WireVersion::V2 ? encode_packet_v2(p, out, out_len)
                                      : encode_packet(p, out, out_len);
}

namespace {

void decode_v1(const uint8_t *in, Packet *p) {
    p->flags = in[9];

    p->record.gps.time_ms = get_u32(&in[10]);
    p->record.gps.lat_1e7 = static_cast<int32_t>(get_u32(&in[14]));
    p->record.gps.lon_1e7 = static_cast<int32_t>(get_u32(&in[18]));
    p->record.gps.alt_mm = static_cast<int32_t>(get_u32(&in[22]));
    p->record.gps.satellites = in[26];
    p->record.gps.valid = in[27] != 0;

    p->record.imu.time_ms = get_u32(&in[28]);
    p->record.imu.yaw_cd = static_cast<int16_t>(get_u16(&in[32]));
    p->record.imu.pitch_cd = static_cast<int16_t>(get_u16(&in[34]));
    p->record.imu.roll_cd = static_cast<int16_t>(get_u16(&in[36]));
    p->record.imu.valid = in[38] != 0;

    p->record.diag.drops = get_u16(&in[39]);
    p->record.diag.crc_errors = get_u16(&in[41]);
}

void decode_v2(const uint8_t *in, Packet *p) {
    p->flags = static_cast<uint8_t>(in[9] & ~FLAG_VALIDITY_MASK);

    p->record.gps.time_ms = get_u32(&in[10]);
    p->record.gps.lat_1e7 = static_cast<int32_t>(get_u32(&in[14]));
    p->record.gps.lon_1e7 = static_cast<int32_t>(get_u32(&in[18]));
    p->record.gps.alt_mm = static_cast<int32_t>(get_u32(&in[22]));
    p->record.gps.satellites = in[26];
    p->record.gps.valid = (in[9] & FLAG_GPS_VALID) != 0;

    const int16_t skew = static_cast<int16_t>(get_u16(&in[27]));
    p->record.imu.valid = (in[9] & FLAG_IMU_VALID) != 0;
    p->record.imu.time_ms =
        p->record.imu.valid
            ? p->record.gps.time_ms + static_cast<uint32_t>(static_cast<int32_t>(skew))
            : 0;
    p->record.imu.yaw_cd = static_cast<int16_t>(get_u16(&in[29]));
    p->record.imu.pitch_cd = static_cast<int16_t>(get_u16(&in[31]));
    p->record.imu.roll_cd = static_cast<int16_t>(get_u16(&in[33]));

    p->record.diag.drops = get_u16(&in[35]);
    p->record.diag.crc_errors = 0;  // not carried: always zero in every milestone
    p->boot_count = get_u16(&in[37]);
    p->tx_timeouts = get_u16(&in[39]);
    p->battery_mv = get_u16(&in[41]);
}

}  // namespace

bool decode_packet(const uint8_t *in, size_t len, Packet *out) {
    if (in == nullptr || out == nullptr || len != PACKET_SIZE) {
        return false;
    }
    if (in[0] != PACKET_MAGIC) {
        return false;
    }
    if (in[1] != PACKET_VERSION && in[1] != PACKET_VERSION_V2) {
        return false;
    }
    if (get_u16(&in[43]) != crc16_ccitt(in, 43)) {
        return false;
    }

    Packet p;
    p.node_id = get_u16(&in[2]);
    p.seq = get_u32(&in[4]);
    p.ttl = in[8];
    if (in[1] == PACKET_VERSION) {
        decode_v1(in, &p);
    } else {
        decode_v2(in, &p);
    }

    *out = p;
    return true;
}
```

The anonymous-namespace helpers `put_u16` etc. are defined at the top of the file, so the second anonymous namespace above sees them. `INT16_MIN`/`INT16_MAX` come from `<stdint.h>`, already included through `packet.hpp`.

Update the `crc_errors` comment in `lib/floodnet_core/include/floodnet/sample.hpp` so it stays true:

```cpp
    uint16_t crc_errors = 0;   ///< Reserved. Always zero: a node only transmits and never
                                ///< receives a radio packet, so it has nothing to fail a CRC
                                ///< check on. Carried by v0x01 only; v0x02 drops it.
```

- [ ] **Step 4: Run the tests to verify they pass**

Run: `pio test -e native -f test_packet`
Expected: all tests PASS, including the unchanged v0x01 golden vector.

- [ ] **Step 5: Run the whole suite**

Run: `pio test -e native`
Expected: every suite PASSES. No existing caller changed.

- [ ] **Step 6: Commit**

```bash
git add lib/floodnet_core/include/floodnet/packet.hpp lib/floodnet_core/src/packet.cpp lib/floodnet_core/include/floodnet/sample.hpp test/test_packet/test_main.cpp
git commit -m "feat: add the 45-byte v0x02 wire format with boot, timeout and battery fields"
```

---

### Task 2: Dedup keyed on boot count, and the gateway

**Files:**
- Modify: `lib/floodnet_core/include/floodnet/mesh.hpp`
- Modify: `lib/floodnet_core/src/mesh.cpp`
- Modify: `src/gateway_main.cpp`
- Test: `test/test_mesh/test_main.cpp`

**Interfaces:**
- Consumes: `Packet::boot_count` (Task 1).
- Produces: `bool DedupTable::seen(uint16_t node_id, uint16_t boot_count, uint32_t seq);` (the two-argument form is removed). `should_relay()` keeps its signature.

- [ ] **Step 1: Write the failing test**

In `test/test_mesh/test_main.cpp`, give the helper a boot count and add the test:

```cpp
static Packet packet(uint16_t node_id, uint32_t seq, uint8_t ttl, uint16_t boot_count = 1) {
    Packet p;
    p.node_id = node_id;
    p.seq = seq;
    p.ttl = ttl;
    p.boot_count = boot_count;
    return p;
}

void test_same_sequence_after_a_reboot_relays(void) {
    // A node restarts its sequence at 0 after every reset. Keyed on
    // (node_id, seq) alone, its first packets after a quick reboot match
    // entries still in the table and are silently discarded.
    DedupTable table;
    TEST_ASSERT_TRUE(should_relay(table, packet(1, 0, 3, 1)));
    TEST_ASSERT_TRUE(should_relay(table, packet(1, 0, 3, 2)));
}
```

Register it after `RUN_TEST(test_same_sequence_from_another_node_relays);`:

```cpp
    RUN_TEST(test_same_sequence_after_a_reboot_relays);
```

- [ ] **Step 2: Run to verify it fails for the right reason**

Run: `pio test -e native -f test_mesh`
Expected: FAIL in `test_same_sequence_after_a_reboot_relays` at the second assertion (the bug reproduced, not a compile error).

- [ ] **Step 3: Implement**

`mesh.hpp`, replace the class comment, `seen()` declaration and `Entry`:

```cpp
/// Remembers recently seen (node_id, boot_count, seq) triples, evicting the
/// oldest first. boot_count is in the key because a node restarts `seq` at 0
/// on every boot; without it, a node's first packets after a quick reboot
/// would match its packets from before and be discarded.
class DedupTable {
  public:
    DedupTable();

    /// Records the triple and reports whether it had already been recorded.
    bool seen(uint16_t node_id, uint16_t boot_count, uint32_t seq);

  private:
    struct Entry {
        uint16_t node_id;
        uint16_t boot_count;
        uint32_t seq;
        bool used;
    };
```

`mesh.cpp`:

```cpp
DedupTable::DedupTable() : entries_(), next_(0) {
    for (size_t i = 0; i < DEDUP_CAPACITY; ++i) {
        entries_[i].node_id = 0;
        entries_[i].boot_count = 0;
        entries_[i].seq = 0;
        entries_[i].used = false;
    }
}

bool DedupTable::seen(uint16_t node_id, uint16_t boot_count, uint32_t seq) {
    for (size_t i = 0; i < DEDUP_CAPACITY; ++i) {
        if (entries_[i].used && entries_[i].node_id == node_id &&
            entries_[i].boot_count == boot_count && entries_[i].seq == seq) {
            return true;
        }
    }

    entries_[next_].node_id = node_id;
    entries_[next_].boot_count = boot_count;
    entries_[next_].seq = seq;
    entries_[next_].used = true;
    next_ = (next_ + 1) % DEDUP_CAPACITY;
    return false;
}

bool should_relay(DedupTable &table, const Packet &p) {
    if (p.ttl == 0) {
        return false;
    }
    return !table.seen(p.node_id, p.boot_count, p.seq);
}
```

`src/gateway_main.cpp`: the dedup call becomes

```cpp
    if (g_dedup.seen(packet.node_id, packet.boot_count, packet.seq)) {
        return;
    }
```

and `print_record()` appends the three fields after `imu_valid`, keeping the existing order:

```cpp
    Serial.print(p.record.gps.valid ? 1 : 0);
    Serial.print(',');
    Serial.print(p.record.imu.valid ? 1 : 0);
    // Appended, not inserted, so consumers of the existing fields are
    // unaffected. v0x01 packets decode these as 0.
    Serial.print(',');
    Serial.print(p.boot_count);
    Serial.print(',');
    Serial.print(p.tx_timeouts);
    Serial.print(',');
    Serial.println(p.battery_mv);
```

(`Serial.println(p.record.imu.valid ? 1 : 0);` becomes `Serial.print(...)` as shown.)

- [ ] **Step 4: Run the tests and build the gateway**

Run: `pio test -e native -f test_mesh && pio run -e gateway`
Expected: all mesh tests PASS; gateway builds with no warnings.

- [ ] **Step 5: Commit**

```bash
git add lib/floodnet_core/include/floodnet/mesh.hpp lib/floodnet_core/src/mesh.cpp src/gateway_main.cpp test/test_mesh/test_main.cpp
git commit -m "fix: key dedup on boot count so a rebooted node is not suppressed"
```

---

### Task 3: UBX-RXM-PMREQ frame builder

**Files:**
- Create: `lib/floodnet_core/include/floodnet/ubx.hpp`
- Create: `lib/floodnet_core/src/ubx.cpp`
- Test: `test/test_ubx/test_main.cpp`

**Interfaces:**
- Produces: `const size_t UBX_PMREQ_BACKUP_SIZE = 24;`, `size_t ubx_pmreq_backup(uint8_t *out, size_t out_len);`, `void ubx_checksum(const uint8_t *data, size_t len, uint8_t *ck_a, uint8_t *ck_b);`

- [ ] **Step 1: Write the failing test**

`test/test_ubx/test_main.cpp`:

```cpp
#include <unity.h>

#include <floodnet/ubx.hpp>

using namespace floodnet;

void setUp(void) {}
void tearDown(void) {}

void test_pmreq_backup_golden_frame(void) {
    uint8_t frame[UBX_PMREQ_BACKUP_SIZE];
    TEST_ASSERT_EQUAL_UINT(UBX_PMREQ_BACKUP_SIZE, ubx_pmreq_backup(frame, sizeof(frame)));

    // Sync, class 0x02 id 0x41, length 16, then version 0, three reserved
    // bytes, duration 0 (indefinite), flags 0x02 (backup), wakeupSources
    // 0x08 (uartrx), then the Fletcher checksum. Computed independently.
    static const uint8_t expected[UBX_PMREQ_BACKUP_SIZE] = {
        0xB5, 0x62, 0x02, 0x41, 0x10, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x02, 0x00, 0x00, 0x00, 0x08, 0x00, 0x00, 0x00, 0x5D, 0x4B,
    };
    TEST_ASSERT_EQUAL_UINT8_ARRAY(expected, frame, UBX_PMREQ_BACKUP_SIZE);
}

void test_pmreq_rejects_short_buffer(void) {
    uint8_t frame[UBX_PMREQ_BACKUP_SIZE - 1];
    TEST_ASSERT_EQUAL_UINT(0, ubx_pmreq_backup(frame, sizeof(frame)));
}

void test_checksum_covers_class_through_payload(void) {
    // UBX-ACK-ACK acknowledging a CFG-PRT (class 0x06, id 0x00):
    // B5 62 05 01 02 00 06 00, checksum 0E 37, worked by hand.
    const uint8_t body[] = {0x05, 0x01, 0x02, 0x00, 0x06, 0x00};
    uint8_t a = 0;
    uint8_t b = 0;
    ubx_checksum(body, sizeof(body), &a, &b);
    TEST_ASSERT_EQUAL_HEX8(0x0E, a);
    TEST_ASSERT_EQUAL_HEX8(0x37, b);
}

int main(int, char **) {
    UNITY_BEGIN();
    RUN_TEST(test_pmreq_backup_golden_frame);
    RUN_TEST(test_pmreq_rejects_short_buffer);
    RUN_TEST(test_checksum_covers_class_through_payload);
    return UNITY_END();
}
```

- [ ] **Step 2: Run to verify it fails**

Run: `pio test -e native -f test_ubx`
Expected: compile failure, `floodnet/ubx.hpp` not found.

- [ ] **Step 3: Implement**

`ubx.hpp`:

```cpp
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
```

`ubx.cpp`:

```cpp
#include <floodnet/ubx.hpp>

namespace floodnet {

void ubx_checksum(const uint8_t *data, size_t len, uint8_t *ck_a, uint8_t *ck_b) {
    uint8_t a = 0;
    uint8_t b = 0;
    for (size_t i = 0; i < len; ++i) {
        a = static_cast<uint8_t>(a + data[i]);
        b = static_cast<uint8_t>(b + a);
    }
    *ck_a = a;
    *ck_b = b;
}

size_t ubx_pmreq_backup(uint8_t *out, size_t out_len) {
    if (out == nullptr || out_len < UBX_PMREQ_BACKUP_SIZE) {
        return 0;
    }

    static const uint8_t PMREQ_CLASS = 0x02;
    static const uint8_t PMREQ_ID = 0x41;
    static const uint8_t PAYLOAD_LEN = 16;
    static const uint8_t FLAG_BACKUP = 0x02;
    static const uint8_t WAKE_UARTRX = 0x08;

    for (size_t i = 0; i < UBX_PMREQ_BACKUP_SIZE; ++i) {
        out[i] = 0;
    }
    out[0] = 0xB5;
    out[1] = 0x62;
    out[2] = PMREQ_CLASS;
    out[3] = PMREQ_ID;
    out[4] = PAYLOAD_LEN;  // little-endian u16; high byte stays 0
    // Payload starts at 6: version 0, reserved[3], duration 0 = indefinite.
    out[14] = FLAG_BACKUP;   // flags, bits 0-7
    out[18] = WAKE_UARTRX;   // wakeupSources, bits 0-7

    ubx_checksum(&out[2], 4 + PAYLOAD_LEN, &out[22], &out[23]);
    return UBX_PMREQ_BACKUP_SIZE;
}

}  // namespace floodnet
```

- [ ] **Step 4: Run to verify it passes**

Run: `pio test -e native -f test_ubx`
Expected: 3 tests PASS.

- [ ] **Step 5: Commit**

```bash
git add lib/floodnet_core/include/floodnet/ubx.hpp lib/floodnet_core/src/ubx.cpp test/test_ubx/test_main.cpp
git commit -m "feat: build the UBX-RXM-PMREQ frame that puts the GPS into backup"
```

---

### Task 4: Node state machine

**Files:**
- Create: `lib/floodnet_core/include/floodnet/node_state.hpp`
- Create: `lib/floodnet_core/src/node_state.cpp`
- Test: `test/test_node_state/test_main.cpp`

**Interfaces:**
- Produces (exact):

```cpp
enum class NodeState : uint8_t { Boot, Acquire, Transmit, Sleep };
enum class NodeEvent : uint8_t { PeripheralsReady, FixQueued, FixQueuedNoImu, TxSucceeded, TxFailed, Tick };
struct NodeTiming { uint32_t report_interval_ms = 300000; uint32_t acquire_timeout_ms = 60000;
                    uint32_t transmit_timeout_ms = 10000; uint32_t sleep_chunk_ms = 60000;
                    uint8_t radio_fail_limit = 3; uint8_t imu_fail_limit = 3; };
struct PowerPlan { bool gps = false; bool imu = false; bool radio = false; };
struct NodeActions { bool transitioned = false; bool kick_watchdog = false; PowerPlan power;
                     bool queue_heartbeat = false; bool power_cycle_radio = false;
                     bool power_cycle_imu = false; uint32_t sleep_until_ms = 0; };
bool time_reached(uint32_t now_ms, uint32_t deadline_ms);
class NodeStateMachine {
  public:
    explicit NodeStateMachine(const NodeTiming &timing = NodeTiming());
    NodeActions boot(uint32_t now_ms);
    NodeActions handle(NodeEvent event, uint32_t now_ms);
    NodeState state() const;
    uint32_t next_slot_ms() const;
    uint8_t radio_failures() const;
    uint8_t imu_failures() const;
};
```

- [ ] **Step 1: Write the failing tests**

`test/test_node_state/test_main.cpp`:

```cpp
#include <unity.h>

#include <floodnet/node_state.hpp>

using namespace floodnet;

void setUp(void) {}
void tearDown(void) {}

static NodeStateMachine booted(uint32_t now) {
    NodeStateMachine m;
    m.boot(now);
    m.handle(NodeEvent::PeripheralsReady, now);
    return m;
}

/// Ticks a sleeping machine at each deadline it asks for until it wakes into
/// ACQUIRE, and returns the wake time. `a` is the action that entered SLEEP.
static uint32_t sleep_until_acquire(NodeStateMachine *m, NodeActions a) {
    uint32_t now = a.sleep_until_ms;
    for (int guard = 0; guard < 100 && m->state() == NodeState::Sleep; ++guard) {
        now = a.sleep_until_ms;
        a = m->handle(NodeEvent::Tick, now);
    }
    return now;
}

/// One full cycle from ACQUIRE: fix at +1000, transmit outcome at +4000.
static NodeActions cycle(NodeStateMachine *m, uint32_t start, NodeEvent fix, NodeEvent tx) {
    m->handle(fix, start + 1000);
    return m->handle(tx, start + 4000);
}

void test_boot_powers_everything_and_kicks(void) {
    NodeStateMachine m;
    NodeActions a = m.boot(0);
    TEST_ASSERT_TRUE(m.state() == NodeState::Boot);
    TEST_ASSERT_TRUE(a.transitioned);
    TEST_ASSERT_TRUE(a.kick_watchdog);
    TEST_ASSERT_TRUE(a.power.gps && a.power.imu && a.power.radio);
}

void test_ready_enters_acquire_with_gps_and_imu(void) {
    NodeStateMachine m;
    m.boot(0);
    NodeActions a = m.handle(NodeEvent::PeripheralsReady, 0);
    TEST_ASSERT_TRUE(m.state() == NodeState::Acquire);
    TEST_ASSERT_TRUE(a.power.gps);
    TEST_ASSERT_TRUE(a.power.imu);
    TEST_ASSERT_FALSE(a.power.radio);
    TEST_ASSERT_EQUAL_UINT32(300000, m.next_slot_ms());
}

void test_fix_moves_to_transmit_and_powers_down_sensors(void) {
    NodeStateMachine m = booted(0);
    NodeActions a = m.handle(NodeEvent::FixQueued, 1200);
    TEST_ASSERT_TRUE(m.state() == NodeState::Transmit);
    TEST_ASSERT_FALSE(a.power.gps);
    TEST_ASSERT_FALSE(a.power.imu);
    TEST_ASSERT_TRUE(a.power.radio);
    TEST_ASSERT_FALSE(a.queue_heartbeat);
}

void test_tick_without_a_transition_does_not_kick(void) {
    NodeStateMachine m = booted(0);
    NodeActions a = m.handle(NodeEvent::Tick, 59999);
    TEST_ASSERT_TRUE(m.state() == NodeState::Acquire);
    TEST_ASSERT_FALSE(a.transitioned);
    TEST_ASSERT_FALSE(a.kick_watchdog);
    TEST_ASSERT_TRUE(a.power.gps);  // a held state still reports its plan
}

void test_acquire_timeout_sends_a_heartbeat(void) {
    NodeStateMachine m = booted(0);
    NodeActions a = m.handle(NodeEvent::Tick, 60000);
    TEST_ASSERT_TRUE(m.state() == NodeState::Transmit);
    TEST_ASSERT_TRUE(a.queue_heartbeat);
    TEST_ASSERT_TRUE(a.kick_watchdog);
}

void test_success_sleeps_until_the_first_chunk(void) {
    NodeStateMachine m = booted(0);
    NodeActions a = cycle(&m, 0, NodeEvent::FixQueued, NodeEvent::TxSucceeded);
    TEST_ASSERT_TRUE(m.state() == NodeState::Sleep);
    TEST_ASSERT_EQUAL_UINT32(64000, a.sleep_until_ms);
    TEST_ASSERT_FALSE(a.power.gps || a.power.imu || a.power.radio);
}

void test_sleep_chunks_renew_and_kick_until_the_slot(void) {
    NodeStateMachine m = booted(0);
    NodeActions a = cycle(&m, 0, NodeEvent::FixQueued, NodeEvent::TxSucceeded);
    const uint32_t expected[] = {124000, 184000, 244000, 300000};
    for (size_t i = 0; i < 4; ++i) {
        a = m.handle(NodeEvent::Tick, a.sleep_until_ms);
        TEST_ASSERT_TRUE(m.state() == NodeState::Sleep);
        TEST_ASSERT_TRUE(a.kick_watchdog);
        TEST_ASSERT_EQUAL_UINT32(expected[i], a.sleep_until_ms);
    }
    a = m.handle(NodeEvent::Tick, 300000);
    TEST_ASSERT_TRUE(m.state() == NodeState::Acquire);
    TEST_ASSERT_EQUAL_UINT32(600000, m.next_slot_ms());
}

void test_early_wake_changes_nothing(void) {
    NodeStateMachine m = booted(0);
    cycle(&m, 0, NodeEvent::FixQueued, NodeEvent::TxSucceeded);
    NodeActions a = m.handle(NodeEvent::Tick, 30000);
    TEST_ASSERT_FALSE(a.transitioned);
    TEST_ASSERT_EQUAL_UINT32(64000, a.sleep_until_ms);
}

void test_schedule_is_anchored_to_the_slot_not_the_work(void) {
    NodeStateMachine m = booted(0);
    m.handle(NodeEvent::Tick, 60000);                                   // slow: timed out
    NodeActions a = m.handle(NodeEvent::TxSucceeded, 63100);
    TEST_ASSERT_EQUAL_UINT32(300000, sleep_until_acquire(&m, a));       // not 363100
    TEST_ASSERT_EQUAL_UINT32(600000, m.next_slot_ms());
}

void test_missed_slots_are_skipped_not_burst(void) {
    NodeStateMachine m = booted(0);
    cycle(&m, 0, NodeEvent::FixQueued, NodeEvent::TxSucceeded);
    m.handle(NodeEvent::Tick, 950000);  // woke far too late
    TEST_ASSERT_TRUE(m.state() == NodeState::Acquire);
    TEST_ASSERT_EQUAL_UINT32(1200000, m.next_slot_ms());
}

void test_three_failed_transmits_power_cycle_the_radio(void) {
    NodeStateMachine m = booted(0);
    uint32_t start = 0;
    for (int i = 0; i < 2; ++i) {
        NodeActions a = cycle(&m, start, NodeEvent::FixQueued, NodeEvent::TxFailed);
        TEST_ASSERT_FALSE(a.power_cycle_radio);
        start = sleep_until_acquire(&m, a);
    }
    NodeActions a = cycle(&m, start, NodeEvent::FixQueued, NodeEvent::TxFailed);
    TEST_ASSERT_TRUE(a.power_cycle_radio);
    TEST_ASSERT_EQUAL_UINT8(0, m.radio_failures());
}

void test_success_resets_the_radio_failure_count(void) {
    NodeStateMachine m = booted(0);
    const NodeEvent outcomes[] = {NodeEvent::TxFailed, NodeEvent::TxFailed,
                                  NodeEvent::TxSucceeded, NodeEvent::TxFailed};
    uint32_t start = 0;
    for (size_t i = 0; i < 4; ++i) {
        NodeActions a = cycle(&m, start, NodeEvent::FixQueued, outcomes[i]);
        TEST_ASSERT_FALSE(a.power_cycle_radio);
        start = sleep_until_acquire(&m, a);
    }
    TEST_ASSERT_EQUAL_UINT8(1, m.radio_failures());
}

void test_transmit_timeout_counts_as_a_failure(void) {
    NodeStateMachine m = booted(0);
    m.handle(NodeEvent::FixQueued, 1000);
    NodeActions a = m.handle(NodeEvent::Tick, 10999);
    TEST_ASSERT_TRUE(m.state() == NodeState::Transmit);
    TEST_ASSERT_FALSE(a.transitioned);
    m.handle(NodeEvent::Tick, 11000);
    TEST_ASSERT_TRUE(m.state() == NodeState::Sleep);
    TEST_ASSERT_EQUAL_UINT8(1, m.radio_failures());
}

void test_three_wakes_without_imu_power_cycle_the_imu(void) {
    NodeStateMachine m = booted(0);
    uint32_t start = 0;
    for (int i = 0; i < 2; ++i) {
        NodeActions a = m.handle(NodeEvent::FixQueuedNoImu, start + 1000);
        TEST_ASSERT_FALSE(a.power_cycle_imu);
        start = sleep_until_acquire(&m, m.handle(NodeEvent::TxSucceeded, start + 4000));
    }
    NodeActions a = m.handle(NodeEvent::FixQueuedNoImu, start + 1000);
    TEST_ASSERT_TRUE(a.power_cycle_imu);
    TEST_ASSERT_EQUAL_UINT8(0, m.imu_failures());
}

void test_heartbeat_does_not_count_against_the_imu(void) {
    NodeStateMachine m = booted(0);
    m.handle(NodeEvent::FixQueuedNoImu, 1000);
    NodeActions a = m.handle(NodeEvent::TxSucceeded, 4000);
    const uint32_t start = sleep_until_acquire(&m, a);
    m.handle(NodeEvent::Tick, start + 60000);  // no fix at all: heartbeat
    TEST_ASSERT_EQUAL_UINT8(1, m.imu_failures());
}

void test_schedule_survives_millis_wrap(void) {
    const uint32_t t0 = 0xFFFF0000u;  // about 65 s before millis() wraps
    NodeStateMachine m = booted(t0);
    NodeActions a = cycle(&m, t0, NodeEvent::FixQueued, NodeEvent::TxSucceeded);
    TEST_ASSERT_EQUAL_UINT32(t0 + 64000u, a.sleep_until_ms);  // the slot below lies past the wrap
    TEST_ASSERT_EQUAL_UINT32(t0 + 300000u, sleep_until_acquire(&m, a));
    TEST_ASSERT_EQUAL_UINT32(t0 + 600000u, m.next_slot_ms());
}

void test_unrelated_events_are_ignored(void) {
    NodeStateMachine m = booted(0);
    NodeActions a = m.handle(NodeEvent::TxSucceeded, 500);
    TEST_ASSERT_TRUE(m.state() == NodeState::Acquire);
    TEST_ASSERT_FALSE(a.transitioned);

    cycle(&m, 0, NodeEvent::FixQueued, NodeEvent::TxSucceeded);
    a = m.handle(NodeEvent::FixQueued, 5000);
    TEST_ASSERT_TRUE(m.state() == NodeState::Sleep);
    TEST_ASSERT_FALSE(a.transitioned);
}

int main(int, char **) {
    UNITY_BEGIN();
    RUN_TEST(test_boot_powers_everything_and_kicks);
    RUN_TEST(test_ready_enters_acquire_with_gps_and_imu);
    RUN_TEST(test_fix_moves_to_transmit_and_powers_down_sensors);
    RUN_TEST(test_tick_without_a_transition_does_not_kick);
    RUN_TEST(test_acquire_timeout_sends_a_heartbeat);
    RUN_TEST(test_success_sleeps_until_the_first_chunk);
    RUN_TEST(test_sleep_chunks_renew_and_kick_until_the_slot);
    RUN_TEST(test_early_wake_changes_nothing);
    RUN_TEST(test_schedule_is_anchored_to_the_slot_not_the_work);
    RUN_TEST(test_missed_slots_are_skipped_not_burst);
    RUN_TEST(test_three_failed_transmits_power_cycle_the_radio);
    RUN_TEST(test_success_resets_the_radio_failure_count);
    RUN_TEST(test_transmit_timeout_counts_as_a_failure);
    RUN_TEST(test_three_wakes_without_imu_power_cycle_the_imu);
    RUN_TEST(test_heartbeat_does_not_count_against_the_imu);
    RUN_TEST(test_schedule_survives_millis_wrap);
    RUN_TEST(test_unrelated_events_are_ignored);
    return UNITY_END();
}
```

- [ ] **Step 2: Run to verify it fails**

Run: `pio test -e native -f test_node_state`
Expected: compile failure, `floodnet/node_state.hpp` not found.

- [ ] **Step 3: Implement**

`node_state.hpp`:

```cpp
#ifndef FLOODNET_NODE_STATE_HPP
#define FLOODNET_NODE_STATE_HPP

#include <stdint.h>

namespace floodnet {

enum class NodeState : uint8_t { Boot, Acquire, Transmit, Sleep };

/// What the caller observed since the last call. `Tick` carries only the
/// time, and is how timeouts, slot wakes and watchdog chunks are detected.
enum class NodeEvent : uint8_t {
    PeripheralsReady,
    FixQueued,       ///< one valid fix queued, with a paired IMU sample
    FixQueuedNoImu,  ///< one valid fix queued, without one
    TxSucceeded,     ///< queue drained and at least one transmit completed
    TxFailed,        ///< queue drained and no transmit completed
    Tick,
};

/// Every value is from the milestone 3 design doc, "Watchdog timing".
struct NodeTiming {
    uint32_t report_interval_ms = 300000;
    /// Over twice the NEO-M8N's 26 s typical cold-start TTFF.
    uint32_t acquire_timeout_ms = 60000;
    /// Twice the sampler's 5 s transmit timeout, so a radio refusing every
    /// begin_transmit() cannot hold the node in TRANSMIT.
    uint32_t transmit_timeout_ms = 10000;
    /// A 5-minute sleep outlasts WDOG1's 128 s ceiling, so sleep is taken in
    /// chunks, each one a transition that kicks the watchdog.
    uint32_t sleep_chunk_ms = 60000;
    uint8_t radio_fail_limit = 3;
    uint8_t imu_fail_limit = 3;
};

/// Which peripherals should be powered. Reported on every call, so a caller
/// can apply it only when it changes.
struct PowerPlan {
    bool gps = false;
    bool imu = false;
    bool radio = false;
};

struct NodeActions {
    /// State entered (or a sleep chunk renewed) on this call.
    bool transitioned = false;
    /// Set exactly when `transitioned` is. Kicking only on progress is the
    /// point: a loop that spins while stuck in one state is still reset.
    bool kick_watchdog = false;
    PowerPlan power;
    bool queue_heartbeat = false;
    bool power_cycle_radio = false;
    bool power_cycle_imu = false;
    /// Meaningful only in SLEEP: when to call back with a Tick.
    uint32_t sleep_until_ms = 0;
};

/// Wrap-safe "has `deadline_ms` passed": correct across the millis() rollover
/// for any deadline within 2^31 ms (24.8 days) of now.
bool time_reached(uint32_t now_ms, uint32_t deadline_ms);

/// The duty-cycle and recovery policy, as a pure transition function.
///
/// Holds no hardware reference: the caller reports events and applies the
/// returned actions. That is what lets every transition be tested on the host
/// with hand-built event sequences. See the milestone 3 design doc.
class NodeStateMachine {
  public:
    explicit NodeStateMachine(const NodeTiming &timing = NodeTiming());

    /// Enters BOOT. The first report slot is `now_ms`.
    NodeActions boot(uint32_t now_ms);

    NodeActions handle(NodeEvent event, uint32_t now_ms);

    NodeState state() const { return state_; }
    uint32_t next_slot_ms() const { return next_slot_ms_; }
    uint8_t radio_failures() const { return radio_failures_; }
    uint8_t imu_failures() const { return imu_failures_; }

  private:
    NodeActions enter(NodeState next, uint32_t now_ms);
    NodeActions hold() const;

    NodeTiming timing_;
    NodeState state_;
    uint32_t entered_ms_;
    uint32_t next_slot_ms_;
    uint32_t sleep_until_ms_;
    uint8_t radio_failures_;
    uint8_t imu_failures_;
};

}  // namespace floodnet

#endif  // FLOODNET_NODE_STATE_HPP
```

`node_state.cpp`:

```cpp
#include <floodnet/node_state.hpp>

namespace floodnet {

namespace {

PowerPlan plan_for(NodeState state) {
    PowerPlan plan;
    switch (state) {
    case NodeState::Boot:
        plan.gps = true;
        plan.imu = true;
        plan.radio = true;
        break;
    case NodeState::Acquire:
        plan.gps = true;
        plan.imu = true;
        break;
    case NodeState::Transmit:
        plan.radio = true;
        break;
    case NodeState::Sleep:
        break;
    }
    return plan;
}

}  // namespace

bool time_reached(uint32_t now_ms, uint32_t deadline_ms) {
    return static_cast<int32_t>(now_ms - deadline_ms) >= 0;
}

NodeStateMachine::NodeStateMachine(const NodeTiming &timing)
    : timing_(timing),
      state_(NodeState::Boot),
      entered_ms_(0),
      next_slot_ms_(0),
      sleep_until_ms_(0),
      radio_failures_(0),
      imu_failures_(0) {}

NodeActions NodeStateMachine::boot(uint32_t now_ms) {
    radio_failures_ = 0;
    imu_failures_ = 0;
    next_slot_ms_ = now_ms;
    return enter(NodeState::Boot, now_ms);
}

NodeActions NodeStateMachine::handle(NodeEvent event, uint32_t now_ms) {
    switch (state_) {
    case NodeState::Boot:
        if (event == NodeEvent::PeripheralsReady) {
            return enter(NodeState::Acquire, now_ms);
        }
        break;

    case NodeState::Acquire:
        if (event == NodeEvent::FixQueued) {
            imu_failures_ = 0;
            return enter(NodeState::Transmit, now_ms);
        }
        if (event == NodeEvent::FixQueuedNoImu) {
            NodeActions actions = enter(NodeState::Transmit, now_ms);
            if (++imu_failures_ >= timing_.imu_fail_limit) {
                actions.power_cycle_imu = true;
                imu_failures_ = 0;
            }
            return actions;
        }
        if (event == NodeEvent::Tick &&
            time_reached(now_ms, entered_ms_ + timing_.acquire_timeout_ms)) {
            // No fix: still transmit, so the gateway can tell a node that
            // cannot see the sky from a node that is dead. Not an IMU failure.
            NodeActions actions = enter(NodeState::Transmit, now_ms);
            actions.queue_heartbeat = true;
            return actions;
        }
        break;

    case NodeState::Transmit:
        if (event == NodeEvent::TxSucceeded) {
            radio_failures_ = 0;
            return enter(NodeState::Sleep, now_ms);
        }
        if (event == NodeEvent::TxFailed ||
            (event == NodeEvent::Tick &&
             time_reached(now_ms, entered_ms_ + timing_.transmit_timeout_ms))) {
            NodeActions actions = enter(NodeState::Sleep, now_ms);
            if (++radio_failures_ >= timing_.radio_fail_limit) {
                actions.power_cycle_radio = true;
                radio_failures_ = 0;
            }
            return actions;
        }
        break;

    case NodeState::Sleep:
        if (event == NodeEvent::Tick) {
            if (time_reached(now_ms, next_slot_ms_)) {
                return enter(NodeState::Acquire, now_ms);
            }
            if (time_reached(now_ms, sleep_until_ms_)) {
                return enter(NodeState::Sleep, now_ms);  // next chunk
            }
        }
        break;
    }
    return hold();
}

NodeActions NodeStateMachine::hold() const {
    NodeActions actions;
    actions.power = plan_for(state_);
    actions.sleep_until_ms = sleep_until_ms_;
    return actions;
}

NodeActions NodeStateMachine::enter(NodeState next, uint32_t now_ms) {
    state_ = next;
    entered_ms_ = now_ms;

    if (next == NodeState::Acquire) {
        // Anchored: advance from the slot just taken, not from now, so a slow
        // acquisition delays one report rather than every report after it.
        // A slot already missed is skipped rather than chased.
        next_slot_ms_ += timing_.report_interval_ms;
        while (time_reached(now_ms, next_slot_ms_)) {
            next_slot_ms_ += timing_.report_interval_ms;
        }
    }
    if (next == NodeState::Sleep) {
        const uint32_t chunk_end = now_ms + timing_.sleep_chunk_ms;
        sleep_until_ms_ = time_reached(chunk_end, next_slot_ms_) ? next_slot_ms_ : chunk_end;
    }

    NodeActions actions;
    actions.transitioned = true;
    actions.kick_watchdog = true;
    actions.power = plan_for(next);
    actions.sleep_until_ms = sleep_until_ms_;
    return actions;
}

}  // namespace floodnet
```

- [ ] **Step 4: Run to verify it passes**

Run: `pio test -e native -f test_node_state`
Expected: 17 tests PASS.

- [ ] **Step 5: Commit**

```bash
git add lib/floodnet_core/include/floodnet/node_state.hpp lib/floodnet_core/src/node_state.cpp test/test_node_state/test_main.cpp
git commit -m "feat: add the node state machine for duty cycling and recovery"
```

---

### Task 5: Downtime calculation

**Files:**
- Create: `lib/floodnet_core/include/floodnet/downtime.hpp`
- Create: `lib/floodnet_core/src/downtime.cpp`
- Test: `test/test_downtime/test_main.cpp`

**Interfaces:**
- Produces (exact):

```cpp
enum class DownCause : uint8_t { Startup = 0, Battery, NoGps, Radio, Reboot, Silent };
const size_t DOWN_CAUSE_COUNT = 6;
struct GatewayRecord { uint32_t arrival_ms = 0; bool gps_valid = false; uint16_t boot_count = 0; uint16_t tx_timeouts = 0; };
struct DowntimeInput { const GatewayRecord *records = nullptr; size_t count = 0; uint32_t scenario_ms = 0;
                       uint32_t stale_after_ms = 600000; bool died = false; uint32_t died_at_ms = 0; };
struct DowntimeReport { uint32_t down_ms[DOWN_CAUSE_COUNT] = {}; uint32_t total_down_ms = 0; bool down_at_end = false; };
DowntimeReport compute_downtime(const DowntimeInput &in);
const char *down_cause_name(DownCause cause);
```

The spec's cause table gains `startup` here: the interval before a node's first valid record, which the spec's definition counts as down but no other cause describes. Task 12 records this in the spec's revisions.

- [ ] **Step 1: Write the failing tests**

`test/test_downtime/test_main.cpp`:

```cpp
#include <unity.h>

#include <floodnet/downtime.hpp>

using namespace floodnet;

void setUp(void) {}
void tearDown(void) {}

static GatewayRecord rec(uint32_t arrival, bool valid, uint16_t boot = 1, uint16_t tx = 0) {
    GatewayRecord r;
    r.arrival_ms = arrival;
    r.gps_valid = valid;
    r.boot_count = boot;
    r.tx_timeouts = tx;
    return r;
}

static DowntimeReport run(const GatewayRecord *records, size_t count, uint32_t scenario_ms) {
    DowntimeInput in;
    in.records = records;
    in.count = count;
    in.scenario_ms = scenario_ms;
    return compute_downtime(in);
}

static uint32_t cause(const DowntimeReport &r, DownCause c) {
    return r.down_ms[static_cast<size_t>(c)];
}

void test_no_records_is_all_startup(void) {
    DowntimeReport r = run(nullptr, 0, 3600000);
    TEST_ASSERT_EQUAL_UINT32(3600000, cause(r, DownCause::Startup));
    TEST_ASSERT_EQUAL_UINT32(3600000, r.total_down_ms);
    TEST_ASSERT_TRUE(r.down_at_end);
}

void test_steady_reports_leave_only_startup(void) {
    GatewayRecord records[12];
    for (uint32_t k = 0; k < 12; ++k) {
        records[k] = rec(29000 + k * 300000, true);
    }
    DowntimeReport r = run(records, 12, 3600000);
    TEST_ASSERT_EQUAL_UINT32(29000, cause(r, DownCause::Startup));
    TEST_ASSERT_EQUAL_UINT32(29000, r.total_down_ms);
    TEST_ASSERT_FALSE(r.down_at_end);
}

void test_gap_longer_than_two_intervals_is_silent(void) {
    const GatewayRecord records[] = {rec(1000, true), rec(301000, true), rec(1201000, true)};
    DowntimeReport r = run(records, 3, 1300000);
    TEST_ASSERT_EQUAL_UINT32(300000, cause(r, DownCause::Silent));  // 901000 to 1201000
    TEST_ASSERT_EQUAL_UINT32(301000, r.total_down_ms);
}

void test_exactly_stale_after_is_not_down(void) {
    const GatewayRecord records[] = {rec(1000, true), rec(601000, true)};
    DowntimeReport r = run(records, 2, 700000);
    TEST_ASSERT_EQUAL_UINT32(1000, r.total_down_ms);  // startup only
}

void test_heartbeats_during_a_gap_attribute_to_no_gps(void) {
    const GatewayRecord records[] = {rec(1000, true), rec(301000, false), rec(601000, false),
                                     rec(901000, false), rec(1201000, true)};
    DowntimeReport r = run(records, 5, 1300000);
    TEST_ASSERT_EQUAL_UINT32(600000, cause(r, DownCause::NoGps));
}

void test_heartbeat_before_the_gap_does_not_claim_it(void) {
    // A heartbeat while still fresh, then silence: that later silence was not
    // explained by the heartbeat.
    const GatewayRecord records[] = {rec(1000, true), rec(301000, false), rec(1201000, true)};
    DowntimeReport r = run(records, 3, 1300000);
    TEST_ASSERT_EQUAL_UINT32(0, cause(r, DownCause::NoGps));
    TEST_ASSERT_EQUAL_UINT32(600000, cause(r, DownCause::Silent));
}

void test_boot_count_change_attributes_to_reboot(void) {
    const GatewayRecord records[] = {rec(1000, true, 1), rec(1001000, true, 2)};
    DowntimeReport r = run(records, 2, 1100000);
    TEST_ASSERT_EQUAL_UINT32(400000, cause(r, DownCause::Reboot));
}

void test_tx_timeouts_increase_attributes_to_radio(void) {
    const GatewayRecord records[] = {rec(1000, true, 1, 0), rec(1001000, true, 1, 3)};
    DowntimeReport r = run(records, 2, 1100000);
    TEST_ASSERT_EQUAL_UINT32(400000, cause(r, DownCause::Radio));
}

void test_reboot_takes_precedence_over_radio(void) {
    const GatewayRecord records[] = {rec(1000, true, 1, 0), rec(1001000, true, 2, 5)};
    DowntimeReport r = run(records, 2, 1100000);
    TEST_ASSERT_EQUAL_UINT32(400000, cause(r, DownCause::Reboot));
    TEST_ASSERT_EQUAL_UINT32(0, cause(r, DownCause::Radio));
}

void test_battery_death_splits_the_interval(void) {
    const GatewayRecord records[] = {rec(1000, true)};
    DowntimeInput in;
    in.records = records;
    in.count = 1;
    in.scenario_ms = 1000000;
    in.died = true;
    in.died_at_ms = 700000;
    DowntimeReport r = compute_downtime(in);
    TEST_ASSERT_EQUAL_UINT32(99000, cause(r, DownCause::Silent));   // 601000 to 700000
    TEST_ASSERT_EQUAL_UINT32(300000, cause(r, DownCause::Battery));
    TEST_ASSERT_TRUE(r.down_at_end);
}

void test_death_before_any_record_is_battery_after_startup(void) {
    DowntimeInput in;
    in.scenario_ms = 10000;
    in.died = true;
    in.died_at_ms = 5000;
    DowntimeReport r = compute_downtime(in);
    TEST_ASSERT_EQUAL_UINT32(5000, cause(r, DownCause::Startup));
    TEST_ASSERT_EQUAL_UINT32(5000, cause(r, DownCause::Battery));
}

void test_records_after_the_scenario_are_ignored(void) {
    const GatewayRecord records[] = {rec(1000, true), rec(2000000, true)};
    DowntimeReport r = run(records, 2, 1000000);
    TEST_ASSERT_EQUAL_UINT32(399000, cause(r, DownCause::Silent));
}

void test_cause_names_are_stable(void) {
    TEST_ASSERT_EQUAL_STRING("startup", down_cause_name(DownCause::Startup));
    TEST_ASSERT_EQUAL_STRING("battery", down_cause_name(DownCause::Battery));
    TEST_ASSERT_EQUAL_STRING("no_gps", down_cause_name(DownCause::NoGps));
    TEST_ASSERT_EQUAL_STRING("radio", down_cause_name(DownCause::Radio));
    TEST_ASSERT_EQUAL_STRING("reboot", down_cause_name(DownCause::Reboot));
    TEST_ASSERT_EQUAL_STRING("silent", down_cause_name(DownCause::Silent));
}

int main(int, char **) {
    UNITY_BEGIN();
    RUN_TEST(test_no_records_is_all_startup);
    RUN_TEST(test_steady_reports_leave_only_startup);
    RUN_TEST(test_gap_longer_than_two_intervals_is_silent);
    RUN_TEST(test_exactly_stale_after_is_not_down);
    RUN_TEST(test_heartbeats_during_a_gap_attribute_to_no_gps);
    RUN_TEST(test_heartbeat_before_the_gap_does_not_claim_it);
    RUN_TEST(test_boot_count_change_attributes_to_reboot);
    RUN_TEST(test_tx_timeouts_increase_attributes_to_radio);
    RUN_TEST(test_reboot_takes_precedence_over_radio);
    RUN_TEST(test_battery_death_splits_the_interval);
    RUN_TEST(test_death_before_any_record_is_battery_after_startup);
    RUN_TEST(test_records_after_the_scenario_are_ignored);
    RUN_TEST(test_cause_names_are_stable);
    return UNITY_END();
}
```

- [ ] **Step 2: Run to verify it fails**

Run: `pio test -e native -f test_downtime`
Expected: compile failure, `floodnet/downtime.hpp` not found.

- [ ] **Step 3: Implement**

`downtime.hpp`:

```cpp
#ifndef FLOODNET_DOWNTIME_HPP
#define FLOODNET_DOWNTIME_HPP

#include <stddef.h>
#include <stdint.h>

namespace floodnet {

/// Why a node was down. See the milestone 3 design doc, "Cause attribution".
enum class DownCause : uint8_t {
    Startup = 0,  ///< before the first valid record of the run
    Battery,      ///< the node had died; known to a simulation, not to a gateway
    NoGps,        ///< heartbeats (gps_valid = 0) arrived during the interval
    Radio,        ///< the record ending the interval shows more tx_timeouts
    Reboot,       ///< the record ending the interval shows a new boot_count
    Silent,       ///< down, and nothing in-band explains why
};
const size_t DOWN_CAUSE_COUNT = 6;

/// What a gateway knows about one received packet.
struct GatewayRecord {
    uint32_t arrival_ms = 0;
    bool gps_valid = false;
    uint16_t boot_count = 0;
    uint16_t tx_timeouts = 0;
};

struct DowntimeInput {
    const GatewayRecord *records = nullptr;  ///< in arrival order
    size_t count = 0;
    /// Run length, starting at 0. Times are plain uint32 milliseconds, so a
    /// run must stay under 2^32 ms (49.7 days); runs here are at most 45 days.
    uint32_t scenario_ms = 0;
    /// Down once the newest valid record is older than this: two intervals.
    uint32_t stale_after_ms = 600000;
    bool died = false;
    uint32_t died_at_ms = 0;
};

struct DowntimeReport {
    uint32_t down_ms[DOWN_CAUSE_COUNT] = {};
    uint32_t total_down_ms = 0;
    bool down_at_end = false;  ///< still down when the run ended
};

/// Total down time and its split by cause. A node is down at time t when its
/// newest gps_valid record arrived more than stale_after_ms before t, or when
/// it has none yet. A record exactly stale_after_ms old is not down.
DowntimeReport compute_downtime(const DowntimeInput &in);

const char *down_cause_name(DownCause cause);

}  // namespace floodnet

#endif  // FLOODNET_DOWNTIME_HPP
```

`downtime.cpp`:

```cpp
#include <floodnet/downtime.hpp>

namespace floodnet {

namespace {

void add(DowntimeReport *report, DownCause cause, uint32_t ms) {
    report->down_ms[static_cast<size_t>(cause)] += ms;
    report->total_down_ms += ms;
}

/// Attributes [start, end). `prev` is the last valid record before the
/// interval (null before the first); `next` the valid record that ends it
/// (null when the run ends first). `heartbeat` is whether a gps_valid = 0
/// record arrived inside the interval.
void attribute(DowntimeReport *report, const DowntimeInput &in, uint32_t start, uint32_t end,
               const GatewayRecord *prev, const GatewayRecord *next, bool heartbeat) {
    if (end <= start) {
        return;
    }

    uint32_t alive_end = end;
    if (in.died && in.died_at_ms < end) {
        const uint32_t dead_from = in.died_at_ms > start ? in.died_at_ms : start;
        add(report, DownCause::Battery, end - dead_from);
        alive_end = dead_from;
    }
    if (alive_end <= start) {
        return;
    }

    DownCause cause = DownCause::Silent;
    if (prev == nullptr) {
        cause = DownCause::Startup;
    } else if (heartbeat) {
        cause = DownCause::NoGps;
    } else if (next != nullptr && next->boot_count != prev->boot_count) {
        cause = DownCause::Reboot;
    } else if (next != nullptr && next->tx_timeouts > prev->tx_timeouts) {
        cause = DownCause::Radio;
    }
    add(report, cause, alive_end - start);
}

}  // namespace

DowntimeReport compute_downtime(const DowntimeInput &in) {
    DowntimeReport report;
    const GatewayRecord *prev = nullptr;
    uint32_t fresh_until = 0;
    bool have_heartbeat = false;
    uint32_t last_heartbeat_ms = 0;

    for (size_t i = 0; i < in.count; ++i) {
        const GatewayRecord &record = in.records[i];
        if (record.arrival_ms >= in.scenario_ms) {
            break;
        }
        if (!record.gps_valid) {
            have_heartbeat = true;
            last_heartbeat_ms = record.arrival_ms;
            continue;
        }
        // Heartbeats arrive in order, so "any heartbeat inside the interval"
        // is "the latest one is at or after its start".
        const bool heartbeat = have_heartbeat && last_heartbeat_ms >= fresh_until;
        attribute(&report, in, fresh_until, record.arrival_ms, prev, &record, heartbeat);
        prev = &record;
        have_heartbeat = false;
        fresh_until = record.arrival_ms + in.stale_after_ms;
    }

    if (fresh_until < in.scenario_ms) {
        const bool heartbeat = have_heartbeat && last_heartbeat_ms >= fresh_until;
        attribute(&report, in, fresh_until, in.scenario_ms, prev, nullptr, heartbeat);
        report.down_at_end = true;
    }
    return report;
}

const char *down_cause_name(DownCause cause) {
    switch (cause) {
    case DownCause::Startup:
        return "startup";
    case DownCause::Battery:
        return "battery";
    case DownCause::NoGps:
        return "no_gps";
    case DownCause::Radio:
        return "radio";
    case DownCause::Reboot:
        return "reboot";
    case DownCause::Silent:
        return "silent";
    }
    return "unknown";
}

}  // namespace floodnet
```

- [ ] **Step 4: Run to verify it passes**

Run: `pio test -e native -f test_downtime`
Expected: 13 tests PASS.

- [ ] **Step 5: Commit**

```bash
git add lib/floodnet_core/include/floodnet/downtime.hpp lib/floodnet_core/src/downtime.cpp test/test_downtime/test_main.cpp
git commit -m "feat: compute sensor downtime and its cause from gateway records"
```

---

### Task 6: Powered fakes, GPS time to first fix, and fault hooks

**Files:**
- Modify: `test/support/sim_clock.hpp`
- Modify: `test/support/fake_gps.hpp`
- Modify: `test/support/fake_imu.hpp`
- Modify: `test/support/fake_radio.hpp`
- Create: `test/support/hang_breaker.hpp`
- Test: `test/test_fakes/test_main.cpp`

**Interfaces:**
- Consumes: `parse_gga`, `NmeaLineAssembler` (existing, for the GPS tests).
- Produces:
  - `SimClock::advance_to(uint32_t t)`; observer capacity 8.
  - `class IHangBreaker { virtual bool should_break() const = 0; };`
  - `FakeGps`: `NO_FIX_SENTENCE`, `HOT_START_MS` 1000, `COLD_START_MS` 26000, `EPHEMERIS_LIFETIME_MS` 7 200 000, `set_powered(bool)`, `powered()`, `set_sky_blocked(bool)`, `acquiring()`, `inject(const char *)`.
  - `FakeImu`: `set_powered(bool)`, `powered()`, `set_failing(bool)`, `hang_next_read(const IHangBreaker *)`, `power_cycle()`, `power_cycles()`.
  - `FakeRadio`: `set_powered(bool)`, `powered()`, `set_wedged(bool)`, `set_refusing(bool)`, `power_cycle()`, `power_cycles()`, `transmitting()`.

- [ ] **Step 1: Write the failing tests**

Add to `test/test_fakes/test_main.cpp` (add `#include <floodnet/nmea.hpp>` and `#include "../support/hang_breaker.hpp"` to the includes):

```cpp
namespace {

struct CountingTick : public ISimTick {
    size_t calls = 0;
    uint32_t total_ms = 0;
    void on_tick(uint32_t elapsed_ms) override {
        ++calls;
        total_ms += elapsed_ms;
    }
    bool pending() const override { return false; }
};

struct AtTime : public IHangBreaker {
    SimClock &clock;
    uint32_t at_ms;
    AtTime(SimClock &c, uint32_t t) : clock(c), at_ms(t) {}
    bool should_break() const override { return clock.now_ms() >= at_ms; }
};

const char kFix[] = "$GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,*47\r\n";

/// Advances 1 ms at a time, parsing what the GPS emits, and returns the time
/// the first valid fix completes, or UINT32_MAX if none by `limit_ms`.
uint32_t first_valid_fix_ms(SimClock &clock, FakeGps &gps, uint32_t limit_ms) {
    NmeaLineAssembler line;
    while (clock.now_ms() < limit_ms) {
        clock.delay_ms(1);
        for (int b = gps.read_byte(); b >= 0; b = gps.read_byte()) {
            if (!line.feed(static_cast<char>(b))) {
                continue;
            }
            GpsFix fix;
            if (parse_gga(line.sentence(), line.length(), clock.now_ms(), &fix) && fix.valid) {
                return clock.now_ms();
            }
        }
    }
    return UINT32_MAX;
}

}  // namespace

void test_advance_to_delivers_one_tick(void) {
    SimClock clock;
    CountingTick tick;
    clock.add_observer(&tick);
    clock.advance_to(5000);
    TEST_ASSERT_EQUAL_UINT(1, tick.calls);
    TEST_ASSERT_EQUAL_UINT32(5000, tick.total_ms);
    TEST_ASSERT_EQUAL_UINT32(5000, clock.now_ms());

    clock.advance_to(4000);  // already past: nothing happens
    TEST_ASSERT_EQUAL_UINT(1, tick.calls);
    TEST_ASSERT_EQUAL_UINT32(5000, clock.now_ms());
}

void test_clock_accepts_eight_observers(void) {
    SimClock clock;
    CountingTick ticks[8];
    for (size_t i = 0; i < 8; ++i) {
        clock.add_observer(&ticks[i]);
    }
    clock.delay_ms(1);
    TEST_ASSERT_EQUAL_UINT(1, ticks[7].calls);
}

void test_default_gps_still_fixes_immediately(void) {
    SimClock clock;
    FakeGps gps(kFix, 0.96, FakeGps::MAX_FIFO_DEPTH);
    clock.add_observer(&gps);
    TEST_ASSERT_FALSE(gps.acquiring());
    TEST_ASSERT_LESS_OR_EQUAL_UINT32(72, first_valid_fix_ms(clock, gps, 1000));
}

void test_unpowered_gps_emits_nothing(void) {
    SimClock clock;
    FakeGps gps(kFix, 0.96, FakeGps::MAX_FIFO_DEPTH);
    clock.add_observer(&gps);
    gps.set_powered(false);
    clock.delay_ms(1000);
    TEST_ASSERT_EQUAL_INT(-1, gps.read_byte());
    TEST_ASSERT_FALSE(gps.pending());
}

void test_gps_cold_start_sends_no_fix_until_ttff(void) {
    SimClock clock;
    FakeGps gps(kFix, 0.96, FakeGps::MAX_FIFO_DEPTH);
    clock.add_observer(&gps);
    gps.set_powered(false);
    gps.set_powered(true);  // never had a fix: cold
    TEST_ASSERT_TRUE(gps.acquiring());

    const uint32_t first = first_valid_fix_ms(clock, gps, 40000);
    TEST_ASSERT_GREATER_OR_EQUAL_UINT32(FakeGps::COLD_START_MS, first);
    TEST_ASSERT_LESS_OR_EQUAL_UINT32(FakeGps::COLD_START_MS + 120, first);
    TEST_ASSERT_FALSE(gps.acquiring());
}

void test_gps_hot_start_after_a_short_backup(void) {
    SimClock clock;
    FakeGps gps(kFix, 0.96, FakeGps::MAX_FIFO_DEPTH);
    clock.add_observer(&gps);
    first_valid_fix_ms(clock, gps, 1000);  // has a fix
    gps.set_powered(false);
    clock.delay_ms(300000);
    while (gps.read_byte() >= 0) {
    }
    gps.set_powered(true);
    const uint32_t start = clock.now_ms();

    const uint32_t first = first_valid_fix_ms(clock, gps, start + 40000);
    TEST_ASSERT_GREATER_OR_EQUAL_UINT32(start + FakeGps::HOT_START_MS, first);
    TEST_ASSERT_LESS_OR_EQUAL_UINT32(start + FakeGps::HOT_START_MS + 120, first);
}

void test_gps_cold_start_after_ephemeris_expires(void) {
    SimClock clock;
    FakeGps gps(kFix, 0.96, FakeGps::MAX_FIFO_DEPTH);
    clock.add_observer(&gps);
    first_valid_fix_ms(clock, gps, 1000);
    gps.set_powered(false);
    clock.delay_ms(FakeGps::EPHEMERIS_LIFETIME_MS);
    gps.set_powered(true);
    const uint32_t start = clock.now_ms();
    TEST_ASSERT_GREATER_OR_EQUAL_UINT32(start + FakeGps::COLD_START_MS,
                                        first_valid_fix_ms(clock, gps, start + 40000));
}

void test_sky_blocked_gps_never_fixes(void) {
    SimClock clock;
    FakeGps gps(kFix, 0.96, FakeGps::MAX_FIFO_DEPTH);
    clock.add_observer(&gps);
    gps.set_sky_blocked(true);
    TEST_ASSERT_EQUAL_UINT32(UINT32_MAX, first_valid_fix_ms(clock, gps, 60000));
    TEST_ASSERT_TRUE(gps.acquiring());
}

void test_injected_bytes_are_readable_while_unpowered(void) {
    SimClock clock;
    FakeGps gps(kFix, 0.96, FakeGps::MAX_FIFO_DEPTH);
    gps.set_powered(false);
    gps.inject("$AB");
    TEST_ASSERT_EQUAL_INT('$', gps.read_byte());
    TEST_ASSERT_EQUAL_INT('A', gps.read_byte());
}

void test_unpowered_imu_is_never_ready(void) {
    SimClock clock;
    FakeImu imu(clock, 10);
    clock.add_observer(&imu);
    imu.set_powered(false);
    clock.delay_ms(100);
    TEST_ASSERT_FALSE(imu.data_ready());
    ImuSample s;
    TEST_ASSERT_FALSE(imu.read(&s));
}

void test_powering_imu_on_primes_a_sample(void) {
    SimClock clock;
    FakeImu imu(clock, 10);
    imu.set_powered(false);
    imu.set_powered(true);
    TEST_ASSERT_TRUE(imu.data_ready());
}

void test_failing_imu_read_returns_false(void) {
    SimClock clock;
    FakeImu imu(clock, 10);
    imu.set_failing(true);
    ImuSample s;
    TEST_ASSERT_FALSE(imu.read(&s));
    TEST_ASSERT_EQUAL_UINT32(10, clock.now_ms());  // a failed transaction still costs time
}

void test_hung_imu_read_returns_when_the_breaker_trips(void) {
    SimClock clock;
    FakeImu imu(clock, 10);
    AtTime breaker(clock, 90050);
    imu.hang_next_read(&breaker);
    ImuSample s;
    TEST_ASSERT_FALSE(imu.read(&s));
    TEST_ASSERT_GREATER_OR_EQUAL_UINT32(90050, clock.now_ms());
    TEST_ASSERT_LESS_THAN_UINT32(90050 + FakeImu::HANG_STEP_MS, clock.now_ms());

    TEST_ASSERT_TRUE(imu.read(&s));  // one-shot
}

void test_imu_power_cycle_is_counted(void) {
    SimClock clock;
    FakeImu imu(clock, 10);
    imu.power_cycle();
    TEST_ASSERT_EQUAL_UINT(1, imu.power_cycles());
    TEST_ASSERT_TRUE(imu.powered());
}

void test_unpowered_radio_refuses(void) {
    SimClock clock;
    FakeRadio radio(clock, 92);
    radio.set_powered(false);
    const uint8_t payload[] = {1};
    TEST_ASSERT_FALSE(radio.begin_transmit(payload, 1));
    TEST_ASSERT_FALSE(radio.transmit(payload, 1));
}

void test_powering_radio_down_abandons_a_transmission(void) {
    SimClock clock;
    FakeRadio radio(clock, 92);
    clock.add_observer(&radio);
    const uint8_t payload[] = {1};
    TEST_ASSERT_TRUE(radio.begin_transmit(payload, 1));
    radio.set_powered(false);
    TEST_ASSERT_FALSE(radio.transmitting());
    clock.delay_ms(200);
    TEST_ASSERT_EQUAL_UINT(0, radio.sent_count());
}

void test_wedged_radio_never_completes_until_power_cycled(void) {
    SimClock clock;
    FakeRadio radio(clock, 92);
    clock.add_observer(&radio);
    radio.set_wedged(true);
    const uint8_t payload[] = {1};
    TEST_ASSERT_TRUE(radio.begin_transmit(payload, 1));
    clock.delay_ms(10000);
    TEST_ASSERT_TRUE(radio.tx_busy());

    radio.power_cycle();
    TEST_ASSERT_EQUAL_UINT(1, radio.power_cycles());
    TEST_ASSERT_FALSE(radio.tx_busy());
    TEST_ASSERT_TRUE(radio.begin_transmit(payload, 1));
    clock.delay_ms(92);
    TEST_ASSERT_EQUAL_UINT(1, radio.sent_count());
}

void test_refusing_radio_refuses_begin_transmit(void) {
    SimClock clock;
    FakeRadio radio(clock, 92);
    radio.set_refusing(true);
    const uint8_t payload[] = {1};
    TEST_ASSERT_FALSE(radio.begin_transmit(payload, 1));
}
```

Register them at the end of `main()`:

```cpp
    RUN_TEST(test_advance_to_delivers_one_tick);
    RUN_TEST(test_clock_accepts_eight_observers);
    RUN_TEST(test_default_gps_still_fixes_immediately);
    RUN_TEST(test_unpowered_gps_emits_nothing);
    RUN_TEST(test_gps_cold_start_sends_no_fix_until_ttff);
    RUN_TEST(test_gps_hot_start_after_a_short_backup);
    RUN_TEST(test_gps_cold_start_after_ephemeris_expires);
    RUN_TEST(test_sky_blocked_gps_never_fixes);
    RUN_TEST(test_injected_bytes_are_readable_while_unpowered);
    RUN_TEST(test_unpowered_imu_is_never_ready);
    RUN_TEST(test_powering_imu_on_primes_a_sample);
    RUN_TEST(test_failing_imu_read_returns_false);
    RUN_TEST(test_hung_imu_read_returns_when_the_breaker_trips);
    RUN_TEST(test_imu_power_cycle_is_counted);
    RUN_TEST(test_unpowered_radio_refuses);
    RUN_TEST(test_powering_radio_down_abandons_a_transmission);
    RUN_TEST(test_wedged_radio_never_completes_until_power_cycled);
    RUN_TEST(test_refusing_radio_refuses_begin_transmit);
```

- [ ] **Step 2: Run to verify they fail**

Run: `pio test -e native -f test_fakes`
Expected: compile failure on the new members.

- [ ] **Step 3: Implement**

`test/support/hang_breaker.hpp`:

```cpp
#ifndef FLOODNET_TEST_HANG_BREAKER_HPP
#define FLOODNET_TEST_HANG_BREAKER_HPP

namespace floodnet {

/// Ends a simulated hang. On hardware nothing ends a hung I2C read except a
/// reset; in simulation the hung call has to return eventually, and this says
/// when: the watchdog expiring, or the run ending for a build that has none.
class IHangBreaker {
  public:
    virtual ~IHangBreaker() {}
    virtual bool should_break() const = 0;
};

}  // namespace floodnet

#endif  // FLOODNET_TEST_HANG_BREAKER_HPP
```

`sim_clock.hpp`: change `MAX_OBSERVERS` to 8 and add, after `wait_for_event()`:

```cpp
    /// Moves straight to `t_ms` in one on_tick. Used for sleep, where 1 ms
    /// steps would make a 30-day run take billions of iterations. An observer
    /// that is powered down ignores the elapsed time; one that is not sees a
    /// single large tick. Does nothing if `t_ms` is not in the future.
    void advance_to(uint32_t t_ms) {
        const int32_t ahead = static_cast<int32_t>(t_ms - now_ms_);
        if (ahead > 0) {
            delay_ms(static_cast<uint32_t>(ahead));
        }
    }
```

`fake_gps.hpp`: replace the class body with the following. The emission loop must produce exactly the old byte sequence when the fake is left at its defaults, which is what keeps the milestone 1 and 2 benchmark rows identical:

```cpp
class FakeGps : public IGpsSource, public ISimTick {
  public:
    static const size_t DEFAULT_FIFO_DEPTH = 64;
    static const size_t MAX_FIFO_DEPTH = 4096;

    /// What a NEO-M8N sends with no fix: a well-formed GGA with fix quality 0.
    /// parse_gga() accepts it and reports valid == false.
    static constexpr const char *NO_FIX_SENTENCE = "$GPGGA,123519,,,,,0,00,,,M,,M,,*6B\r\n";

    /// NEO-M8 data sheet UBX-15031086, TTFF, GPS + GLONASS.
    static const uint32_t HOT_START_MS = 1000;
    static const uint32_t COLD_START_MS = 26000;

    /// How old the last fix may be for a hot start. The data sheet states no
    /// ephemeris lifetime; this is the milestone 3 spec's stated assumption.
    static const uint32_t EPHEMERIS_LIFETIME_MS = 2UL * 60UL * 60UL * 1000UL;

    FakeGps(const char *sentence, double bytes_per_ms,
            size_t fifo_depth = DEFAULT_FIFO_DEPTH)
        : sentence_(sentence),
          current_(sentence),
          current_len_(strlen(sentence)),
          source_index_(0),
          bytes_per_ms_(bytes_per_ms),
          pending_(0.0),
          fifo_depth_(fifo_depth),
          head_(0),
          tail_(0),
          count_(0),
          dropped_(0),
          powered_(true),
          sky_blocked_(false),
          ttff_ms_(0),
          since_power_on_ms_(0),
          has_fixed_(false),
          since_fix_ms_(0) {
        // Not assert(): this guards the fifo_ indexing below, and a guard
        // that NDEBUG can strip would turn a bad depth into memory corruption.
        if (fifo_depth == 0 || fifo_depth > MAX_FIFO_DEPTH) {
            fprintf(stderr, "FakeGps depth must be between 1 and MAX_FIFO_DEPTH\n");
            abort();
        }
    }

    void on_tick(uint32_t elapsed_ms) override {
        since_fix_ms_ = saturating_add(since_fix_ms_, elapsed_ms);
        if (!powered_) {
            return;  // in backup: sends nothing, but ephemeris keeps ageing
        }
        since_power_on_ms_ = saturating_add(since_power_on_ms_, elapsed_ms);

        pending_ += static_cast<double>(elapsed_ms) * bytes_per_ms_;
        while (pending_ >= 1.0) {
            // Chosen only at a sentence boundary, so a switch never splices
            // two sentences together.
            if (source_index_ == 0) {
                choose_sentence();
            }
            push(current_[source_index_]);
            source_index_ = (source_index_ + 1) % current_len_;
            if (source_index_ == 0 && current_ == sentence_) {
                has_fixed_ = true;
                since_fix_ms_ = 0;
            }
            pending_ -= 1.0;
        }
    }

    int read_byte() override {
        if (count_ == 0) {
            return -1;
        }
        const uint8_t value = fifo_[tail_];
        tail_ = (tail_ + 1) % fifo_depth_;
        --count_;
        return static_cast<int>(value);
    }

    uint16_t rx_overflows() const override { return dropped_; }

    bool pending() const override { return count_ > 0; }

    /// Off models software backup: nothing is sent, ephemeris is kept. On
    /// starts a hot start if the last fix is recent enough, a cold one if not.
    /// Only changes of state do anything.
    void set_powered(bool on) {
        if (on == powered_) {
            return;
        }
        powered_ = on;
        if (on) {
            const bool hot = has_fixed_ && since_fix_ms_ < EPHEMERIS_LIFETIME_MS;
            ttff_ms_ = hot ? HOT_START_MS : COLD_START_MS;
            since_power_on_ms_ = 0;
            pending_ = 0.0;
            source_index_ = 0;
        }
    }

    bool powered() const { return powered_; }

    /// No satellites visible: only no-fix sentences until unblocked.
    void set_sky_blocked(bool blocked) { sky_blocked_ = blocked; }

    /// Powered and not yet producing fixes: the acquisition current applies.
    bool acquiring() const {
        return powered_ && (sky_blocked_ || since_power_on_ms_ < ttff_ms_);
    }

    /// Places bytes in the receive buffer directly, as if left over from an
    /// earlier wake. Test hook.
    void inject(const char *bytes) {
        for (const char *p = bytes; *p != '\0'; ++p) {
            push(*p);
        }
    }

  private:
    static uint32_t saturating_add(uint32_t a, uint32_t b) {
        return a > UINT32_MAX - b ? UINT32_MAX : a + b;
    }

    void choose_sentence() {
        const bool fixing = !sky_blocked_ && since_power_on_ms_ >= ttff_ms_;
        current_ = fixing ? sentence_ : NO_FIX_SENTENCE;
        current_len_ = strlen(current_);
    }

    void push(char value) {
        if (count_ == fifo_depth_) {
            if (dropped_ < 0xFFFF) {
                ++dropped_;
            }
            return;
        }
        fifo_[head_] = static_cast<uint8_t>(value);
        head_ = (head_ + 1) % fifo_depth_;
        ++count_;
    }

    const char *sentence_;
    const char *current_;
    size_t current_len_;
    size_t source_index_;
    double bytes_per_ms_;
    double pending_;

    size_t fifo_depth_;
    uint8_t fifo_[MAX_FIFO_DEPTH];
    size_t head_;
    size_t tail_;
    size_t count_;
    uint16_t dropped_;

    bool powered_;
    bool sky_blocked_;
    uint32_t ttff_ms_;
    uint32_t since_power_on_ms_;
    bool has_fixed_;
    uint32_t since_fix_ms_;
};
```

Add `#include <stdint.h>` to `fake_gps.hpp` for `UINT32_MAX`. The class comment above it stays, with one sentence added: "It can also be powered down into backup, and models the NEO-M8N's time to first fix when powered back up."

`fake_imu.hpp`: add `#include "hang_breaker.hpp"`, and replace the class body with:

```cpp
class FakeImu : public IImuSource, public ISimTick {
  public:
    /// BNO055 NDOF fusion output is fixed at 100 Hz.
    static const uint32_t SAMPLE_PERIOD_MS = 10;

    /// How far a hung read advances the clock between breaker checks.
    static const uint32_t HANG_STEP_MS = 100;

    FakeImu(SimClock &clock, uint32_t read_cost_ms)
        : clock_(clock),
          read_cost_ms_(read_cost_ms),
          reads_(0),
          since_sample_ms_(0),
          ready_(true),
          powered_(true),
          failing_(false),
          hang_(nullptr),
          power_cycles_(0) {}

    void on_tick(uint32_t elapsed_ms) override {
        if (!powered_) {
            return;  // suspended: no fusion output
        }
        since_sample_ms_ += elapsed_ms;
        if (since_sample_ms_ >= SAMPLE_PERIOD_MS) {
            since_sample_ms_ = 0;
            ready_ = true;
        }
    }

    bool pending() const override { return ready_; }

    bool data_ready() const override { return ready_; }

    bool read(ImuSample *out) override {
        // Cleared before the transaction rather than after. A sample becoming
        // available during the read is a real event on hardware, and clearing
        // afterwards would swallow it.
        ready_ = false;

        if (hang_ != nullptr) {
            // A read that never returns, until something outside it (the
            // watchdog, or the end of the run) says stop.
            const IHangBreaker *breaker = hang_;
            hang_ = nullptr;
            while (!breaker->should_break()) {
                clock_.delay_ms(HANG_STEP_MS);
            }
            return false;
        }
        if (!powered_) {
            return false;
        }

        clock_.delay_ms(read_cost_ms_);
        ++reads_;
        if (failing_) {
            return false;
        }

        out->time_ms = clock_.now_ms();
        out->yaw_cd = 1500;
        out->pitch_cd = -200;
        out->roll_cd = 50;
        out->valid = true;
        return true;
    }

    size_t reads() const { return reads_; }

    /// Off models BNO055 suspend. On primes a sample, as TeensyImu::resume()
    /// does. Only changes of state do anything.
    void set_powered(bool on) {
        if (on == powered_) {
            return;
        }
        powered_ = on;
        ready_ = on;
        since_sample_ms_ = 0;
    }

    bool powered() const { return powered_; }

    /// Reads cost their time and then fail, as a NACKing bus would. A power
    /// cycle does not clear this: the fault is outside the sensor.
    void set_failing(bool failing) { failing_ = failing; }

    /// The next read hangs until `breaker` says stop. One-shot.
    void hang_next_read(const IHangBreaker *breaker) { hang_ = breaker; }

    void power_cycle() {
        set_powered(false);
        set_powered(true);
        ++power_cycles_;
    }

    size_t power_cycles() const { return power_cycles_; }

  private:
    SimClock &clock_;
    uint32_t read_cost_ms_;
    size_t reads_;
    uint32_t since_sample_ms_;
    bool ready_;
    bool powered_;
    bool failing_;
    const IHangBreaker *hang_;
    size_t power_cycles_;
};
```

`fake_radio.hpp`: add members `bool powered_; bool wedged_; bool refusing_; size_t power_cycles_;`, initialised to `true, false, false, 0` in the constructor's initialiser list after `lose_next_completion_(false)`. Then change and add methods:

```cpp
    // In transmit(), before the existing busy_ check:
        if (!powered_) {
            return false;
        }

    // begin_transmit() becomes:
    bool begin_transmit(const uint8_t *data, size_t len) override {
        if (!powered_ || refusing_ || busy_ || len > sizeof(last_payload_)) {
            return false;
        }
        memcpy(last_payload_, data, len);
        last_length_ = len;
        busy_ = true;
        // Wedged: accepted, sent to the modem, never reported finished.
        remaining_ms_ = wedged_ ? UINT32_MAX : tx_cost_ms_;
        return true;
    }

    /// Off models RH_RF95::sleep(): a transmission in progress is abandoned.
    void set_powered(bool on) {
        powered_ = on;
        if (!on) {
            busy_ = false;
            remaining_ms_ = 0;
        }
    }

    bool powered() const { return powered_; }

    /// Every transmission from now on hangs, until power_cycle().
    void set_wedged(bool wedged) { wedged_ = wedged; }

    /// begin_transmit() refuses outright while set.
    void set_refusing(bool refusing) { refusing_ = refusing; }

    /// Models a reset-pin pulse and re-init: clears a wedge, leaves it on.
    void power_cycle() {
        busy_ = false;
        remaining_ms_ = 0;
        wedged_ = false;
        powered_ = true;
        ++power_cycles_;
    }

    size_t power_cycles() const { return power_cycles_; }

    /// Const view of tx_busy() for energy accounting.
    bool transmitting() const { return busy_; }
```

- [ ] **Step 4: Run the fake tests**

Run: `pio test -e native -f test_fakes`
Expected: all tests PASS, old and new.

- [ ] **Step 5: Prove the published benchmark rows are unchanged**

Run: `diff <(grep -o '^BENCH,[^ ]*' README.md) <(pio test -e native -f test_benchmark -v 2>&1 | grep -o 'BENCH,[^ ]*')`
Expected: no output (identical). If anything differs, stop: the fakes changed a measured baseline. Do not edit the README to match.

- [ ] **Step 6: Run the whole suite**

Run: `pio test -e native`
Expected: every suite PASSES.

- [ ] **Step 7: Commit**

```bash
git add test/support/ test/test_fakes/test_main.cpp
git commit -m "test: power the fakes down, model GPS time to first fix, and add fault hooks"
```

---

### Task 7: Power, watchdog and persistent-store HAL, with their fakes

**Files:**
- Create: `lib/floodnet_hal/include/floodnet/hal/power.hpp`
- Create: `lib/floodnet_hal/include/floodnet/hal/watchdog.hpp`
- Create: `lib/floodnet_hal/include/floodnet/hal/persistent_store.hpp`
- Create: `test/support/current_model.hpp`
- Create: `test/support/fake_power.hpp`
- Create: `test/support/fake_watchdog.hpp`
- Create: `test/support/fake_persistent_store.hpp`
- Test: `test/test_fakes/test_main.cpp`

**Interfaces:**
- Consumes: Task 6's `FakeGps::acquiring()/powered()/set_powered()`, `FakeImu::powered()/set_powered()/power_cycle()`, `FakeRadio::powered()/transmitting()/set_powered()/power_cycle()`, `SimClock::advance_to()`, `IHangBreaker`.
- Produces:
  - `enum class Peripheral : uint8_t { Gps, Imu, Radio }; enum class PowerState : uint8_t { On, Low };`
  - `class IPower { set_power(Peripheral, PowerState); bool power_cycle(Peripheral); void sleep_until(uint32_t); uint16_t battery_mv(); };`
  - `class IWatchdog { void begin(uint32_t timeout_ms); void kick(); };`
  - `const size_t PERSISTENT_SLOTS = 4; class IPersistentStore { uint32_t read_u32(size_t); void write_u32(size_t, uint32_t); };`
  - `struct CurrentModel` (nA fields, `with_teensy_sleep()`), `TEENSY_SLEEP_LOW_NA`, `TEENSY_SLEEP_HIGH_NA`, `NCR18650B_CAPACITY_NA_MS`
  - `FakePower(SimClock&, FakeGps&, FakeImu&, FakeRadio&, const CurrentModel&, uint64_t capacity_nA_ms)` with `current_nA()`, `depleted()`, `depleted_at_ms()`, `consumed_nA_ms()`, `sleeping()`, `sleeps()`; capacity 0 means unlimited.
  - `FakeWatchdog` (also `ISimTick`, `IHangBreaker`) with `expired()` (latched until `on_mcu_reset()`), `on_mcu_reset()`, `max_since_kick_ms()`, `kicks()`.
  - `FakePersistentStore`.

- [ ] **Step 1: Write the failing tests**

Add to `test/test_fakes/test_main.cpp` includes: `#include "../support/current_model.hpp"`, `#include "../support/fake_power.hpp"`, `#include "../support/fake_watchdog.hpp"`, `#include "../support/fake_persistent_store.hpp"`. Then:

```cpp
namespace {

/// Everything FakePower needs, registered with FakePower first so each tick
/// is charged at the state that held when it began.
struct PowerBench {
    SimClock clock;
    FakeGps gps;
    FakeImu imu;
    FakeRadio radio;
    CurrentModel model;
    FakePower power;

    explicit PowerBench(uint64_t capacity_nA_ms)
        : clock(),
          gps(kFix, 0.96, FakeGps::MAX_FIFO_DEPTH),
          imu(clock, 10),
          radio(clock, 92),
          model(),
          power(clock, gps, imu, radio, model, capacity_nA_ms) {
        clock.add_observer(&power);
        clock.add_observer(&gps);
        clock.add_observer(&imu);
        clock.add_observer(&radio);
    }

    void all_low() {
        power.set_power(Peripheral::Gps, PowerState::Low);
        power.set_power(Peripheral::Imu, PowerState::Low);
        power.set_power(Peripheral::Radio, PowerState::Low);
    }
};

// 6 mA + 30 uA + 40 uA + 0.2 uA, in nA.
const uint64_t kAllLowSleepingNa = 6000000ULL + 30000ULL + 40000ULL + 200ULL;

}  // namespace

void test_awake_with_everything_on_draws_the_table_sum(void) {
    PowerBench b(0);
    // Teensy awake + GPS tracking + BNO055 normal + RFM95W standby.
    TEST_ASSERT_EQUAL_UINT64(100000000ULL + 23000000ULL + 12300000ULL + 1600000ULL,
                             b.power.current_nA());
}

void test_transmitting_replaces_standby_with_transmit_current(void) {
    PowerBench b(0);
    const uint8_t payload[] = {1};
    b.radio.begin_transmit(payload, 1);
    TEST_ASSERT_EQUAL_UINT64(100000000ULL + 23000000ULL + 12300000ULL + 120000000ULL,
                             b.power.current_nA());
}

void test_acquiring_gps_draws_acquisition_current(void) {
    PowerBench b(0);
    b.gps.set_powered(false);
    b.gps.set_powered(true);
    TEST_ASSERT_EQUAL_UINT64(100000000ULL + 25000000ULL + 12300000ULL + 1600000ULL,
                             b.power.current_nA());
}

void test_sleep_charges_sleep_currents_for_the_whole_jump(void) {
    PowerBench b(0);
    b.all_low();
    b.power.sleep_until(60000);
    TEST_ASSERT_EQUAL_UINT32(60000, b.clock.now_ms());
    TEST_ASSERT_EQUAL_UINT64(kAllLowSleepingNa * 60000ULL, b.power.consumed_nA_ms());
    TEST_ASSERT_FALSE(b.power.sleeping());
    TEST_ASSERT_EQUAL_UINT(1, b.power.sleeps());
}

void test_sleep_until_the_past_returns_at_once(void) {
    PowerBench b(0);
    b.clock.delay_ms(100);
    b.power.sleep_until(50);
    TEST_ASSERT_EQUAL_UINT32(100, b.clock.now_ms());
    TEST_ASSERT_EQUAL_UINT(0, b.power.sleeps());
}

void test_depletion_time_is_exact_inside_a_long_tick(void) {
    PowerBench b(kAllLowSleepingNa * 30000ULL);
    b.all_low();
    b.power.sleep_until(60000);
    TEST_ASSERT_TRUE(b.power.depleted());
    TEST_ASSERT_EQUAL_UINT32(30000, b.power.depleted_at_ms());  // not 60000
}

void test_battery_mv_falls_linearly_as_a_placeholder(void) {
    PowerBench b(kAllLowSleepingNa * 120000ULL);
    TEST_ASSERT_EQUAL_UINT16(4200, b.power.battery_mv());
    b.all_low();
    b.power.sleep_until(60000);
    TEST_ASSERT_EQUAL_UINT16(3600, b.power.battery_mv());
}

void test_unlimited_battery_never_depletes(void) {
    PowerBench b(0);
    b.clock.delay_ms(1000000);
    TEST_ASSERT_FALSE(b.power.depleted());
    TEST_ASSERT_EQUAL_UINT16(4200, b.power.battery_mv());
}

void test_power_cycle_radio_clears_a_wedge(void) {
    PowerBench b(0);
    b.radio.set_wedged(true);
    const uint8_t payload[] = {1};
    b.radio.begin_transmit(payload, 1);
    TEST_ASSERT_TRUE(b.power.power_cycle(Peripheral::Radio));
    TEST_ASSERT_FALSE(b.radio.tx_busy());
    TEST_ASSERT_EQUAL_UINT(1, b.radio.power_cycles());
}

void test_watchdog_expires_only_after_its_timeout(void) {
    SimClock clock;
    FakeWatchdog wd;
    clock.add_observer(&wd);
    wd.begin(90000);
    clock.delay_ms(90000);
    TEST_ASSERT_FALSE(wd.expired());
    clock.delay_ms(1);
    TEST_ASSERT_TRUE(wd.expired());
    TEST_ASSERT_TRUE(wd.should_break());
    // Latched: on hardware the reset has already happened, so a kick from
    // code still running in the simulation cannot undo it.
    wd.kick();
    TEST_ASSERT_TRUE(wd.expired());
    TEST_ASSERT_EQUAL_UINT32(90001, wd.max_since_kick_ms());
    wd.on_mcu_reset();
    TEST_ASSERT_FALSE(wd.expired());
}

void test_unarmed_watchdog_never_expires(void) {
    SimClock clock;
    FakeWatchdog wd;
    clock.add_observer(&wd);
    clock.delay_ms(1000000);
    TEST_ASSERT_FALSE(wd.expired());

    wd.begin(1000);
    clock.delay_ms(2000);
    wd.on_mcu_reset();  // a reset leaves WDOG1 disabled until begin()
    TEST_ASSERT_FALSE(wd.expired());
}

void test_persistent_store_keeps_values_and_bounds_slots(void) {
    FakePersistentStore store;
    store.write_u32(0, 41);
    store.write_u32(PERSISTENT_SLOTS, 99);  // out of range: ignored
    TEST_ASSERT_EQUAL_UINT32(41, store.read_u32(0));
    TEST_ASSERT_EQUAL_UINT32(0, store.read_u32(PERSISTENT_SLOTS));
}
```

Register them at the end of `main()`, in order.

- [ ] **Step 2: Run to verify they fail**

Run: `pio test -e native -f test_fakes`
Expected: compile failure, new headers not found.

- [ ] **Step 3: Implement the HAL interfaces**

`lib/floodnet_hal/include/floodnet/hal/power.hpp`:

```cpp
#ifndef FLOODNET_HAL_POWER_HPP
#define FLOODNET_HAL_POWER_HPP

#include <stdint.h>

namespace floodnet {

enum class Peripheral : uint8_t { Gps, Imu, Radio };

enum class PowerState : uint8_t { On, Low };

class IPower {
  public:
    virtual ~IPower() {}

    /// `Low` is each part's own low-power command, not a load switch: backup
    /// for the GPS (keeps ephemeris, so the next wake is a hot start), suspend
    /// for the IMU, sleep for the radio.
    virtual void set_power(Peripheral p, PowerState s) = 0;

    /// Low, then On, then re-initialise the driver: for a part that has
    /// stopped answering. Re-initialisation is driver-specific, which is why
    /// it lives here. False if re-initialisation failed.
    virtual bool power_cycle(Peripheral p) = 0;

    /// Puts the MCU in its lowest RAM-retaining mode until `wake_ms` on the
    /// IClock timeline. Returns at once if `wake_ms` has passed (wrap-safe).
    /// May return early; callers re-check the time.
    virtual void sleep_until(uint32_t wake_ms) = 0;

    virtual uint16_t battery_mv() = 0;
};

}  // namespace floodnet

#endif  // FLOODNET_HAL_POWER_HPP
```

`watchdog.hpp`:

```cpp
#ifndef FLOODNET_HAL_WATCHDOG_HPP
#define FLOODNET_HAL_WATCHDOG_HPP

#include <stdint.h>

namespace floodnet {

/// A hardware timer that resets the MCU unless kicked within its timeout.
class IWatchdog {
  public:
    virtual ~IWatchdog() {}
    virtual void begin(uint32_t timeout_ms) = 0;
    virtual void kick() = 0;
};

}  // namespace floodnet

#endif  // FLOODNET_HAL_WATCHDOG_HPP
```

`persistent_store.hpp`:

```cpp
#ifndef FLOODNET_HAL_PERSISTENT_STORE_HPP
#define FLOODNET_HAL_PERSISTENT_STORE_HPP

#include <stddef.h>
#include <stdint.h>

namespace floodnet {

/// The i.MX RT1062 has four SNVS low-power general purpose registers.
const size_t PERSISTENT_SLOTS = 4;

/// Words that survive a watchdog reset. Not flash: nothing here wears out.
class IPersistentStore {
  public:
    virtual ~IPersistentStore() {}
    /// 0 for a slot never written or out of range.
    virtual uint32_t read_u32(size_t slot) = 0;
    /// Ignored for a slot out of range.
    virtual void write_u32(size_t slot, uint32_t value) = 0;
};

}  // namespace floodnet

#endif  // FLOODNET_HAL_PERSISTENT_STORE_HPP
```

- [ ] **Step 4: Implement the fakes**

`test/support/current_model.hpp`:

```cpp
#ifndef FLOODNET_TEST_CURRENT_MODEL_HPP
#define FLOODNET_TEST_CURRENT_MODEL_HPP

#include <stdint.h>

namespace floodnet {

/// Supply currents, in nanoamps so the RFM95W's 0.2 uA sleep current is
/// representable. Taken at face value from the battery: regulator efficiency
/// and quiescent current are not modelled (milestone 3 design doc, "Energy
/// accounting"). Every value is cited; none may be changed to move a result.
struct CurrentModel {
    /// PJRC Teensy 4.1 product page, "approximately 100 mA" at 600 MHz (the
    /// sentence names the Teensy 4.0: same MCU, same clock).
    uint32_t teensy_awake_nA = 100000000;
    /// PJRC forum, "Teensy 4.1 deep sleep and watchdog": 6 mA from 5 V in
    /// Snooze deepSleep. A user measurement, not a data sheet value.
    uint32_t teensy_sleep_nA = 6000000;
    /// NEO-M8 data sheet UBX-15031086, Table 11, NEO-M8N, GPS, at 3 V.
    uint32_t gps_acquisition_nA = 25000000;
    uint32_t gps_tracking_nA = 23000000;  ///< same table, continuous mode
    uint32_t gps_backup_nA = 30000;       ///< same data sheet, I_SWBCKP at VCC = 3 V
    /// BNO055 data sheet BST-BNO055-DS000: normal mode, and suspend mode.
    uint32_t imu_normal_nA = 12300000;
    uint32_t imu_suspend_nA = 40000;
    /// RFM95/96/97/98W data sheet, Table 51.
    uint32_t radio_tx_nA = 120000000;    ///< +20 dBm on PA_BOOST
    uint32_t radio_standby_nA = 1600000; ///< IDDST
    uint32_t radio_sleep_nA = 200;       ///< IDDSL

    static CurrentModel with_teensy_sleep(uint32_t sleep_nA) {
        CurrentModel model;
        model.teensy_sleep_nA = sleep_nA;
        return model;
    }
};

/// The two measured Snooze deepSleep currents; the battery-life result is
/// reported at both because neither is a data sheet value.
const uint32_t TEENSY_SLEEP_LOW_NA = 6000000;    ///< PJRC forum, see above
/// PJRC forum, "Teensy 4.1 using Snooze library with deepSleep": 25.86 mA.
const uint32_t TEENSY_SLEEP_HIGH_NA = 25860000;

/// Panasonic NCR18650B data sheet: rated capacity 3200 mAh (minimum).
const uint64_t NCR18650B_CAPACITY_NA_MS = 3200ULL * 1000000ULL * 3600000ULL;

}  // namespace floodnet

#endif  // FLOODNET_TEST_CURRENT_MODEL_HPP
```

`test/support/fake_power.hpp`:

```cpp
#ifndef FLOODNET_TEST_FAKE_POWER_HPP
#define FLOODNET_TEST_FAKE_POWER_HPP

#include <stdint.h>

#include <floodnet/hal/power.hpp>

#include "current_model.hpp"
#include "fake_gps.hpp"
#include "fake_imu.hpp"
#include "fake_radio.hpp"
#include "sim_clock.hpp"

namespace floodnet {

/// Drives the fakes' power states and integrates the current they draw into a
/// battery. Register it as a clock observer before the fakes, so each tick is
/// charged at the state that held when the tick began.
class FakePower : public IPower, public ISimTick {
  public:
    /// Telemetry-only placeholder: a straight line, not a discharge model.
    /// No reported result depends on it.
    static const uint16_t FULL_MV = 4200;
    static const uint16_t EMPTY_MV = 3000;

    /// `capacity_nA_ms` of 0 means unlimited.
    FakePower(SimClock &clock, FakeGps &gps, FakeImu &imu, FakeRadio &radio,
              const CurrentModel &model, uint64_t capacity_nA_ms)
        : clock_(clock),
          gps_(gps),
          imu_(imu),
          radio_(radio),
          model_(model),
          capacity_(capacity_nA_ms),
          consumed_(0),
          depleted_(false),
          depleted_at_ms_(0),
          sleeping_(false),
          sleeps_(0) {}

    void set_power(Peripheral p, PowerState s) override {
        const bool on = s == PowerState::On;
        switch (p) {
        case Peripheral::Gps:
            gps_.set_powered(on);
            break;
        case Peripheral::Imu:
            imu_.set_powered(on);
            break;
        case Peripheral::Radio:
            radio_.set_powered(on);
            break;
        }
    }

    bool power_cycle(Peripheral p) override {
        switch (p) {
        case Peripheral::Gps:
            gps_.set_powered(false);
            gps_.set_powered(true);
            break;
        case Peripheral::Imu:
            imu_.power_cycle();
            break;
        case Peripheral::Radio:
            radio_.power_cycle();
            break;
        }
        return true;
    }

    void sleep_until(uint32_t wake_ms) override {
        if (static_cast<int32_t>(wake_ms - clock_.now_ms()) <= 0) {
            return;
        }
        sleeping_ = true;
        clock_.advance_to(wake_ms);
        sleeping_ = false;
        ++sleeps_;
    }

    uint16_t battery_mv() override {
        if (capacity_ == 0) {
            return FULL_MV;
        }
        const uint64_t left = consumed_ >= capacity_ ? 0 : capacity_ - consumed_;
        return static_cast<uint16_t>(EMPTY_MV + (FULL_MV - EMPTY_MV) * left / capacity_);
    }

    void on_tick(uint32_t elapsed_ms) override {
        const uint64_t current = current_nA();
        const uint64_t before = consumed_;
        consumed_ += current * elapsed_ms;
        if (capacity_ != 0 && !depleted_ && consumed_ >= capacity_) {
            // Exact, not rounded to the end of the tick: a 60 s sleep jump
            // must not move the death time by up to a minute.
            depleted_ = true;
            const uint32_t tick_start = clock_.now_ms() - elapsed_ms;
            depleted_at_ms_ =
                tick_start + static_cast<uint32_t>((capacity_ - before + current - 1) / current);
        }
    }

    bool pending() const override { return false; }

    uint64_t current_nA() const {
        uint64_t total = sleeping_ ? model_.teensy_sleep_nA : model_.teensy_awake_nA;
        if (!gps_.powered()) {
            total += model_.gps_backup_nA;
        } else {
            total += gps_.acquiring() ? model_.gps_acquisition_nA : model_.gps_tracking_nA;
        }
        total += imu_.powered() ? model_.imu_normal_nA : model_.imu_suspend_nA;
        if (!radio_.powered()) {
            total += model_.radio_sleep_nA;
        } else {
            total += radio_.transmitting() ? model_.radio_tx_nA : model_.radio_standby_nA;
        }
        return total;
    }

    bool depleted() const { return depleted_; }
    uint32_t depleted_at_ms() const { return depleted_at_ms_; }
    uint64_t consumed_nA_ms() const { return consumed_; }
    bool sleeping() const { return sleeping_; }
    size_t sleeps() const { return sleeps_; }

  private:
    SimClock &clock_;
    FakeGps &gps_;
    FakeImu &imu_;
    FakeRadio &radio_;
    CurrentModel model_;
    uint64_t capacity_;
    uint64_t consumed_;
    bool depleted_;
    uint32_t depleted_at_ms_;
    bool sleeping_;
    size_t sleeps_;
};

}  // namespace floodnet

#endif  // FLOODNET_TEST_FAKE_POWER_HPP
```

`test/support/fake_watchdog.hpp`:

```cpp
#ifndef FLOODNET_TEST_FAKE_WATCHDOG_HPP
#define FLOODNET_TEST_FAKE_WATCHDOG_HPP

#include <stddef.h>
#include <stdint.h>

#include <floodnet/hal/watchdog.hpp>

#include "hang_breaker.hpp"
#include "sim_clock.hpp"

namespace floodnet {

/// Counts time since the last kick. Expiry is reported, not acted on: the
/// harness that owns the node sees expired() and rebuilds it, which is what a
/// reset does to RAM on hardware.
///
/// Expiry latches until on_mcu_reset(). On hardware the reset is immediate;
/// in simulation the node's code keeps running until the harness looks, and
/// a kick from that code must not cancel a reset that has already happened.
class FakeWatchdog : public IWatchdog, public ISimTick, public IHangBreaker {
  public:
    FakeWatchdog()
        : armed_(false),
          fired_(false),
          timeout_ms_(0),
          since_kick_ms_(0),
          max_since_kick_ms_(0),
          kicks_(0) {}

    void begin(uint32_t timeout_ms) override {
        armed_ = true;
        fired_ = false;
        timeout_ms_ = timeout_ms;
        since_kick_ms_ = 0;
    }

    void kick() override {
        since_kick_ms_ = 0;
        ++kicks_;
    }

    void on_tick(uint32_t elapsed_ms) override {
        if (!armed_) {
            return;
        }
        since_kick_ms_ = since_kick_ms_ > UINT32_MAX - elapsed_ms ? UINT32_MAX
                                                                   : since_kick_ms_ + elapsed_ms;
        if (since_kick_ms_ > max_since_kick_ms_) {
            max_since_kick_ms_ = since_kick_ms_;
        }
        if (since_kick_ms_ > timeout_ms_) {
            fired_ = true;
        }
    }

    bool pending() const override { return false; }

    bool expired() const { return fired_; }

    bool should_break() const override { return expired(); }

    /// WDOG1 is disabled after a reset until begin() arms it again.
    void on_mcu_reset() {
        armed_ = false;
        fired_ = false;
        since_kick_ms_ = 0;
    }

    uint32_t max_since_kick_ms() const { return max_since_kick_ms_; }
    size_t kicks() const { return kicks_; }

  private:
    bool armed_;
    bool fired_;
    uint32_t timeout_ms_;
    uint32_t since_kick_ms_;
    uint32_t max_since_kick_ms_;
    size_t kicks_;
};

}  // namespace floodnet

#endif  // FLOODNET_TEST_FAKE_WATCHDOG_HPP
```

`test/support/fake_persistent_store.hpp`:

```cpp
#ifndef FLOODNET_TEST_FAKE_PERSISTENT_STORE_HPP
#define FLOODNET_TEST_FAKE_PERSISTENT_STORE_HPP

#include <floodnet/hal/persistent_store.hpp>

namespace floodnet {

/// Outlives the node it serves, as the SNVS registers outlive a reset.
class FakePersistentStore : public IPersistentStore {
  public:
    FakePersistentStore() : slots_() {}

    uint32_t read_u32(size_t slot) override { return slot < PERSISTENT_SLOTS ? slots_[slot] : 0; }

    void write_u32(size_t slot, uint32_t value) override {
        if (slot < PERSISTENT_SLOTS) {
            slots_[slot] = value;
        }
    }

  private:
    uint32_t slots_[PERSISTENT_SLOTS];
};

}  // namespace floodnet

#endif  // FLOODNET_TEST_FAKE_PERSISTENT_STORE_HPP
```

- [ ] **Step 5: Run the tests**

Run: `pio test -e native -f test_fakes`
Expected: all PASS.

- [ ] **Step 6: Check the HAL seam and the whole suite**

Run: `grep -rlE '#include[[:space:]]*<(Arduino|Wire|SPI)\.h>' lib/ ; pio test -e native`
Expected: grep prints nothing; every suite PASSES.

- [ ] **Step 7: Commit**

```bash
git add lib/floodnet_hal/include/floodnet/hal/ test/support/ test/test_fakes/test_main.cpp
git commit -m "feat: add power, watchdog and persistent-store interfaces with energy-accounting fakes"
```

---

### Task 8: Additive `InterruptSampler` members

**Files:**
- Modify: `src/sampler_interrupt.hpp`
- Modify: `src/sampler_interrupt.cpp`
- Test: `test/test_interrupt/test_main.cpp`

**Interfaces:**
- Consumes: `WireVersion`, `encode_packet_as()`, `Packet::boot_count/tx_timeouts/battery_mv` (Task 1); `FakeGps::set_powered`, `FakeImu::set_powered` (Task 6).
- Produces: constructor `InterruptSampler(IGpsSource&, IImuSource&, IAsyncRadio&, IClock&, uint16_t node_id, uint8_t ttl, WireVersion version = WireVersion::V1)`, `void set_node_status(uint16_t boot_count, uint16_t battery_mv)`, `void enqueue_heartbeat()`, `bool last_record_imu_valid() const`.

- [ ] **Step 1: Write the failing tests**

Add to `test/test_interrupt/test_main.cpp`:

```cpp
/// A v0x02 sampler on the CONTROL profile, built the way the Rig builds one.
struct V2Rig {
    SimClock clock;
    FakeGps gps;
    FakeImu imu;
    FakeRadio radio;
    InterruptSampler sampler;

    V2Rig()
        : clock(),
          gps(kSentence, kGpsByteRate, FakeGps::MAX_FIFO_DEPTH),
          imu(clock, kImuControlMs),
          radio(clock, kRadioControlMs),
          sampler(gps, imu, radio, clock, 0x0042, 3, WireVersion::V2) {
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

void test_default_wire_version_is_still_v1(void) {
    Rig rig(kImuControlMs, kRadioControlMs, FakeGps::MAX_FIFO_DEPTH);
    rig.run_for(2000);
    TEST_ASSERT_EQUAL_HEX8(0x01, rig.radio.last_payload()[1]);
}

void test_v2_sampler_stamps_node_status_at_send_time(void) {
    V2Rig rig;
    rig.sampler.set_node_status(7, 3700);
    rig.run_for(2000);

    TEST_ASSERT_EQUAL_HEX8(0x02, rig.radio.last_payload()[1]);
    Packet decoded;
    TEST_ASSERT_TRUE(decode_packet(rig.radio.last_payload(), rig.radio.last_length(), &decoded));
    TEST_ASSERT_EQUAL_UINT16(7, decoded.boot_count);
    TEST_ASSERT_EQUAL_UINT16(3700, decoded.battery_mv);
    TEST_ASSERT_EQUAL_UINT16(0, decoded.tx_timeouts);
}

void test_heartbeat_takes_a_sequence_number_and_reports_no_fix(void) {
    V2Rig rig;
    rig.gps.set_powered(false);
    rig.sampler.enqueue_heartbeat();
    TEST_ASSERT_EQUAL_UINT32(1, rig.sampler.next_seq());

    rig.run_for(100);
    TEST_ASSERT_EQUAL_UINT32(1, rig.sampler.packets_sent());
    Packet decoded;
    TEST_ASSERT_TRUE(decode_packet(rig.radio.last_payload(), rig.radio.last_length(), &decoded));
    TEST_ASSERT_EQUAL_UINT32(0, decoded.seq);
    TEST_ASSERT_FALSE(decoded.record.gps.valid);
}

void test_last_record_imu_valid_tracks_pairing(void) {
    V2Rig with_imu;
    with_imu.run_for(200);
    TEST_ASSERT_GREATER_THAN_UINT32(0, with_imu.sampler.next_seq());
    TEST_ASSERT_TRUE(with_imu.sampler.last_record_imu_valid());

    V2Rig without_imu;
    without_imu.imu.set_powered(false);
    without_imu.run_for(200);
    TEST_ASSERT_GREATER_THAN_UINT32(0, without_imu.sampler.next_seq());
    TEST_ASSERT_FALSE(without_imu.sampler.last_record_imu_valid());
}
```

Register the four at the end of `main()`.

- [ ] **Step 2: Run to verify they fail**

Run: `pio test -e native -f test_interrupt`
Expected: compile failure on the new constructor argument and members.

- [ ] **Step 3: Implement**

`sampler_interrupt.hpp`: change the constructor declaration and add the members:

```cpp
    /// `version` selects the wire format. v0x01 is the default, so the
    /// node_interrupt build and every milestone 2 figure are unaffected.
    InterruptSampler(IGpsSource &gps, IImuSource &imu, IAsyncRadio &radio, IClock &clock,
                     uint16_t node_id, uint8_t ttl, WireVersion version = WireVersion::V1);

    /// Node-level facts stamped into each packet when it is sent. Only v0x02
    /// carries them.
    void set_node_status(uint16_t boot_count, uint16_t battery_mv);

    /// Queues a packet with gps.valid == false, stamped now: "alive, no fix".
    /// It takes a sequence number like any other packet.
    void enqueue_heartbeat();

    /// Whether the most recently queued record carried a paired IMU sample.
    /// False before any record is queued.
    bool last_record_imu_valid() const { return last_record_imu_valid_; }
```

and in the private section, after `DiagCounters diag_;`:

```cpp
    WireVersion version_;
    uint16_t boot_count_;
    uint16_t battery_mv_;
    bool last_record_imu_valid_;
```

`sampler_interrupt.cpp`: extend the constructor's parameter list with `WireVersion version` and its initialiser list, after `diag_()`, with `version_(version), boot_count_(0), battery_mv_(0), last_record_imu_valid_(false)`. Then:

```cpp
void InterruptSampler::set_node_status(uint16_t boot_count, uint16_t battery_mv) {
    boot_count_ = boot_count;
    battery_mv_ = battery_mv;
}

void InterruptSampler::enqueue_heartbeat() {
    GpsFix fix;
    fix.time_ms = clock_.now_ms();
    fix.valid = false;
    enqueue(fix);
}
```

In `enqueue()`, after `packet.record = pairer_.pair(fix, diag_);`:

```cpp
    last_record_imu_valid_ = packet.record.imu.valid;
```

In `service_radio()`, replace the `encode_packet(...)` call:

```cpp
    // Stamped at send time rather than queue time, so a packet carries the
    // freshest counters. The v0x01 encoder ignores all three.
    packet.boot_count = boot_count_;
    packet.tx_timeouts = tx_timeouts_;
    packet.battery_mv = battery_mv_;

    uint8_t buffer[PACKET_SIZE];
    if (encode_packet_as(version_, packet, buffer, sizeof(buffer)) != PACKET_SIZE) {
```

- [ ] **Step 4: Run the sampler tests**

Run: `pio test -e native -f test_interrupt`
Expected: all PASS.

- [ ] **Step 5: Prove the published benchmark rows are unchanged**

Run: `diff <(grep -o '^BENCH,[^ ]*' README.md) <(pio test -e native -f test_benchmark -v 2>&1 | grep -o 'BENCH,[^ ]*')`
Expected: no output. If anything differs, stop and report.

- [ ] **Step 6: Build the milestone 2 firmware**

Run: `pio run -e node_interrupt -e node_polling`
Expected: both build with no warnings.

- [ ] **Step 7: Commit**

```bash
git add src/sampler_interrupt.hpp src/sampler_interrupt.cpp test/test_interrupt/test_main.cpp
git commit -m "feat: let the interrupt sampler send v0x02 status and heartbeat packets"
```

---

### Task 9: `DutyCycledNode` and the `PowerRig` harness

**Files:**
- Create: `src/duty_cycled_node.hpp`
- Create: `src/duty_cycled_node.cpp`
- Create: `test/support/power_rig.hpp`
- Modify: `platformio.ini` (native `build_src_filter`; node and gateway envs exclude the new file)
- Test: `test/test_duty_cycled/test_main.cpp`

**Interfaces:**
- Consumes: everything from Tasks 1-8.
- Produces:
  - `struct DutyCycledNodeConfig { uint16_t node_id = 1; uint8_t ttl = 3; NodeTiming timing = NodeTiming(); uint32_t watchdog_timeout_ms = 90000; };`
  - `class DutyCycledNode { DutyCycledNode(IGpsSource&, IImuSource&, IAsyncRadio&, IClock&, IPower&, IWatchdog&, IPersistentStore&, const DutyCycledNodeConfig&); void begin(); void step(); NodeState state() const; uint16_t boot_count() const; const InterruptSampler &sampler() const; static const size_t BOOT_COUNT_SLOT = 0; };`
  - `enum class Build { Interrupt, DutyCycled }; enum class FaultKind { LostCompletion, SkyBlockage, ImuFailing, RadioWedge, Hang };`
  - `class PowerRig { PowerRig(Build, uint32_t teensy_sleep_nA, uint64_t capacity_nA_ms); void add_fault(FaultKind, uint32_t start_ms, uint32_t end_ms); void run_until(uint32_t end_ms); DowntimeReport downtime(uint32_t scenario_ms) const; ... }` with accessors listed in the code below.

- [ ] **Step 1: Add the new source file to the native build and exclude it from the others**

In `platformio.ini`:

```ini
[env:node_polling]
extends = teensy_base
build_src_filter = +<*> -<gateway_main.cpp> -<sampler_interrupt.cpp> -<duty_cycled_node.cpp>
```

```ini
[env:node_interrupt]
extends = teensy_base
build_src_filter = +<*> -<gateway_main.cpp> -<sampler_polling.cpp> -<duty_cycled_node.cpp>
```

```ini
[env:gateway]
extends = teensy_base
build_src_filter = +<*> -<main.cpp> -<sampler_polling.cpp> -<sampler_interrupt.cpp> -<duty_cycled_node.cpp>
```

```ini
[env:native]
...
build_src_filter = +<sampler_polling.cpp> +<sampler_interrupt.cpp> +<duty_cycled_node.cpp>
```

- [ ] **Step 2: Write the harness**

`test/support/power_rig.hpp`:

```cpp
#ifndef FLOODNET_TEST_POWER_RIG_HPP
#define FLOODNET_TEST_POWER_RIG_HPP

#include <stddef.h>
#include <stdint.h>

#include <memory>
#include <vector>

#include <floodnet/downtime.hpp>
#include <floodnet/mesh.hpp>
#include <floodnet/packet.hpp>

#include "current_model.hpp"
#include "duty_cycled_node.hpp"
#include "fake_gps.hpp"
#include "fake_imu.hpp"
#include "fake_persistent_store.hpp"
#include "fake_power.hpp"
#include "fake_radio.hpp"
#include "fake_watchdog.hpp"
#include "hang_breaker.hpp"
#include "sampler_interrupt.hpp"
#include "sim_clock.hpp"

namespace floodnet {

enum class Build { Interrupt, DutyCycled };

enum class FaultKind { LostCompletion, SkyBlockage, ImuFailing, RadioWedge, Hang };

// The milestone 2 benchmark's sentence, byte rate, IMU read cost and SF12
// airtime, so both milestones measure the same simulated hardware.
const double kRigGpsByteRate = 0.96;
const char kRigFixSentence[] =
    "$GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,*47\r\n";
const uint32_t kRigImuReadMs = 10;
const uint32_t kRigRadioSf12Ms = 3023;
const uint16_t kRigNodeId = 0x0042;
const uint8_t kRigTtl = 3;

/// Ends a hang when the run ends: for the build that has no watchdog.
class ScenarioEnd : public IHangBreaker {
  public:
    explicit ScenarioEnd(SimClock &clock) : clock_(clock), end_ms_(0) {}
    void set_end(uint32_t end_ms) { end_ms_ = end_ms; }
    bool should_break() const override { return clock_.now_ms() >= end_ms_; }

  private:
    SimClock &clock_;
    uint32_t end_ms_;
};

/// One node, either build, on the same fakes, with a gateway that keeps what
/// it receives. Faults are applied between node steps, so one scheduled
/// inside a sleep jump lands up to SLEEP_CHUNK_MS late for the duty-cycled
/// build; the README states this.
class PowerRig {
  public:
    /// `capacity_nA_ms` of 0 means unlimited.
    PowerRig(Build build, uint32_t teensy_sleep_nA, uint64_t capacity_nA_ms)
        : build_(build),
          clock_(),
          gps_(kRigFixSentence, kRigGpsByteRate, FakeGps::MAX_FIFO_DEPTH),
          imu_(clock_, kRigImuReadMs),
          radio_(clock_, kRigRadioSf12Ms),
          power_(clock_, gps_, imu_, radio_, CurrentModel::with_teensy_sleep(teensy_sleep_nA),
                 capacity_nA_ms),
          watchdog_(),
          store_(),
          end_(clock_),
          dedup_(),
          last_sent_(0),
          reboots_(0) {
        // FakePower first: each tick is charged at the state it began in.
        clock_.add_observer(&power_);
        clock_.add_observer(&gps_);
        clock_.add_observer(&imu_);
        clock_.add_observer(&radio_);
        clock_.add_observer(&watchdog_);

        // Both builds start with the GPS cold, so their startups are alike.
        gps_.set_powered(false);
        if (build_ == Build::DutyCycled) {
            boot();
        } else {
            gps_.set_powered(true);
            sampler_.reset(
                new InterruptSampler(gps_, imu_, radio_, clock_, kRigNodeId, kRigTtl));
        }
    }

    void add_fault(FaultKind kind, uint32_t start_ms, uint32_t end_ms) {
        Fault fault;
        fault.kind = kind;
        fault.start_ms = start_ms;
        fault.end_ms = end_ms;
        fault.started = false;
        fault.ended = false;
        faults_.push_back(fault);
    }

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

    DowntimeReport downtime(uint32_t scenario_ms) const {
        DowntimeInput in;
        in.records = records_.data();
        in.count = records_.size();
        in.scenario_ms = scenario_ms;
        in.died = power_.depleted();
        in.died_at_ms = power_.depleted_at_ms();
        return compute_downtime(in);
    }

    SimClock &clock() { return clock_; }
    FakeGps &gps() { return gps_; }
    FakeImu &imu() { return imu_; }
    FakeRadio &radio() { return radio_; }
    FakePower &power() { return power_; }
    FakeWatchdog &watchdog() { return watchdog_; }
    const DutyCycledNode &node() const { return *node_; }
    const std::vector<Packet> &deliveries() const { return deliveries_; }
    const std::vector<GatewayRecord> &records() const { return records_; }
    size_t reboots() const { return reboots_; }

  private:
    struct Fault {
        FaultKind kind;
        uint32_t start_ms;
        uint32_t end_ms;  ///< 0: never ends by itself
        bool started;
        bool ended;
    };

    static DutyCycledNodeConfig config() {
        DutyCycledNodeConfig c;
        c.node_id = kRigNodeId;
        c.ttl = kRigTtl;
        return c;
    }

    void boot() {
        node_.reset(new DutyCycledNode(gps_, imu_, radio_, clock_, power_, watchdog_, store_,
                                       config()));
        node_->begin();
    }

    /// What a watchdog reset does: RAM is lost, the SNVS store and the
    /// peripherals keep their state, and WDOG1 is disabled until begin().
    void reboot() {
        ++reboots_;
        node_.reset();
        watchdog_.on_mcu_reset();
        boot();
    }

    void collect_delivery() {
        if (radio_.sent_count() == last_sent_) {
            return;
        }
        last_sent_ = radio_.sent_count();
        Packet packet;
        if (!decode_packet(radio_.last_payload(), radio_.last_length(), &packet)) {
            return;
        }
        // The gateway's own dedup, so a key bug would show up as lost records.
        if (dedup_.seen(packet.node_id, packet.boot_count, packet.seq)) {
            return;
        }
        deliveries_.push_back(packet);
        GatewayRecord record;
        record.arrival_ms = clock_.now_ms();
        record.gps_valid = packet.record.gps.valid;
        record.boot_count = packet.boot_count;
        record.tx_timeouts = packet.tx_timeouts;
        records_.push_back(record);
    }

    void apply_faults() {
        const uint32_t now = clock_.now_ms();
        for (size_t i = 0; i < faults_.size(); ++i) {
            Fault &f = faults_[i];
            if (!f.started && now >= f.start_ms) {
                f.started = true;
                begin_fault(f.kind);
            }
            if (f.started && !f.ended && f.end_ms != 0 && now >= f.end_ms) {
                f.ended = true;
                end_fault(f.kind);
            }
        }
    }

    void begin_fault(FaultKind kind) {
        switch (kind) {
        case FaultKind::LostCompletion:
            radio_.lose_next_completion();
            break;
        case FaultKind::SkyBlockage:
            gps_.set_sky_blocked(true);
            break;
        case FaultKind::ImuFailing:
            imu_.set_failing(true);
            break;
        case FaultKind::RadioWedge:
            radio_.set_wedged(true);  // cleared only by a power cycle
            break;
        case FaultKind::Hang:
            if (node_) {
                imu_.hang_next_read(&watchdog_);
            } else {
                imu_.hang_next_read(&end_);
            }
            break;
        }
    }

    void end_fault(FaultKind kind) {
        if (kind == FaultKind::SkyBlockage) {
            gps_.set_sky_blocked(false);
        } else if (kind == FaultKind::ImuFailing) {
            imu_.set_failing(false);
        }
    }

    Build build_;
    SimClock clock_;
    FakeGps gps_;
    FakeImu imu_;
    FakeRadio radio_;
    FakePower power_;
    FakeWatchdog watchdog_;
    FakePersistentStore store_;
    ScenarioEnd end_;
    DedupTable dedup_;
    std::unique_ptr<DutyCycledNode> node_;
    std::unique_ptr<InterruptSampler> sampler_;
    std::vector<Fault> faults_;
    std::vector<Packet> deliveries_;
    std::vector<GatewayRecord> records_;
    size_t last_sent_;
    size_t reboots_;
};

}  // namespace floodnet

#endif  // FLOODNET_TEST_POWER_RIG_HPP
```

- [ ] **Step 3: Write the failing node tests**

`test/test_duty_cycled/test_main.cpp`:

```cpp
#include <unity.h>

#include <floodnet/packet.hpp>

#include "../support/power_rig.hpp"

using namespace floodnet;

void setUp(void) {}
void tearDown(void) {}

static const uint32_t kSlotMs = 300000;

static PowerRig *make_rig() {
    return new PowerRig(Build::DutyCycled, TEENSY_SLEEP_LOW_NA, 0);
}

void test_first_report_follows_a_cold_start(void) {
    std::unique_ptr<PowerRig> rig(make_rig());
    rig->run_until(60000);

    TEST_ASSERT_EQUAL_UINT(1, rig->deliveries().size());
    const Packet &p = rig->deliveries()[0];
    TEST_ASSERT_TRUE(p.record.gps.valid);
    TEST_ASSERT_EQUAL_UINT16(1, p.boot_count);
    TEST_ASSERT_EQUAL_UINT32(0, p.seq);
    // 26 s cold start, one no-fix and one fix sentence, then 3023 ms airtime.
    TEST_ASSERT_GREATER_OR_EQUAL_UINT32(29000, rig->records()[0].arrival_ms);
    TEST_ASSERT_LESS_OR_EQUAL_UINT32(29500, rig->records()[0].arrival_ms);
}

void test_one_report_per_slot_on_an_anchored_schedule(void) {
    std::unique_ptr<PowerRig> rig(make_rig());
    rig->run_until(12 * kSlotMs);

    TEST_ASSERT_EQUAL_UINT(12, rig->deliveries().size());
    for (uint32_t k = 0; k < 12; ++k) {
        TEST_ASSERT_EQUAL_UINT32(k, rig->deliveries()[k].seq);
    }
    for (uint32_t k = 1; k < 12; ++k) {
        // Hot start (1 s) plus airtime (3023 ms), measured from the slot.
        const uint32_t late = rig->records()[k].arrival_ms - k * kSlotMs;
        TEST_ASSERT_GREATER_OR_EQUAL_UINT32(4000, late);
        TEST_ASSERT_LESS_OR_EQUAL_UINT32(4500, late);
    }
}

void test_watchdog_is_kicked_well_inside_its_timeout(void) {
    std::unique_ptr<PowerRig> rig(make_rig());
    rig->run_until(12 * kSlotMs);
    TEST_ASSERT_EQUAL_UINT(0, rig->reboots());
    TEST_ASSERT_LESS_OR_EQUAL_UINT32(61000, rig->watchdog().max_since_kick_ms());
}

void test_sky_blockage_sends_a_heartbeat_at_the_acquire_timeout(void) {
    std::unique_ptr<PowerRig> rig(make_rig());
    rig->gps().set_sky_blocked(true);
    rig->run_until(90000);

    TEST_ASSERT_EQUAL_UINT(1, rig->deliveries().size());
    TEST_ASSERT_FALSE(rig->deliveries()[0].record.gps.valid);
    TEST_ASSERT_GREATER_OR_EQUAL_UINT32(63000, rig->records()[0].arrival_ms);
    TEST_ASSERT_LESS_OR_EQUAL_UINT32(63500, rig->records()[0].arrival_ms);
}

void test_stale_sentence_from_before_sleep_is_not_reported(void) {
    std::unique_ptr<PowerRig> rig(make_rig());
    rig->run_until(100000);  // first report sent; asleep
    rig->gps().inject(kRigFixSentence);  // a whole fix left in the buffer
    rig->gps().set_sky_blocked(true);
    rig->run_until(kSlotMs + 90000);

    TEST_ASSERT_EQUAL_UINT(2, rig->deliveries().size());
    TEST_ASSERT_FALSE(rig->deliveries()[1].record.gps.valid);
}

void test_wedged_radio_is_power_cycled_and_recovers(void) {
    std::unique_ptr<PowerRig> rig(make_rig());
    rig->run_until(100000);
    rig->radio().set_wedged(true);
    rig->run_until(4 * kSlotMs + 60000);  // slots 1-3 fail; slot 4 succeeds

    TEST_ASSERT_EQUAL_UINT(1, rig->radio().power_cycles());
    TEST_ASSERT_EQUAL_UINT(2, rig->deliveries().size());
    TEST_ASSERT_EQUAL_UINT16(3, rig->deliveries()[1].tx_timeouts);
    TEST_ASSERT_GREATER_OR_EQUAL_UINT32(4 * kSlotMs, rig->records()[1].arrival_ms);
}

void test_refused_transmit_leaves_transmit_after_its_timeout(void) {
    std::unique_ptr<PowerRig> rig(make_rig());
    rig->run_until(100000);
    rig->radio().set_refusing(true);
    rig->run_until(kSlotMs + 1500);
    TEST_ASSERT_TRUE(rig->node().state() == NodeState::Transmit);
    rig->run_until(kSlotMs + 12000);
    TEST_ASSERT_TRUE(rig->node().state() == NodeState::Sleep);
    TEST_ASSERT_EQUAL_UINT(1, rig->deliveries().size());
}

void test_failing_imu_is_power_cycled_after_three_wakes(void) {
    std::unique_ptr<PowerRig> rig(make_rig());
    rig->run_until(100000);
    rig->imu().set_failing(true);
    rig->run_until(3 * kSlotMs + 60000);

    TEST_ASSERT_EQUAL_UINT(1, rig->imu().power_cycles());
    TEST_ASSERT_EQUAL_UINT(4, rig->deliveries().size());
    for (size_t k = 1; k < 4; ++k) {
        TEST_ASSERT_TRUE(rig->deliveries()[k].record.gps.valid);  // still not down
        TEST_ASSERT_FALSE(rig->deliveries()[k].record.imu.valid);
    }
}

void test_hang_is_reset_by_the_watchdog_and_reports_resume(void) {
    std::unique_ptr<PowerRig> rig(make_rig());
    rig->run_until(100000);
    rig->imu().hang_next_read(&rig->watchdog());
    rig->run_until(kSlotMs + 120000);

    TEST_ASSERT_EQUAL_UINT(1, rig->reboots());
    TEST_ASSERT_EQUAL_UINT16(2, rig->node().boot_count());
    TEST_ASSERT_EQUAL_UINT(2, rig->deliveries().size());
    // seq restarted at 0 after the reset; the gateway kept it anyway.
    TEST_ASSERT_EQUAL_UINT32(0, rig->deliveries()[1].seq);
    TEST_ASSERT_EQUAL_UINT16(2, rig->deliveries()[1].boot_count);
}

void test_interrupt_build_streams_and_never_sleeps(void) {
    PowerRig rig(Build::Interrupt, TEENSY_SLEEP_LOW_NA, 0);
    rig.run_until(60000);
    TEST_ASSERT_EQUAL_UINT(0, rig.power().sleeps());
    TEST_ASSERT_GREATER_THAN_UINT(5, rig.deliveries().size());
    TEST_ASSERT_EQUAL_UINT16(0, rig.deliveries()[0].boot_count);  // v0x01
}

int main(int, char **) {
    UNITY_BEGIN();
    RUN_TEST(test_first_report_follows_a_cold_start);
    RUN_TEST(test_one_report_per_slot_on_an_anchored_schedule);
    RUN_TEST(test_watchdog_is_kicked_well_inside_its_timeout);
    RUN_TEST(test_sky_blockage_sends_a_heartbeat_at_the_acquire_timeout);
    RUN_TEST(test_stale_sentence_from_before_sleep_is_not_reported);
    RUN_TEST(test_wedged_radio_is_power_cycled_and_recovers);
    RUN_TEST(test_refused_transmit_leaves_transmit_after_its_timeout);
    RUN_TEST(test_failing_imu_is_power_cycled_after_three_wakes);
    RUN_TEST(test_hang_is_reset_by_the_watchdog_and_reports_resume);
    RUN_TEST(test_interrupt_build_streams_and_never_sleeps);
    return UNITY_END();
}
```

- [ ] **Step 4: Run to verify they fail**

Run: `pio test -e native -f test_duty_cycled`
Expected: compile failure, `duty_cycled_node.hpp` not found.

- [ ] **Step 5: Implement the node**

`src/duty_cycled_node.hpp`:

```cpp
#ifndef FLOODNET_DUTY_CYCLED_NODE_HPP
#define FLOODNET_DUTY_CYCLED_NODE_HPP

#include <stddef.h>
#include <stdint.h>

#include <floodnet/hal/clock.hpp>
#include <floodnet/hal/gps.hpp>
#include <floodnet/hal/imu.hpp>
#include <floodnet/hal/persistent_store.hpp>
#include <floodnet/hal/power.hpp>
#include <floodnet/hal/radio.hpp>
#include <floodnet/hal/watchdog.hpp>
#include <floodnet/node_state.hpp>

#include "sampler_interrupt.hpp"

namespace floodnet {

struct DutyCycledNodeConfig {
    uint16_t node_id = 1;
    uint8_t ttl = 3;
    NodeTiming timing = NodeTiming();
    /// Above the longest legitimate gap between transitions (ACQUIRE's 60 s
    /// timeout), below WDOG1's 128 s ceiling.
    uint32_t watchdog_timeout_ms = 90000;
};

/// One fix every report interval, asleep in between, recovering from the
/// faults in the milestone 3 design doc.
///
/// NodeStateMachine decides what is powered and when; InterruptSampler does
/// the acquisition and transmission exactly as it does in node_interrupt.
/// This class only carries events from one to the other.
class DutyCycledNode {
  public:
    /// Persistent slot holding the boot counter.
    static const size_t BOOT_COUNT_SLOT = 0;

    DutyCycledNode(IGpsSource &gps, IImuSource &imu, IAsyncRadio &radio, IClock &clock,
                   IPower &power, IWatchdog &watchdog, IPersistentStore &store,
                   const DutyCycledNodeConfig &config);

    /// Call once, after the drivers are initialised.
    void begin();

    /// One pass of the main loop. May sleep for up to one watchdog chunk.
    void step();

    NodeState state() const { return fsm_.state(); }
    uint16_t boot_count() const { return boot_count_; }
    const InterruptSampler &sampler() const { return sampler_; }

  private:
    void apply(const NodeActions &actions);
    void set_power(Peripheral p, bool on, bool *applied);
    void drain_gps();

    IGpsSource &gps_;
    IClock &clock_;
    IPower &power_;
    IWatchdog &watchdog_;
    IPersistentStore &store_;
    DutyCycledNodeConfig config_;
    InterruptSampler sampler_;
    NodeStateMachine fsm_;
    PowerPlan applied_;
    bool power_known_;
    uint32_t seq_at_acquire_;
    uint32_t sent_at_transmit_;
    uint32_t sleep_until_ms_;
    uint16_t boot_count_;
};

}  // namespace floodnet

#endif  // FLOODNET_DUTY_CYCLED_NODE_HPP
```

`src/duty_cycled_node.cpp`:

```cpp
#include "duty_cycled_node.hpp"

namespace floodnet {

DutyCycledNode::DutyCycledNode(IGpsSource &gps, IImuSource &imu, IAsyncRadio &radio,
                               IClock &clock, IPower &power, IWatchdog &watchdog,
                               IPersistentStore &store, const DutyCycledNodeConfig &config)
    : gps_(gps),
      clock_(clock),
      power_(power),
      watchdog_(watchdog),
      store_(store),
      config_(config),
      sampler_(gps, imu, radio, clock, config.node_id, config.ttl, WireVersion::V2),
      fsm_(config.timing),
      applied_(),
      power_known_(false),
      seq_at_acquire_(0),
      sent_at_transmit_(0),
      sleep_until_ms_(0),
      boot_count_(0) {}

void DutyCycledNode::begin() {
    // Counted first, so every boot is visible at the gateway, including one
    // that ends in another reset before it reports.
    const uint32_t stored = store_.read_u32(BOOT_COUNT_SLOT);
    boot_count_ = stored >= 0xFFFF ? 0xFFFF : static_cast<uint16_t>(stored + 1);
    store_.write_u32(BOOT_COUNT_SLOT, boot_count_);

    watchdog_.begin(config_.watchdog_timeout_ms);
    apply(fsm_.boot(clock_.now_ms()));
    // The caller initialised the drivers before begin(), so the peripherals
    // are ready as soon as they are powered.
    apply(fsm_.handle(NodeEvent::PeripheralsReady, clock_.now_ms()));
}

void DutyCycledNode::step() {
    NodeEvent event = NodeEvent::Tick;

    switch (fsm_.state()) {
    case NodeState::Boot:
        event = NodeEvent::PeripheralsReady;
        break;

    case NodeState::Acquire:
        sampler_.step();
        if (sampler_.next_seq() != seq_at_acquire_) {
            event = sampler_.last_record_imu_valid() ? NodeEvent::FixQueued
                                                     : NodeEvent::FixQueuedNoImu;
        }
        break;

    case NodeState::Transmit:
        // The GPS is going into backup, but bytes already on the wire still
        // land in the receive buffer. Discarding them is what keeps this wake
        // to one fix: the sampler would otherwise queue a second.
        drain_gps();
        sampler_.step();
        if (sampler_.tx_queue_size() == 0 && !sampler_.tx_in_flight()) {
            event = sampler_.packets_sent() != sent_at_transmit_ ? NodeEvent::TxSucceeded
                                                                 : NodeEvent::TxFailed;
        }
        break;

    case NodeState::Sleep:
        power_.sleep_until(sleep_until_ms_);
        break;
    }

    apply(fsm_.handle(event, clock_.now_ms()));
}

void DutyCycledNode::apply(const NodeActions &actions) {
    if (actions.kick_watchdog) {
        watchdog_.kick();
    }
    // A power cycle leaves the part on; the plan below then applies the
    // state the new state wants.
    if (actions.power_cycle_radio) {
        power_.power_cycle(Peripheral::Radio);
        applied_.radio = true;
    }
    if (actions.power_cycle_imu) {
        power_.power_cycle(Peripheral::Imu);
        applied_.imu = true;
    }
    if (!actions.transitioned) {
        return;
    }

    switch (fsm_.state()) {
    case NodeState::Acquire:
        // Before the GPS is powered: a sentence left in the receive buffer
        // from the previous wake would otherwise be parsed now and stamped
        // with this wake's time, reporting a stale position as fresh.
        drain_gps();
        sampler_.set_node_status(boot_count_, power_.battery_mv());
        seq_at_acquire_ = sampler_.next_seq();
        break;
    case NodeState::Transmit:
        if (actions.queue_heartbeat) {
            sampler_.enqueue_heartbeat();
        }
        sent_at_transmit_ = sampler_.packets_sent();
        break;
    case NodeState::Sleep:
        sleep_until_ms_ = actions.sleep_until_ms;
        break;
    case NodeState::Boot:
        break;
    }

    set_power(Peripheral::Gps, actions.power.gps, &applied_.gps);
    set_power(Peripheral::Imu, actions.power.imu, &applied_.imu);
    set_power(Peripheral::Radio, actions.power.radio, &applied_.radio);
    power_known_ = true;
}

void DutyCycledNode::set_power(Peripheral p, bool on, bool *applied) {
    // Only on change: waking the GPS or resuming the IMU is not free on
    // hardware, and a sleep chunk renewal must not repeat it.
    if (power_known_ && *applied == on) {
        return;
    }
    power_.set_power(p, on ? PowerState::On : PowerState::Low);
    *applied = on;
}

void DutyCycledNode::drain_gps() {
    while (gps_.read_byte() >= 0) {
    }
}

}  // namespace floodnet
```

- [ ] **Step 6: Run the node tests**

Run: `pio test -e native -f test_duty_cycled`
Expected: 10 tests PASS.

If a timing assertion misses its window, investigate the cause with the systematic-debugging skill before touching anything. The windows follow from the cited constants (26 s cold start, 1 s hot start, 3023 ms airtime); a miss means a bug in the node, the fakes or the window's arithmetic, and the fix goes to whichever one is wrong. Never widen a window just to make it pass.

- [ ] **Step 7: Run the whole suite and the builds**

Run: `pio test -e native && pio run -e node_polling -e node_interrupt -e gateway`
Expected: all PASS; all three firmware images build with no warnings.

- [ ] **Step 8: Commit**

```bash
git add src/duty_cycled_node.hpp src/duty_cycled_node.cpp test/support/power_rig.hpp test/test_duty_cycled/test_main.cpp platformio.ini
git commit -m "feat: add the duty-cycled node and a harness that runs both builds"
```

---

### Task 10: Teensy implementations and the `node_duty_cycled` build

**Files:**
- Modify: `src/hal/teensy_gps.hpp`, `src/hal/teensy_imu.hpp`, `src/hal/teensy_radio.hpp`
- Create: `src/hal/teensy_power.hpp`, `src/hal/teensy_watchdog.hpp`, `src/hal/teensy_persistent_store.hpp`
- Modify: `src/main.cpp`
- Modify: `platformio.ini`
- Modify: `.github/workflows/ci.yml`
- Modify: `docs/hardware.md`

**Interfaces:**
- Consumes: `IPower`, `IWatchdog`, `IPersistentStore` (Task 7), `ubx_pmreq_backup()` (Task 3), `DutyCycledNode` (Task 9).
- Produces: the `node_duty_cycled` firmware image. There is no host test for this task: every file here includes Arduino headers. The check is that all four firmware targets compile, which is all CI can check without a board.

- [ ] **Step 1: Driver low-power methods**

`teensy_gps.hpp`: add `#include <floodnet/ubx.hpp>` and, after `note_buffer_state()`:

```cpp
    /// Software backup via UBX-RXM-PMREQ, keeping ephemeris for a hot start.
    /// The frame asks for wake-up on UART RX activity; see wake().
    void enter_backup() {
        uint8_t frame[UBX_PMREQ_BACKUP_SIZE];
        const size_t len = ubx_pmreq_backup(frame, sizeof(frame));
        port_.write(frame, len);
        port_.flush();  // wait until the last byte is out before anything sleeps
    }

    /// Any edge on the module's RX line wakes it from backup. Unverified on a
    /// board, like enter_backup().
    void wake() {
        port_.write(static_cast<uint8_t>(0xFF));
        port_.flush();
    }
```

`teensy_imu.hpp`, after `read()`:

```cpp
    /// BNO055 suspend mode, and no timer ticks while suspended: a flag set
    /// with nothing to read would make the sampler issue a pointless read.
    void suspend() {
        timer_.end();
        s_sample_due = false;
        if (ready_) {
            sensor_.enterSuspendMode();
        }
    }

    /// Back to NDOF output. Primes a read, as begin() does, and fails the same
    /// way begin() does if no hardware timer is available.
    bool resume() {
        if (!ready_) {
            return false;
        }
        sensor_.enterNormalMode();
        s_sample_due = true;
        if (!timer_.begin(on_sample_due, SAMPLE_PERIOD_US)) {
            ready_ = false;
            return false;
        }
        return true;
    }
```

`teensy_radio.hpp`, after `abort_transmit()`:

```cpp
    /// RFM95W sleep mode (0.2 uA typical). The FIFO is not retained.
    void sleep() {
        if (ready_) {
            driver_.sleep();
        }
    }

    /// Standby, ready for send().
    void wake() {
        if (ready_) {
            driver_.setModeIdle();
        }
    }
```

- [ ] **Step 2: The three new Teensy HAL implementations**

`src/hal/teensy_watchdog.hpp`:

```cpp
#ifndef FLOODNET_TEENSY_WATCHDOG_HPP
#define FLOODNET_TEENSY_WATCHDOG_HPP

#include <Arduino.h>

#include <floodnet/hal/watchdog.hpp>

namespace floodnet {

/// WDOG1, programmed directly from imxrt.h rather than through a library.
///
/// WCR[WT] counts in half seconds: the timeout is (WT + 1) x 0.5 s, up to
/// 128 s (Linux imx2_wdt, IMX2_WDT_MAX_TIME). WDZST is left clear on purpose:
/// suspending the watchdog in low-power modes would let a wake timer that
/// never fires hang the node with nothing to reset it. See the milestone 3
/// design doc, "Watchdog timing". Whether WDOG1 keeps counting through Snooze
/// deepSleep is unverified without a board.
class TeensyWatchdog : public IWatchdog {
  public:
    void begin(uint32_t timeout_ms) override {
        CCM_CCGR3 |= CCM_CCGR3_WDOG1(CCM_CCGR_ON);
        const uint32_t half_seconds = timeout_ms / 500;
        const uint32_t wt = half_seconds == 0 ? 0 : (half_seconds > 256 ? 255 : half_seconds - 1);
        WDOG1_WMCR = 0;  // power-down counter off: it would reset us after 16 s
        WDOG1_WCR = WDOG_WCR_WDE | WDOG_WCR_SRS | WDOG_WCR_WDA |
                    WDOG_WCR_WT(static_cast<uint8_t>(wt));
        kick();
    }

    void kick() override {
        // The documented service sequence.
        WDOG1_WSR = 0x5555;
        WDOG1_WSR = 0xAAAA;
    }
};

}  // namespace floodnet

#endif  // FLOODNET_TEENSY_WATCHDOG_HPP
```

`src/hal/teensy_persistent_store.hpp`:

```cpp
#ifndef FLOODNET_TEENSY_PERSISTENT_STORE_HPP
#define FLOODNET_TEENSY_PERSISTENT_STORE_HPP

#include <Arduino.h>

#include <floodnet/hal/persistent_store.hpp>

namespace floodnet {

/// The SNVS low-power general purpose registers, retained across resets while
/// SNVS is powered (i.MX RT1060 reference manual, section 19.4.2.1). Registers
/// rather than flash, so writing on every boot wears nothing out. Retention
/// across a WDOG1 reset specifically is unverified on a board.
class TeensyPersistentStore : public IPersistentStore {
  public:
    uint32_t read_u32(size_t slot) override {
        switch (slot) {
        case 0:
            return SNVS_LPGPR0;
        case 1:
            return SNVS_LPGPR1;
        case 2:
            return SNVS_LPGPR2;
        case 3:
            return SNVS_LPGPR3;
        default:
            return 0;
        }
    }

    void write_u32(size_t slot, uint32_t value) override {
        switch (slot) {
        case 0:
            SNVS_LPGPR0 = value;
            break;
        case 1:
            SNVS_LPGPR1 = value;
            break;
        case 2:
            SNVS_LPGPR2 = value;
            break;
        case 3:
            SNVS_LPGPR3 = value;
            break;
        default:
            break;
        }
    }
};

}  // namespace floodnet

#endif  // FLOODNET_TEENSY_PERSISTENT_STORE_HPP
```

`src/hal/teensy_power.hpp`:

```cpp
#ifndef FLOODNET_TEENSY_POWER_HPP
#define FLOODNET_TEENSY_POWER_HPP

#include <Arduino.h>
#include <Snooze.h>

#include <floodnet/hal/power.hpp>

#include "teensy_gps.hpp"
#include "teensy_imu.hpp"
#include "teensy_radio.hpp"

namespace floodnet {

class TeensyPower : public IPower {
  public:
    /// A0, behind a divider of two 100 kOhm resistors from the cell. See
    /// docs/hardware.md.
    static const uint8_t BATTERY_PIN = 14;

    TeensyPower(TeensyGps &gps, TeensyImu &imu, TeensyRadio &radio, float frequency_mhz,
                int8_t tx_power_dbm)
        : gps_(gps),
          imu_(imu),
          radio_(radio),
          frequency_mhz_(frequency_mhz),
          tx_power_dbm_(tx_power_dbm),
          timer_(),
          block_(timer_) {}

    void set_power(Peripheral p, PowerState s) override {
        const bool on = s == PowerState::On;
        switch (p) {
        case Peripheral::Gps:
            if (on) {
                gps_.wake();
            } else {
                gps_.enter_backup();
            }
            break;
        case Peripheral::Imu:
            if (on) {
                imu_.resume();
            } else {
                imu_.suspend();
            }
            break;
        case Peripheral::Radio:
            if (on) {
                radio_.wake();
            } else {
                radio_.sleep();
            }
            break;
        }
    }

    bool power_cycle(Peripheral p) override {
        switch (p) {
        case Peripheral::Gps:
            gps_.enter_backup();
            gps_.wake();
            return true;
        case Peripheral::Imu:
            imu_.suspend();
            return imu_.begin();  // begin() resets the BNO055
        case Peripheral::Radio:
            return radio_.begin(frequency_mhz_, tx_power_dbm_);  // pulses the reset pin
        }
        return false;
    }

    void sleep_until(uint32_t wake_ms) override {
        const int32_t remaining = static_cast<int32_t>(wake_ms - millis());
        if (remaining <= 0) {
            return;
        }

        // Snooze's Teensy 4 timer takes whole seconds.
        const uint32_t whole_s = static_cast<uint32_t>(remaining) / 1000;
        if (whole_s > 0) {
            timer_.setTimer(whole_s);
            Snooze.deepSleep(block_);
            // Snooze 6.3.9 (src/hal/TEENSY_40/SnoozeTimer.cpp) stores
            // period = seconds * 32768 and on wake adds period / 1000 to
            // systick_millis_count: 32.768 ms per slept second instead of
            // 1000. Add the rest, or the schedule stretches about thirty-fold.
            // Read from the library source; not observed on a board.
            systick_millis_count += whole_s * 1000 - (whole_s * 32768) / 1000;
        }

        // The sub-second remainder, and any early wake, on the ordinary tick.
        while (static_cast<int32_t>(wake_ms - millis()) > 0) {
            __asm__ volatile("wfi" ::: "memory");
        }
    }

    uint16_t battery_mv() override {
        // 10-bit ADC against 3.3 V, doubled for the divider.
        const uint32_t raw = static_cast<uint32_t>(analogRead(BATTERY_PIN));
        return static_cast<uint16_t>(raw * 3300UL * 2UL / 1023UL);
    }

  private:
    TeensyGps &gps_;
    TeensyImu &imu_;
    TeensyRadio &radio_;
    float frequency_mhz_;
    int8_t tx_power_dbm_;
    SnoozeTimer timer_;
    SnoozeBlock block_;
};

}  // namespace floodnet

#endif  // FLOODNET_TEENSY_POWER_HPP
```

- [ ] **Step 3: `main.cpp`**

Replace the include block and sampler selection with:

```cpp
#include <Arduino.h>

#include "hal/teensy_clock.hpp"
#include "hal/teensy_gps.hpp"
#include "hal/teensy_imu.hpp"
#include "hal/teensy_radio.hpp"
#include "radio_config.hpp"

#if defined(FLOODNET_NODE_DUTY_CYCLED)
#include "duty_cycled_node.hpp"
#include "hal/teensy_persistent_store.hpp"
#include "hal/teensy_power.hpp"
#include "hal/teensy_watchdog.hpp"
#elif defined(FLOODNET_SAMPLER_INTERRUPT)
#include "sampler_interrupt.hpp"
#elif defined(FLOODNET_SAMPLER_POLLING)
#include "sampler_polling.hpp"
#else
#error "Define exactly one of FLOODNET_SAMPLER_POLLING, FLOODNET_SAMPLER_INTERRUPT or FLOODNET_NODE_DUTY_CYCLED"
#endif
```

and inside the anonymous namespace replace the sampler globals with:

```cpp
#if defined(FLOODNET_NODE_DUTY_CYCLED)
floodnet::TeensyPower g_power(g_gps, g_imu, g_radio, RADIO_FREQUENCY_MHZ, RADIO_TX_POWER_DBM);
floodnet::TeensyWatchdog g_watchdog;
floodnet::TeensyPersistentStore g_store;
floodnet::DutyCycledNode g_node(g_gps, g_imu, g_radio, g_clock, g_power, g_watchdog, g_store,
                                floodnet::DutyCycledNodeConfig{NODE_ID, PACKET_TTL});
const char *const SAMPLER_NAME = "duty-cycled";
#elif defined(FLOODNET_SAMPLER_INTERRUPT)
floodnet::InterruptSampler g_sampler(g_gps, g_imu, g_radio, g_clock, NODE_ID, PACKET_TTL);
const char *const SAMPLER_NAME = "interrupt";
#else
floodnet::PollingSampler g_sampler(g_gps, g_imu, g_radio, g_clock, NODE_ID, PACKET_TTL);
const char *const SAMPLER_NAME = "polling";
#endif
```

At the end of `setup()`:

```cpp
#if defined(FLOODNET_NODE_DUTY_CYCLED)
    g_node.begin();
#endif
```

and `loop()`:

```cpp
void loop() {
    g_gps.note_buffer_state();
#if defined(FLOODNET_NODE_DUTY_CYCLED)
    g_node.step();
#else
    g_sampler.step();
#endif
}
```

- [ ] **Step 4: Build target and CI**

`platformio.ini`, after `[env:node_interrupt]`:

```ini
; Snooze is bundled with framework-arduinoteensy, like RadioHead, and is found
; by #include <Snooze.h> alone. Do not add it to lib_deps, for the same reason
; RadioHead is not pinned (see above).
[env:node_duty_cycled]
extends = teensy_base
build_src_filter = +<*> -<gateway_main.cpp> -<sampler_polling.cpp>
build_flags = ${env.build_flags} -D FLOODNET_NODE_DUTY_CYCLED
lib_deps =
    adafruit/Adafruit BNO055@^1.6.3
```

`.github/workflows/ci.yml`, the build step:

```yaml
      - name: Build node and gateway
        run: pio run -e node_polling -e node_interrupt -e node_duty_cycled -e gateway
```

- [ ] **Step 5: Build every firmware target**

Run: `pio run -e node_polling -e node_interrupt -e node_duty_cycled -e gateway`
Expected: all four SUCCESS with no warnings from project files.

If Snooze fails to compile against the bundled framework, stop and report the error. Do not vendor, patch or replace the library.

- [ ] **Step 6: `docs/hardware.md`**

Add a row to the bill of materials table:

```markdown
| Battery sense | 2 x 100 kΩ divider | Cell to Teensy pin 14 (A0), for `battery_mv` |
```

Add a row to the pin assignment table:

```markdown
| Battery sense (divided) | 14 (A0) |
```

Append a section:

```markdown
## Power modes

Milestone 3 puts each part into its own low-power mode rather than switching its supply, so the only hardware addition is the battery divider above.

| Part | Low-power mode | Driver call |
|---|---|---|
| NEO-M8N | software backup, woken by UART RX activity | `TeensyGps::enter_backup()` / `wake()`, using `UBX-RXM-PMREQ` |
| BNO055 | suspend | `TeensyImu::suspend()` / `resume()` |
| RFM95W | sleep | `TeensyRadio::sleep()` / `wake()` |
| Teensy 4.1 | Snooze `deepSleep()` with a GPT timer wake | `TeensyPower::sleep_until()` |

The bundled Snooze 6.3.9 miscounts time on Teensy 4: after a timer wake it advances `millis()` by 32.768 ms per slept second instead of 1000.
`TeensyPower::sleep_until()` adds the difference itself; the comment there cites the library line.
This is read from the library source and has not been observed on a board.

The watchdog is WDOG1 at 90 s, programmed directly through `imxrt.h`.
The boot counter lives in `SNVS_LPGPR0`, which survives resets while SNVS is powered.

None of this has run on hardware.
```

- [ ] **Step 7: Commit**

```bash
git add src/hal/ src/main.cpp platformio.ini .github/workflows/ci.yml docs/hardware.md
git commit -m "feat: add the node_duty_cycled firmware with Teensy sleep, watchdog and SNVS store"
```

---

### Task 11: The experiments

**Files:**
- Create: `test/test_power_bench/test_main.cpp`

**Interfaces:**
- Consumes: `PowerRig`, `FaultKind`, `Build` (Task 9), `compute_downtime`, `down_cause_name` (Task 5), `CurrentModel` constants (Task 7).
- Produces: `LIFE,...` and `DOWNTIME,...` lines, and the null-control assertion.

`LIFE,build,sleep_current_ua,died_at_ms,days` is a line type the spec did not name. Experiment 1's result is a lifetime, and forcing it into the `DOWNTIME` shape would print a meaningless `down_ms`. Task 12 records this in the spec's revisions.

- [ ] **Step 1: Write the experiments**

`test/test_power_bench/test_main.cpp`:

```cpp
#include <stdio.h>

#include <unity.h>

#include <floodnet/downtime.hpp>

#include "../support/current_model.hpp"
#include "../support/power_rig.hpp"

using namespace floodnet;

void setUp(void) {}
void tearDown(void) {}

// Frozen in the milestone 3 design doc, "Experiments", before any of this ran.
// Changing any of these after seeing output must be recorded in the README.
static const uint32_t kHourMs = 3600000UL;
static const uint32_t kDayMs = 24UL * kHourMs;
static const uint32_t kLifeCapMs = 45UL * kDayMs;  // uint32 ms tops out at 49.7 days
static const uint32_t kFaultRunMs = 24UL * kHourMs;
static const uint32_t kFaultAtMs = 2UL * kHourMs;
static const uint32_t kCombinedMs = 30UL * kDayMs;

static const char *build_name(Build build) {
    return build == Build::Interrupt ? "interrupt" : "duty_cycled";
}

/// 0 for the build that never sleeps, where the figure does not apply.
static uint32_t sleep_ua(Build build, uint32_t sleep_nA) {
    return build == Build::Interrupt ? 0 : sleep_nA / 1000;
}

static void print_downtime(const char *experiment, Build build, uint32_t sleep_nA,
                           const DowntimeReport &report, uint32_t scenario_ms,
                           const char *recovered) {
    for (size_t i = 0; i < DOWN_CAUSE_COUNT; ++i) {
        printf("DOWNTIME,%s,%s,%u,%s,%u,%u,%s\n", experiment, build_name(build),
               sleep_ua(build, sleep_nA), down_cause_name(static_cast<DownCause>(i)),
               report.down_ms[i], scenario_ms, recovered);
    }
}

static void run_life(Build build, uint32_t sleep_nA) {
    PowerRig rig(build, sleep_nA, NCR18650B_CAPACITY_NA_MS);
    rig.run_until(kLifeCapMs);
    // A harness bound, not a result: if a build outlives 45 days, the run
    // needs a longer timeline, not a quieter assertion.
    TEST_ASSERT_TRUE_MESSAGE(rig.power().depleted(), "battery outlived the 45-day cap");
    const uint32_t died = rig.power().depleted_at_ms();
    printf("LIFE,%s,%u,%u,%.2f\n", build_name(build), sleep_ua(build, sleep_nA), died,
           static_cast<double>(died) / kDayMs);
}

void test_experiment_1_battery_life(void) {
    run_life(Build::Interrupt, TEENSY_SLEEP_LOW_NA);
    run_life(Build::DutyCycled, TEENSY_SLEEP_LOW_NA);
    run_life(Build::DutyCycled, TEENSY_SLEEP_HIGH_NA);
}

struct FaultCase {
    const char *name;
    FaultKind kind;
    uint32_t duration_ms;  // 0: does not end by itself
};

static const FaultCase kFaults[] = {
    {"fault_lost_completion", FaultKind::LostCompletion, 0},
    {"fault_sky_blockage", FaultKind::SkyBlockage, 2UL * 3600000UL},
    {"fault_imu_failing", FaultKind::ImuFailing, 6UL * 3600000UL},
    {"fault_radio_wedge", FaultKind::RadioWedge, 0},
    {"fault_hang", FaultKind::Hang, 0},
};

void test_experiment_2_fault_recovery(void) {
    const Build builds[] = {Build::Interrupt, Build::DutyCycled};
    for (size_t b = 0; b < 2; ++b) {
        for (size_t f = 0; f < sizeof(kFaults) / sizeof(kFaults[0]); ++f) {
            PowerRig rig(builds[b], TEENSY_SLEEP_LOW_NA, 0);
            const uint32_t end =
                kFaults[f].duration_ms == 0 ? 0 : kFaultAtMs + kFaults[f].duration_ms;
            rig.add_fault(kFaults[f].kind, kFaultAtMs, end);
            rig.run_until(kFaultRunMs);
            const DowntimeReport report = rig.downtime(kFaultRunMs);
            print_downtime(kFaults[f].name, builds[b], TEENSY_SLEEP_LOW_NA, report, kFaultRunMs,
                           report.down_at_end ? "0" : "1");
        }
    }
}

void test_null_control_shows_no_downtime_after_the_first_report(void) {
    const Build builds[] = {Build::Interrupt, Build::DutyCycled};
    for (size_t b = 0; b < 2; ++b) {
        PowerRig rig(builds[b], TEENSY_SLEEP_LOW_NA, 0);
        rig.run_until(kFaultRunMs);
        const DowntimeReport report = rig.downtime(kFaultRunMs);
        print_downtime("null_control", builds[b], TEENSY_SLEEP_LOW_NA, report, kFaultRunMs, "-");
        // With no faults and no battery limit, any downtime past startup is
        // the harness producing the number, not the firmware.
        TEST_ASSERT_EQUAL_UINT32(report.down_ms[static_cast<size_t>(DownCause::Startup)],
                                 report.total_down_ms);
    }
}

void test_experiment_3_combined_30_days(void) {
    const Build builds[] = {Build::Interrupt, Build::DutyCycled};
    for (size_t b = 0; b < 2; ++b) {
        PowerRig rig(builds[b], TEENSY_SLEEP_LOW_NA, NCR18650B_CAPACITY_NA_MS);
        rig.add_fault(FaultKind::LostCompletion, 2 * kDayMs, 0);
        rig.add_fault(FaultKind::SkyBlockage, 5 * kDayMs, 5 * kDayMs + 2 * kHourMs);
        rig.add_fault(FaultKind::ImuFailing, 8 * kDayMs, 8 * kDayMs + 6 * kHourMs);
        rig.add_fault(FaultKind::RadioWedge, 11 * kDayMs, 0);
        rig.add_fault(FaultKind::Hang, 14 * kDayMs, 0);
        rig.run_until(kCombinedMs);
        print_downtime("combined_30d", builds[b], TEENSY_SLEEP_LOW_NA,
                       rig.downtime(kCombinedMs), kCombinedMs, "-");
    }
}

int main(int, char **) {
    UNITY_BEGIN();
    RUN_TEST(test_null_control_shows_no_downtime_after_the_first_report);
    RUN_TEST(test_experiment_1_battery_life);
    RUN_TEST(test_experiment_2_fault_recovery);
    RUN_TEST(test_experiment_3_combined_30_days);
    return UNITY_END();
}
```

- [ ] **Step 2: Run the experiments once and keep the output**

Run: `pio test -e native -f test_power_bench -v 2>&1 | tee /tmp/claude-power-bench.txt | grep -E '^(LIFE|DOWNTIME),|PASS|FAIL'`

(Use the session scratchpad directory in place of `/tmp` if one is provided.)

Expected: 4 tests PASS, 3 `LIFE` lines, and `DOWNTIME` lines for the null control (2 builds x 6 causes), experiment 2 (2 x 5 x 6) and experiment 3 (2 x 6). Record the wall-clock time.

If the null control fails, stop and debug it with the systematic-debugging skill: it means the harness is producing downtime. Do not change any constant in this file or in `current_model.hpp` in response to a number you have seen. This is the milestone's integrity rule; see the spec, "Rules".

- [ ] **Step 3: Sanity-check the output against the spec's rough estimate, without adjusting anything**

The spec's pre-implementation estimate was: interrupt build about 12 hours of battery life, duty-cycled build about 15 days at 6 mA and about 4.7 days at 25.86 mA.
If a `LIFE` figure is off from those by more than a factor of two, look for a bug (a current applied to the wrong state, a sleep jump not charged), fix the bug with a test, and rerun.
Being different from the estimate is not by itself a bug; only an identified defect justifies a change.
List every rerun and its reason in the commit message.

- [ ] **Step 4: Run the whole suite and time it**

Run: `time pio test -e native`
Expected: every suite PASSES. Note the total time for the README (CI runs this on every push).

- [ ] **Step 5: Commit**

```bash
git add test/test_power_bench/test_main.cpp
git commit -m "test: measure battery life, fault recovery and 30-day downtime for both builds"
```

---

### Task 12: Publish the results

**Files:**
- Modify: `README.md`
- Modify: `docs/superpowers/specs/2026-09-29-milestone-3-power-management-design.md`

**Interfaces:**
- Consumes: the output saved in Task 11, Step 2. Every number written here must come from that file; nothing is rounded toward a preferred value.

- [ ] **Step 1: Update the spec**

Change `Status:` to `implemented`.
Append to "Revisions made while planning":

```markdown
- **Cause `startup`.** The definition counts a node as down until its first valid record; `startup` names that interval so it is not reported as `silent`.
- **`LIFE` lines.** Experiment 1 reports a lifetime, printed as `LIFE,build,sleep_current_ua,died_at_ms,days`, rather than forcing it into the `DOWNTIME` shape.
```

Add any further change made during implementation in the same list, with its reason. If none, add nothing.

- [ ] **Step 2: Update the README**

Edit these sections, one sentence per line, using the saved output verbatim:

1. **Status:** milestone 3, power management and fault recovery; three node targets, and which is which.
2. **Building:** add `pio run -e node_duty_cycled`.
3. **Wire format:** state that `node_polling` and `node_interrupt` send v0x01 and `node_duty_cycled` sends v0x02. Add a v0x02 byte table regenerated from `encode_packet_v2()` in `packet.cpp`, pinned by `test_v2_golden_vector_pins_byte_layout`, and the airtime reason it stays at 45 bytes (80 symbols at 45 bytes, 88 at 46 to 50).
4. **Gateway serial output:** the `REC` line gains `boot_count,tx_timeouts,battery_mv` at the end; v0x01 records print `0,0,0`; dedup keys on `(node_id, boot_count, seq)`, and why.
5. **Results:** a new "Milestone 3: downtime" subsection:
   - how to reproduce: `pio test -e native -f test_power_bench -v`
   - the downtime definition, the three experiments and the frozen scenario, in brief
   - the raw `LIFE` and `DOWNTIME` lines, as printed
   - a table per experiment derived from them
   - what the numbers do and do not show, written from the data. In particular: which share of the duty-cycled build's energy goes to the Teensy asleep, whether that makes the MCU choice the finding, and, for experiment 2, which faults each build recovers from (the `recovered` column), not how many minutes a non-recovering build accumulated
   - the null control and what it guards
6. **Known limitations:** add
   - currents are typical data sheet values taken at face value, with no regulator efficiency or quiescent current; Teensy sleep current is a forum measurement, reported at two values
   - the Snooze `millis()` finding and the correction in `TeensyPower`
   - faults scheduled inside a sleep jump land up to 60 s late for the duty-cycled build
   - `battery_mv` in simulation is a straight-line placeholder
   - the full "Not validated on hardware" list from the spec
   - the test suite's run time, since CI runs the experiments on every push
7. **Deferred to later milestones:** remove "Node state machine and sleep/duty cycling" and the watchdog item (now built), keeping the explanation of how a wedged radio is now recovered. Keep mesh relay, ring buffer, sequence-gap counting and host trace replay. Add low-battery load shedding, and hardware-in-the-loop validation of everything in "Not validated on hardware".

- [ ] **Step 3: Check the numbers**

Run: `grep -E '^(LIFE|DOWNTIME),' /tmp/claude-power-bench.txt | while read -r line; do grep -qF "$line" README.md || echo "MISSING: $line"; done`
Expected: no output. Every printed line appears verbatim in the README.

- [ ] **Step 4: Check the writing rules**

Run: `grep -n $'\xe2\x80\x94' README.md docs/hardware.md docs/superpowers/specs/2026-09-29-milestone-3-power-management-design.md docs/superpowers/plans/2026-09-29-milestone-3-power-management.md`
Expected: no output.

- [ ] **Step 5: Final verification**

Run: `pio test -e native && pio run -e node_polling -e node_interrupt -e node_duty_cycled -e gateway && diff <(grep -o '^BENCH,[^ ]*' README.md) <(pio test -e native -f test_benchmark -v 2>&1 | grep -o 'BENCH,[^ ]*')`
Expected: all tests PASS, all four images build, and the diff prints nothing.

- [ ] **Step 6: Commit**

```bash
git add README.md docs/superpowers/specs/2026-09-29-milestone-3-power-management-design.md
git commit -m "docs: publish the milestone 3 results and what they prove"
```
