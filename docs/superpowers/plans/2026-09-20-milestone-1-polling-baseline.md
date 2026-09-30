# FloodNet Milestone 1: Polling Baseline Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Build a complete, working polling-based firmware for the FloodNet sensor node and gateway, with host-runnable unit tests and continuous integration.

**Architecture:** A platform-independent core library (`lib/floodnet_core`) holds all parsing, packing, and relay logic with no hardware dependency, so it compiles and tests on a development machine. Hardware sits behind pure-virtual interfaces (`lib/floodnet_hal`), implemented for Teensy in `src/hal/` and faked in `test/support/` for tests. A `PollingSampler` class holds the superloop body as testable logic, and `src/main.cpp` is a thin wrapper that calls it.

**Tech Stack:** C++17, PlatformIO 6.1.18, Teensy 4.1 (Arduino framework), Unity test framework, GitHub Actions. Third-party drivers: Adafruit BNO055 (IMU), RadioHead RH_RF95 (LoRa).

**Spec:** `docs/superpowers/specs/2026-09-20-floodnet-firmware-design.md`

## Global Constraints

- No file under `lib/floodnet_core/` or `lib/floodnet_hal/` may include an Arduino or Teensy header. This is what keeps the core host-testable.
- C++ standard is `gnu++17`, set via `build_flags` in every environment.
- All multi-byte integers on the wire are little-endian.
- Packet wire size is exactly 45 bytes (`floodnet::PACKET_SIZE`).
- CRC is CRC16-CCITT-FALSE: polynomial `0x1021`, initial value `0xFFFF`, no reflection, no final XOR.
- Namespace for all core and HAL code is `floodnet`.
- Test fakes are header-only, living in `test/support/`, so they need no separate build configuration.
- Commit messages follow Conventional Commits (`feat:`, `test:`, `chore:`, `docs:`).
- Do not add a co-author trailer to any commit.

---

## File Structure

| File | Responsibility |
|---|---|
| `platformio.ini` | Four build environments: `node_polling`, `gateway`, `native`, plus shared defaults |
| `lib/floodnet_core/library.json` | Marks the core as a PlatformIO library |
| `lib/floodnet_core/include/floodnet/sample.hpp` | `GpsFix`, `ImuSample`, `DiagCounters`, `SensorRecord` value types |
| `lib/floodnet_core/include/floodnet/packet.hpp` | Packet type, CRC and codec declarations |
| `lib/floodnet_core/src/packet.cpp` | CRC16 and little-endian encode/decode |
| `lib/floodnet_core/include/floodnet/nmea.hpp` | NMEA checksum and GGA parse declarations |
| `lib/floodnet_core/src/nmea.cpp` | NMEA sentence parsing |
| `lib/floodnet_core/include/floodnet/pairing.hpp` | `SamplePairer` declaration |
| `lib/floodnet_core/src/pairing.cpp` | Timestamp-based GPS/IMU pairing |
| `lib/floodnet_core/include/floodnet/mesh.hpp` | `DedupTable` and relay-decision declarations |
| `lib/floodnet_core/src/mesh.cpp` | Duplicate suppression and TTL handling |
| `lib/floodnet_hal/library.json` | Marks the HAL as a PlatformIO library |
| `lib/floodnet_hal/include/floodnet/hal/clock.hpp` | `IClock` interface |
| `lib/floodnet_hal/include/floodnet/hal/gps.hpp` | `IGpsSource` interface |
| `lib/floodnet_hal/include/floodnet/hal/imu.hpp` | `IImuSource` interface |
| `lib/floodnet_hal/include/floodnet/hal/radio.hpp` | `IRadio` interface |
| `src/sampler_polling.hpp` / `.cpp` | `PollingSampler`, the superloop body as testable logic |
| `src/main.cpp` | Node entry point wiring Teensy HAL to `PollingSampler` |
| `src/gateway_main.cpp` | Gateway entry point: receive, validate, write to serial |
| `src/hal/teensy_*.cpp` | Teensy implementations of the four interfaces |
| `test/support/*.hpp` | Header-only simulation fakes |
| `test/test_*/test_main.cpp` | Unity test suites |
| `.github/workflows/ci.yml` | Native tests plus firmware compile on every push |

---

### Task 1: Project skeleton, build environments, and CI

Establishes the repository so every later task has a working test loop.
Ends with one trivial passing test that proves `pio test -e native` runs end to end.

**Files:**
- Create: `.gitignore`
- Create: `platformio.ini`
- Create: `lib/floodnet_core/library.json`
- Create: `lib/floodnet_core/include/floodnet/sample.hpp`
- Create: `lib/floodnet_hal/library.json`
- Create: `.github/workflows/ci.yml`
- Test: `test/test_sample/test_main.cpp`

**Interfaces:**
- Consumes: nothing.
- Produces: `floodnet::GpsFix`, `floodnet::ImuSample`, `floodnet::DiagCounters`, `floodnet::SensorRecord`. The `native` environment, runnable as `pio test -e native`.

- [ ] **Step 1: Create `.gitignore`**

```gitignore
.pio/
.vscode/
__pycache__/
*.pyc
.DS_Store
```

- [ ] **Step 2: Create `platformio.ini`**

`build_src_filter` is what keeps the two entry points from colliding: the node build excludes the gateway's `main`, and the gateway build excludes the node's.

```ini
[platformio]
default_envs = node_polling

[env]
build_flags = -std=gnu++17 -Wall -Wextra

[teensy_base]
platform = teensy
board = teensy41
framework = arduino
monitor_speed = 115200
upload_protocol = teensy-cli
lib_deps =
    adafruit/Adafruit BNO055@^1.6.3
    mikem/RadioHead@^1.120

[env:node_polling]
extends = teensy_base
build_src_filter = +<*> -<gateway_main.cpp>
build_flags = ${env.build_flags} -D FLOODNET_SAMPLER_POLLING

[env:gateway]
extends = teensy_base
build_src_filter = +<*> -<main.cpp> -<sampler_polling.cpp>

[env:native]
platform = native
lib_compat_mode = off
lib_deps =
    floodnet_core
    floodnet_hal
```

- [ ] **Step 3: Create the two library manifests**

`lib/floodnet_core/library.json`:

```json
{
  "name": "floodnet_core",
  "version": "0.1.0",
  "description": "Platform-independent FloodNet logic: parsing, packing, relay",
  "build": { "flags": ["-I include"] }
}
```

`lib/floodnet_hal/library.json`:

```json
{
  "name": "floodnet_hal",
  "version": "0.1.0",
  "description": "Hardware abstraction interfaces for FloodNet",
  "build": { "flags": ["-I include"] }
}
```

- [ ] **Step 4: Write the failing test**

`test/test_sample/test_main.cpp`:

```cpp
#include <unity.h>
#include <floodnet/sample.hpp>

using namespace floodnet;

void setUp(void) {}
void tearDown(void) {}

void test_sensor_record_defaults_are_invalid(void) {
    SensorRecord record;
    TEST_ASSERT_FALSE(record.gps.valid);
    TEST_ASSERT_FALSE(record.imu.valid);
    TEST_ASSERT_EQUAL_UINT16(0, record.diag.drops);
    TEST_ASSERT_EQUAL_UINT16(0, record.diag.crc_errors);
}

int main(int, char **) {
    UNITY_BEGIN();
    RUN_TEST(test_sensor_record_defaults_are_invalid);
    return UNITY_END();
}
```

- [ ] **Step 5: Run the test to verify it fails**

Run: `pio test -e native -f test_sample`
Expected: FAIL. The compiler reports that `floodnet/sample.hpp` does not exist.

- [ ] **Step 6: Write the minimal implementation**

`lib/floodnet_core/include/floodnet/sample.hpp`:

```cpp
#ifndef FLOODNET_SAMPLE_HPP
#define FLOODNET_SAMPLE_HPP

#include <stdint.h>

namespace floodnet {

/// A single GPS position fix, timestamped with the node's local clock.
struct GpsFix {
    uint32_t time_ms = 0;
    int32_t lat_1e7 = 0;   ///< degrees x 1e7
    int32_t lon_1e7 = 0;   ///< degrees x 1e7
    int32_t alt_mm = 0;    ///< millimetres above mean sea level
    uint8_t satellites = 0;
    bool valid = false;
};

/// A single IMU orientation sample, timestamped with the node's local clock.
struct ImuSample {
    uint32_t time_ms = 0;
    int16_t yaw_cd = 0;    ///< centidegrees
    int16_t pitch_cd = 0;  ///< centidegrees
    int16_t roll_cd = 0;   ///< centidegrees
    bool valid = false;
};

/// Health counters carried in-band so a receiver can see loss at the source.
struct DiagCounters {
    uint16_t drops = 0;        ///< sensor bytes lost before the firmware read them
    uint16_t crc_errors = 0;   ///< packets received with a bad CRC
};

/// One paired observation, the unit this network transports.
struct SensorRecord {
    GpsFix gps;
    ImuSample imu;
    DiagCounters diag;
};

}  // namespace floodnet

#endif  // FLOODNET_SAMPLE_HPP
```

- [ ] **Step 7: Run the test to verify it passes**

Run: `pio test -e native -f test_sample`
Expected: PASS, `1 Tests 0 Failures 0 Ignored`.

- [ ] **Step 8: Create the CI workflow**

`.github/workflows/ci.yml`:

```yaml
name: CI

on:
  push:
  pull_request:

jobs:
  test:
    name: Native unit tests
    runs-on: ubuntu-latest
    steps:
      - uses: actions/checkout@v4
      - uses: actions/setup-python@v5
        with:
          python-version: '3.12'
      - name: Install PlatformIO
        run: pip install platformio
      - name: Run unit tests
        run: pio test -e native

  build:
    name: Firmware build
    runs-on: ubuntu-latest
    steps:
      - uses: actions/checkout@v4
      - uses: actions/setup-python@v5
        with:
          python-version: '3.12'
      - name: Install PlatformIO
        run: pip install platformio
      - name: Build node and gateway
        run: pio run -e node_polling -e gateway
```

- [ ] **Step 9: Commit**

```bash
git add .gitignore platformio.ini lib/ test/ .github/
git commit -m "chore: scaffold PlatformIO project with native test environment"
```

---

### Task 2: CRC16 and packet codec

The wire format.
Every later task depends on these types, so it comes first.

**Files:**
- Create: `lib/floodnet_core/include/floodnet/packet.hpp`
- Create: `lib/floodnet_core/src/packet.cpp`
- Test: `test/test_packet/test_main.cpp`

**Interfaces:**
- Consumes: `floodnet::SensorRecord` from Task 1.
- Produces:
  - `floodnet::Packet` with fields `node_id` (`uint16_t`), `seq` (`uint32_t`), `ttl` (`uint8_t`), `flags` (`uint8_t`), `record` (`SensorRecord`).
  - `uint16_t floodnet::crc16_ccitt(const uint8_t* data, size_t len)`
  - `size_t floodnet::encode_packet(const Packet& p, uint8_t* out, size_t out_len)` returning bytes written, or `0` on failure.
  - `bool floodnet::decode_packet(const uint8_t* in, size_t len, Packet* out)`
  - `floodnet::PACKET_SIZE` == 45.

The byte layout, which the gateway and the host pipeline both depend on:

| Offset | Size | Field |
|---|---|---|
| 0 | 1 | magic `0xFD` |
| 1 | 1 | version `0x01` |
| 2 | 2 | `node_id` |
| 4 | 4 | `seq` |
| 8 | 1 | `ttl` |
| 9 | 1 | `flags` |
| 10 | 4 | `gps.time_ms` |
| 14 | 4 | `gps.lat_1e7` |
| 18 | 4 | `gps.lon_1e7` |
| 22 | 4 | `gps.alt_mm` |
| 26 | 1 | `gps.satellites` |
| 27 | 1 | `gps.valid` |
| 28 | 4 | `imu.time_ms` |
| 32 | 2 | `imu.yaw_cd` |
| 34 | 2 | `imu.pitch_cd` |
| 36 | 2 | `imu.roll_cd` |
| 38 | 1 | `imu.valid` |
| 39 | 2 | `diag.drops` |
| 41 | 2 | `diag.crc_errors` |
| 43 | 2 | CRC16 over bytes 0 to 42 |

- [ ] **Step 1: Write the failing test**

`test/test_packet/test_main.cpp`:

```cpp
#include <string.h>
#include <unity.h>
#include <floodnet/packet.hpp>

using namespace floodnet;

void setUp(void) {}
void tearDown(void) {}

static Packet make_packet(void) {
    Packet p;
    p.node_id = 0x1234;
    p.seq = 0xDEADBEEF;
    p.ttl = 3;
    p.flags = 0;
    p.record.gps.time_ms = 1000;
    p.record.gps.lat_1e7 = 481173000;
    p.record.gps.lon_1e7 = -115166667;
    p.record.gps.alt_mm = 545400;
    p.record.gps.satellites = 8;
    p.record.gps.valid = true;
    p.record.imu.time_ms = 995;
    p.record.imu.yaw_cd = -4500;
    p.record.imu.pitch_cd = 250;
    p.record.imu.roll_cd = 0;
    p.record.imu.valid = true;
    p.record.diag.drops = 7;
    p.record.diag.crc_errors = 2;
    return p;
}

void test_crc16_matches_known_vector(void) {
    // CRC16-CCITT-FALSE of "123456789" is 0x29B1.
    const uint8_t input[] = {'1', '2', '3', '4', '5', '6', '7', '8', '9'};
    TEST_ASSERT_EQUAL_HEX16(0x29B1, crc16_ccitt(input, sizeof(input)));
}

void test_encode_returns_packet_size(void) {
    uint8_t buf[PACKET_SIZE];
    TEST_ASSERT_EQUAL_UINT(PACKET_SIZE, encode_packet(make_packet(), buf, sizeof(buf)));
}

void test_encode_rejects_short_buffer(void) {
    uint8_t buf[PACKET_SIZE - 1];
    TEST_ASSERT_EQUAL_UINT(0, encode_packet(make_packet(), buf, sizeof(buf)));
}

void test_round_trip_preserves_every_field(void) {
    uint8_t buf[PACKET_SIZE];
    Packet original = make_packet();
    TEST_ASSERT_EQUAL_UINT(PACKET_SIZE, encode_packet(original, buf, sizeof(buf)));

    Packet decoded;
    TEST_ASSERT_TRUE(decode_packet(buf, sizeof(buf), &decoded));

    TEST_ASSERT_EQUAL_UINT16(original.node_id, decoded.node_id);
    TEST_ASSERT_EQUAL_UINT32(original.seq, decoded.seq);
    TEST_ASSERT_EQUAL_UINT8(original.ttl, decoded.ttl);
    TEST_ASSERT_EQUAL_UINT32(original.record.gps.time_ms, decoded.record.gps.time_ms);
    TEST_ASSERT_EQUAL_INT32(original.record.gps.lat_1e7, decoded.record.gps.lat_1e7);
    TEST_ASSERT_EQUAL_INT32(original.record.gps.lon_1e7, decoded.record.gps.lon_1e7);
    TEST_ASSERT_EQUAL_INT32(original.record.gps.alt_mm, decoded.record.gps.alt_mm);
    TEST_ASSERT_EQUAL_UINT8(original.record.gps.satellites, decoded.record.gps.satellites);
    TEST_ASSERT_TRUE(decoded.record.gps.valid);
    TEST_ASSERT_EQUAL_UINT32(original.record.imu.time_ms, decoded.record.imu.time_ms);
    TEST_ASSERT_EQUAL_INT16(original.record.imu.yaw_cd, decoded.record.imu.yaw_cd);
    TEST_ASSERT_EQUAL_INT16(original.record.imu.pitch_cd, decoded.record.imu.pitch_cd);
    TEST_ASSERT_TRUE(decoded.record.imu.valid);
    TEST_ASSERT_EQUAL_UINT16(original.record.diag.drops, decoded.record.diag.drops);
    TEST_ASSERT_EQUAL_UINT16(original.record.diag.crc_errors, decoded.record.diag.crc_errors);
}

void test_decode_rejects_corrupted_payload(void) {
    uint8_t buf[PACKET_SIZE];
    encode_packet(make_packet(), buf, sizeof(buf));
    buf[20] ^= 0xFF;  // flip bits inside the longitude field

    Packet decoded;
    TEST_ASSERT_FALSE(decode_packet(buf, sizeof(buf), &decoded));
}

void test_decode_rejects_bad_magic(void) {
    uint8_t buf[PACKET_SIZE];
    encode_packet(make_packet(), buf, sizeof(buf));
    buf[0] = 0x00;

    Packet decoded;
    TEST_ASSERT_FALSE(decode_packet(buf, sizeof(buf), &decoded));
}

void test_decode_rejects_wrong_length(void) {
    uint8_t buf[PACKET_SIZE];
    encode_packet(make_packet(), buf, sizeof(buf));

    Packet decoded;
    TEST_ASSERT_FALSE(decode_packet(buf, PACKET_SIZE - 1, &decoded));
}

int main(int, char **) {
    UNITY_BEGIN();
    RUN_TEST(test_crc16_matches_known_vector);
    RUN_TEST(test_encode_returns_packet_size);
    RUN_TEST(test_encode_rejects_short_buffer);
    RUN_TEST(test_round_trip_preserves_every_field);
    RUN_TEST(test_decode_rejects_corrupted_payload);
    RUN_TEST(test_decode_rejects_bad_magic);
    RUN_TEST(test_decode_rejects_wrong_length);
    return UNITY_END();
}
```

- [ ] **Step 2: Run the test to verify it fails**

Run: `pio test -e native -f test_packet`
Expected: FAIL. The compiler reports that `floodnet/packet.hpp` does not exist.

- [ ] **Step 3: Write the header**

`lib/floodnet_core/include/floodnet/packet.hpp`:

```cpp
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
```

- [ ] **Step 4: Write the implementation**

`lib/floodnet_core/src/packet.cpp`:

```cpp
#include <floodnet/packet.hpp>

namespace floodnet {
namespace {

void put_u16(uint8_t *p, uint16_t v) {
    p[0] = static_cast<uint8_t>(v & 0xFF);
    p[1] = static_cast<uint8_t>((v >> 8) & 0xFF);
}

void put_u32(uint8_t *p, uint32_t v) {
    p[0] = static_cast<uint8_t>(v & 0xFF);
    p[1] = static_cast<uint8_t>((v >> 8) & 0xFF);
    p[2] = static_cast<uint8_t>((v >> 16) & 0xFF);
    p[3] = static_cast<uint8_t>((v >> 24) & 0xFF);
}

uint16_t get_u16(const uint8_t *p) {
    return static_cast<uint16_t>(p[0] | (static_cast<uint16_t>(p[1]) << 8));
}

uint32_t get_u32(const uint8_t *p) {
    return static_cast<uint32_t>(p[0]) | (static_cast<uint32_t>(p[1]) << 8) |
           (static_cast<uint32_t>(p[2]) << 16) | (static_cast<uint32_t>(p[3]) << 24);
}

}  // namespace

uint16_t crc16_ccitt(const uint8_t *data, size_t len) {
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < len; ++i) {
        crc ^= static_cast<uint16_t>(data[i]) << 8;
        for (int bit = 0; bit < 8; ++bit) {
            if (crc & 0x8000) {
                crc = static_cast<uint16_t>((crc << 1) ^ 0x1021);
            } else {
                crc = static_cast<uint16_t>(crc << 1);
            }
        }
    }
    return crc;
}

size_t encode_packet(const Packet &p, uint8_t *out, size_t out_len) {
    if (out == nullptr || out_len < PACKET_SIZE) {
        return 0;
    }

    out[0] = PACKET_MAGIC;
    out[1] = PACKET_VERSION;
    put_u16(&out[2], p.node_id);
    put_u32(&out[4], p.seq);
    out[8] = p.ttl;
    out[9] = p.flags;

    put_u32(&out[10], p.record.gps.time_ms);
    put_u32(&out[14], static_cast<uint32_t>(p.record.gps.lat_1e7));
    put_u32(&out[18], static_cast<uint32_t>(p.record.gps.lon_1e7));
    put_u32(&out[22], static_cast<uint32_t>(p.record.gps.alt_mm));
    out[26] = p.record.gps.satellites;
    out[27] = p.record.gps.valid ? 1 : 0;

    put_u32(&out[28], p.record.imu.time_ms);
    put_u16(&out[32], static_cast<uint16_t>(p.record.imu.yaw_cd));
    put_u16(&out[34], static_cast<uint16_t>(p.record.imu.pitch_cd));
    put_u16(&out[36], static_cast<uint16_t>(p.record.imu.roll_cd));
    out[38] = p.record.imu.valid ? 1 : 0;

    put_u16(&out[39], p.record.diag.drops);
    put_u16(&out[41], p.record.diag.crc_errors);

    put_u16(&out[43], crc16_ccitt(out, 43));
    return PACKET_SIZE;
}

bool decode_packet(const uint8_t *in, size_t len, Packet *out) {
    if (in == nullptr || out == nullptr || len != PACKET_SIZE) {
        return false;
    }
    if (in[0] != PACKET_MAGIC || in[1] != PACKET_VERSION) {
        return false;
    }
    if (get_u16(&in[43]) != crc16_ccitt(in, 43)) {
        return false;
    }

    Packet p;
    p.node_id = get_u16(&in[2]);
    p.seq = get_u32(&in[4]);
    p.ttl = in[8];
    p.flags = in[9];

    p.record.gps.time_ms = get_u32(&in[10]);
    p.record.gps.lat_1e7 = static_cast<int32_t>(get_u32(&in[14]));
    p.record.gps.lon_1e7 = static_cast<int32_t>(get_u32(&in[18]));
    p.record.gps.alt_mm = static_cast<int32_t>(get_u32(&in[22]));
    p.record.gps.satellites = in[26];
    p.record.gps.valid = in[27] != 0;

    p.record.imu.time_ms = get_u32(&in[28]);
    p.record.imu.yaw_cd = static_cast<int16_t>(get_u16(&in[32]));
    p.record.imu.pitch_cd = static_cast<int16_t>(get_u16(&in[34]));
    p.record.imu.roll_cd = static_cast<int16_t>(get_u16(&in[36]));
    p.record.imu.valid = in[38] != 0;

    p.record.diag.drops = get_u16(&in[39]);
    p.record.diag.crc_errors = get_u16(&in[41]);

    *out = p;
    return true;
}

}  // namespace floodnet
```

- [ ] **Step 5: Run the tests to verify they pass**

Run: `pio test -e native -f test_packet`
Expected: PASS, `7 Tests 0 Failures 0 Ignored`.

- [ ] **Step 6: Commit**

```bash
git add lib/floodnet_core/include/floodnet/packet.hpp lib/floodnet_core/src/packet.cpp test/test_packet/
git commit -m "feat: add CRC16 packet codec with fixed 45-byte wire format"
```

---

### Task 3: NMEA GGA parser

Turns raw GPS bytes into a `GpsFix`.
Expressed as a pure function over a character span so it is trivially testable and has no I/O.

**Files:**
- Create: `lib/floodnet_core/include/floodnet/nmea.hpp`
- Create: `lib/floodnet_core/src/nmea.cpp`
- Test: `test/test_nmea/test_main.cpp`

**Interfaces:**
- Consumes: `floodnet::GpsFix` from Task 1.
- Produces:
  - `bool floodnet::nmea_checksum_ok(const char* sentence, size_t len)`
  - `bool floodnet::parse_gga(const char* sentence, size_t len, uint32_t time_ms, GpsFix* out)`
  - `floodnet::NMEA_MAX_SENTENCE` == 82.

Coordinate conversion uses `double` and `llround`. Teensy 4.1 has a hardware FPU, so this is cheap here. On a smaller part it would be fixed-point arithmetic instead.

- [ ] **Step 1: Write the failing test**

`test/test_nmea/test_main.cpp`:

```cpp
#include <string.h>
#include <unity.h>
#include <floodnet/nmea.hpp>

using namespace floodnet;

void setUp(void) {}
void tearDown(void) {}

static const char kValidGga[] =
    "$GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,*47";

void test_checksum_accepts_valid_sentence(void) {
    TEST_ASSERT_TRUE(nmea_checksum_ok(kValidGga, strlen(kValidGga)));
}

void test_checksum_rejects_tampered_sentence(void) {
    char bad[128];
    strcpy(bad, kValidGga);
    bad[10] = '9';  // change a payload digit, leave the stated checksum alone
    TEST_ASSERT_FALSE(nmea_checksum_ok(bad, strlen(bad)));
}

void test_parse_extracts_position(void) {
    GpsFix fix;
    TEST_ASSERT_TRUE(parse_gga(kValidGga, strlen(kValidGga), 4242, &fix));

    TEST_ASSERT_TRUE(fix.valid);
    TEST_ASSERT_EQUAL_UINT32(4242, fix.time_ms);
    TEST_ASSERT_EQUAL_INT32(481173000, fix.lat_1e7);   // 48 deg 07.038 min N
    TEST_ASSERT_EQUAL_INT32(115166667, fix.lon_1e7);   // 11 deg 31.000 min E
    TEST_ASSERT_EQUAL_INT32(545400, fix.alt_mm);
    TEST_ASSERT_EQUAL_UINT8(8, fix.satellites);
}

void test_parse_applies_southern_and_western_signs(void) {
    const char sentence[] =
        "$GPGGA,123519,4807.038,S,01131.000,W,1,08,0.9,545.4,M,46.9,M,,*48";
    GpsFix fix;
    TEST_ASSERT_TRUE(parse_gga(sentence, strlen(sentence), 0, &fix));
    TEST_ASSERT_EQUAL_INT32(-481173000, fix.lat_1e7);
    TEST_ASSERT_EQUAL_INT32(-115166667, fix.lon_1e7);
}

void test_parse_marks_no_fix_invalid(void) {
    // Fix quality field is 0, meaning the receiver has no position solution.
    const char sentence[] = "$GPGGA,123519,4807.038,N,01131.000,E,0,00,,,M,,M,,*52";
    GpsFix fix;
    TEST_ASSERT_TRUE(parse_gga(sentence, strlen(sentence), 0, &fix));
    TEST_ASSERT_FALSE(fix.valid);
}

void test_parse_rejects_wrong_sentence_type(void) {
    const char sentence[] = "$GPRMC,123519,A,4807.038,N,01131.000,E,022.4,084.4,230394,003.1,W*6A";
    GpsFix fix;
    TEST_ASSERT_FALSE(parse_gga(sentence, strlen(sentence), 0, &fix));
}

void test_parse_rejects_truncated_sentence(void) {
    const char sentence[] = "$GPGGA,123519,4807.0";
    GpsFix fix;
    TEST_ASSERT_FALSE(parse_gga(sentence, strlen(sentence), 0, &fix));
}

void test_parse_rejects_oversized_sentence(void) {
    char oversized[NMEA_MAX_SENTENCE + 10];
    memset(oversized, 'A', sizeof(oversized));
    oversized[0] = '$';
    GpsFix fix;
    TEST_ASSERT_FALSE(parse_gga(oversized, sizeof(oversized), 0, &fix));
}

int main(int, char **) {
    UNITY_BEGIN();
    RUN_TEST(test_checksum_accepts_valid_sentence);
    RUN_TEST(test_checksum_rejects_tampered_sentence);
    RUN_TEST(test_parse_extracts_position);
    RUN_TEST(test_parse_applies_southern_and_western_signs);
    RUN_TEST(test_parse_marks_no_fix_invalid);
    RUN_TEST(test_parse_rejects_wrong_sentence_type);
    RUN_TEST(test_parse_rejects_truncated_sentence);
    RUN_TEST(test_parse_rejects_oversized_sentence);
    return UNITY_END();
}
```

All checksums in these sentences were computed and verified when this plan was written.
Step 4 shows how to regenerate them if a payload is edited, since changing any character changes the XOR.

- [ ] **Step 2: Run the test to verify it fails**

Run: `pio test -e native -f test_nmea`
Expected: FAIL. The compiler reports that `floodnet/nmea.hpp` does not exist.

- [ ] **Step 3: Write the header**

`lib/floodnet_core/include/floodnet/nmea.hpp`:

```cpp
#ifndef FLOODNET_NMEA_HPP
#define FLOODNET_NMEA_HPP

#include <stddef.h>
#include <stdint.h>

#include <floodnet/sample.hpp>

namespace floodnet {

/// NMEA 0183 caps a sentence at 82 characters including delimiters.
const size_t NMEA_MAX_SENTENCE = 82;

/// Verifies the trailing "*hh" against an XOR of everything between '$' and '*'.
bool nmea_checksum_ok(const char *sentence, size_t len);

/// Parses a GGA sentence into `*out`, stamping it with `time_ms`.
/// Returns false if the sentence is malformed, oversized, not GGA, or fails checksum.
/// A well-formed sentence reporting no fix returns true with `out->valid == false`.
bool parse_gga(const char *sentence, size_t len, uint32_t time_ms, GpsFix *out);

}  // namespace floodnet

#endif  // FLOODNET_NMEA_HPP
```

- [ ] **Step 4: Confirm the synthetic sentence checksums**

The expected output is `*48` and `*52`, matching the literals already in the test file.
Rerun this if you ever edit a sentence payload:

```bash
python3 - <<'PY'
bodies = [
    "GPGGA,123519,4807.038,S,01131.000,W,1,08,0.9,545.4,M,46.9,M,,",
    "GPGGA,123519,4807.038,N,01131.000,E,0,00,,,M,,M,,",
]
for b in bodies:
    c = 0
    for ch in b:
        c ^= ord(ch)
    print(f"${b}*{c:02X}")
PY
```

Expected: the two sentences printed, ending in `*48` and `*52` respectively. If they differ from the literals in the test file, the test file is what needs correcting.

- [ ] **Step 5: Write the implementation**

`lib/floodnet_core/src/nmea.cpp`:

```cpp
#include <floodnet/nmea.hpp>

#include <math.h>
#include <stdlib.h>
#include <string.h>

namespace floodnet {
namespace {

/// Copies field `index` (comma separated, field 0 is the talker id) into `buf`.
/// Returns false if the field does not exist or does not fit.
bool field(const char *sentence, size_t len, int index, char *buf, size_t buf_len) {
    size_t start = 1;  // skip the leading '$'
    int current = 0;

    for (size_t i = 1; i <= len; ++i) {
        const bool at_end = (i == len) || (sentence[i] == '*');
        if (i == len || sentence[i] == ',' || at_end) {
            if (current == index) {
                const size_t field_len = i - start;
                if (field_len >= buf_len) {
                    return false;
                }
                memcpy(buf, &sentence[start], field_len);
                buf[field_len] = '\0';
                return true;
            }
            if (at_end) {
                return false;
            }
            ++current;
            start = i + 1;
        }
    }
    return false;
}

/// Converts NMEA "ddmm.mmmm" (or "dddmm.mmmm") to degrees x 1e7.
/// `degree_digits` is 2 for latitude and 3 for longitude.
bool to_degrees_1e7(const char *value, int degree_digits, int32_t *out) {
    const size_t len = strlen(value);
    if (len < static_cast<size_t>(degree_digits) + 1) {
        return false;
    }

    char degree_buf[4];
    memcpy(degree_buf, value, static_cast<size_t>(degree_digits));
    degree_buf[degree_digits] = '\0';

    char *end = nullptr;
    const long degrees = strtol(degree_buf, &end, 10);
    if (end == degree_buf) {
        return false;
    }

    const double minutes = strtod(&value[degree_digits], &end);
    if (end == &value[degree_digits]) {
        return false;
    }

    const double total = static_cast<double>(degrees) + (minutes / 60.0);
    *out = static_cast<int32_t>(llround(total * 1e7));
    return true;
}

}  // namespace

bool nmea_checksum_ok(const char *sentence, size_t len) {
    if (sentence == nullptr || len < 4 || len > NMEA_MAX_SENTENCE || sentence[0] != '$') {
        return false;
    }

    size_t star = 0;
    for (size_t i = 1; i < len; ++i) {
        if (sentence[i] == '*') {
            star = i;
            break;
        }
    }
    if (star == 0 || star + 2 >= len + 1 || len < star + 3) {
        return false;
    }

    uint8_t computed = 0;
    for (size_t i = 1; i < star; ++i) {
        computed ^= static_cast<uint8_t>(sentence[i]);
    }

    char stated_buf[3] = {sentence[star + 1], sentence[star + 2], '\0'};
    char *end = nullptr;
    const long stated = strtol(stated_buf, &end, 16);
    if (end != stated_buf + 2) {
        return false;
    }

    return static_cast<uint8_t>(stated) == computed;
}

bool parse_gga(const char *sentence, size_t len, uint32_t time_ms, GpsFix *out) {
    if (out == nullptr || !nmea_checksum_ok(sentence, len)) {
        return false;
    }

    char buf[16];
    if (!field(sentence, len, 0, buf, sizeof(buf))) {
        return false;
    }
    // Accept any talker id: GPGGA, GNGGA, GLGGA all carry the same payload.
    if (strlen(buf) != 5 || strcmp(&buf[2], "GGA") != 0) {
        return false;
    }

    GpsFix fix;
    fix.time_ms = time_ms;

    if (!field(sentence, len, 6, buf, sizeof(buf))) {
        return false;
    }
    const long quality = strtol(buf, nullptr, 10);

    if (!field(sentence, len, 7, buf, sizeof(buf))) {
        return false;
    }
    fix.satellites = static_cast<uint8_t>(strtol(buf, nullptr, 10));

    if (quality == 0) {
        fix.valid = false;
        *out = fix;
        return true;
    }

    char hemisphere[4];
    if (!field(sentence, len, 2, buf, sizeof(buf)) ||
        !field(sentence, len, 3, hemisphere, sizeof(hemisphere)) ||
        !to_degrees_1e7(buf, 2, &fix.lat_1e7)) {
        return false;
    }
    if (hemisphere[0] == 'S') {
        fix.lat_1e7 = -fix.lat_1e7;
    }

    if (!field(sentence, len, 4, buf, sizeof(buf)) ||
        !field(sentence, len, 5, hemisphere, sizeof(hemisphere)) ||
        !to_degrees_1e7(buf, 3, &fix.lon_1e7)) {
        return false;
    }
    if (hemisphere[0] == 'W') {
        fix.lon_1e7 = -fix.lon_1e7;
    }

    if (!field(sentence, len, 9, buf, sizeof(buf))) {
        return false;
    }
    fix.alt_mm = static_cast<int32_t>(llround(strtod(buf, nullptr) * 1000.0));

    fix.valid = true;
    *out = fix;
    return true;
}

}  // namespace floodnet
```

- [ ] **Step 6: Run the tests to verify they pass**

Run: `pio test -e native -f test_nmea`
Expected: PASS, `8 Tests 0 Failures 0 Ignored`.

- [ ] **Step 7: Commit**

```bash
git add lib/floodnet_core/include/floodnet/nmea.hpp lib/floodnet_core/src/nmea.cpp test/test_nmea/
git commit -m "feat: add NMEA GGA parser with checksum validation"
```

---

### Task 4: GPS and IMU sample pairing

GPS fixes and IMU samples arrive on independent schedules.
This decides which IMU sample belongs with a given fix, and refuses to pair samples too far apart in time.

**Files:**
- Create: `lib/floodnet_core/include/floodnet/pairing.hpp`
- Create: `lib/floodnet_core/src/pairing.cpp`
- Test: `test/test_pairing/test_main.cpp`

**Interfaces:**
- Consumes: `floodnet::GpsFix`, `floodnet::ImuSample`, `floodnet::DiagCounters`, `floodnet::SensorRecord` from Task 1.
- Produces: `floodnet::SamplePairer` with
  - `explicit SamplePairer(uint32_t max_skew_ms)`
  - `void submit_imu(const ImuSample& sample)`
  - `SensorRecord pair(const GpsFix& fix, const DiagCounters& diag) const`

- [ ] **Step 1: Write the failing test**

`test/test_pairing/test_main.cpp`:

```cpp
#include <unity.h>
#include <floodnet/pairing.hpp>

using namespace floodnet;

void setUp(void) {}
void tearDown(void) {}

static ImuSample imu_at(uint32_t time_ms) {
    ImuSample s;
    s.time_ms = time_ms;
    s.yaw_cd = 1234;
    s.valid = true;
    return s;
}

static GpsFix fix_at(uint32_t time_ms) {
    GpsFix f;
    f.time_ms = time_ms;
    f.valid = true;
    return f;
}

void test_pairs_imu_within_skew(void) {
    SamplePairer pairer(50);
    pairer.submit_imu(imu_at(1000));

    const SensorRecord record = pairer.pair(fix_at(1030), DiagCounters());
    TEST_ASSERT_TRUE(record.imu.valid);
    TEST_ASSERT_EQUAL_INT16(1234, record.imu.yaw_cd);
}

void test_rejects_imu_beyond_skew(void) {
    SamplePairer pairer(50);
    pairer.submit_imu(imu_at(1000));

    const SensorRecord record = pairer.pair(fix_at(1100), DiagCounters());
    TEST_ASSERT_FALSE(record.imu.valid);
    TEST_ASSERT_TRUE(record.gps.valid);  // the fix itself is still good
}

void test_skew_is_symmetric(void) {
    SamplePairer pairer(50);
    pairer.submit_imu(imu_at(1100));

    // IMU ahead of the fix by more than the skew budget must also be rejected.
    const SensorRecord record = pairer.pair(fix_at(1000), DiagCounters());
    TEST_ASSERT_FALSE(record.imu.valid);
}

void test_no_imu_submitted_yields_invalid_imu(void) {
    SamplePairer pairer(50);
    const SensorRecord record = pairer.pair(fix_at(1000), DiagCounters());
    TEST_ASSERT_FALSE(record.imu.valid);
}

void test_latest_imu_wins(void) {
    SamplePairer pairer(50);
    pairer.submit_imu(imu_at(1000));
    ImuSample newer = imu_at(1020);
    newer.yaw_cd = 999;
    pairer.submit_imu(newer);

    const SensorRecord record = pairer.pair(fix_at(1030), DiagCounters());
    TEST_ASSERT_EQUAL_INT16(999, record.imu.yaw_cd);
}

void test_diag_counters_pass_through(void) {
    SamplePairer pairer(50);
    DiagCounters diag;
    diag.drops = 11;
    diag.crc_errors = 3;

    const SensorRecord record = pairer.pair(fix_at(1000), diag);
    TEST_ASSERT_EQUAL_UINT16(11, record.diag.drops);
    TEST_ASSERT_EQUAL_UINT16(3, record.diag.crc_errors);
}

int main(int, char **) {
    UNITY_BEGIN();
    RUN_TEST(test_pairs_imu_within_skew);
    RUN_TEST(test_rejects_imu_beyond_skew);
    RUN_TEST(test_skew_is_symmetric);
    RUN_TEST(test_no_imu_submitted_yields_invalid_imu);
    RUN_TEST(test_latest_imu_wins);
    RUN_TEST(test_diag_counters_pass_through);
    return UNITY_END();
}
```

- [ ] **Step 2: Run the test to verify it fails**

Run: `pio test -e native -f test_pairing`
Expected: FAIL. The compiler reports that `floodnet/pairing.hpp` does not exist.

- [ ] **Step 3: Write the header**

`lib/floodnet_core/include/floodnet/pairing.hpp`:

```cpp
#ifndef FLOODNET_PAIRING_HPP
#define FLOODNET_PAIRING_HPP

#include <stdint.h>

#include <floodnet/sample.hpp>

namespace floodnet {

/// Holds the most recent IMU sample and attaches it to an arriving GPS fix,
/// provided the two are close enough in time to describe the same instant.
class SamplePairer {
  public:
    explicit SamplePairer(uint32_t max_skew_ms);

    void submit_imu(const ImuSample &sample);

    /// Builds a record from `fix`, attaching the held IMU sample when the gap
    /// between their timestamps is at most `max_skew_ms`. Otherwise the record
    /// carries an invalid IMU sample, which a consumer can detect and ignore.
    SensorRecord pair(const GpsFix &fix, const DiagCounters &diag) const;

  private:
    ImuSample latest_;
    uint32_t max_skew_ms_;
};

}  // namespace floodnet

#endif  // FLOODNET_PAIRING_HPP
```

- [ ] **Step 4: Write the implementation**

`lib/floodnet_core/src/pairing.cpp`:

```cpp
#include <floodnet/pairing.hpp>

namespace floodnet {

SamplePairer::SamplePairer(uint32_t max_skew_ms) : latest_(), max_skew_ms_(max_skew_ms) {}

void SamplePairer::submit_imu(const ImuSample &sample) { latest_ = sample; }

SensorRecord SamplePairer::pair(const GpsFix &fix, const DiagCounters &diag) const {
    SensorRecord record;
    record.gps = fix;
    record.diag = diag;

    if (latest_.valid) {
        // Unsigned subtraction, so order the operands rather than using abs().
        const uint32_t skew = (fix.time_ms > latest_.time_ms) ? (fix.time_ms - latest_.time_ms)
                                                              : (latest_.time_ms - fix.time_ms);
        if (skew <= max_skew_ms_) {
            record.imu = latest_;
        }
    }

    return record;
}

}  // namespace floodnet
```

- [ ] **Step 5: Run the tests to verify they pass**

Run: `pio test -e native -f test_pairing`
Expected: PASS, `6 Tests 0 Failures 0 Ignored`.

- [ ] **Step 6: Commit**

```bash
git add lib/floodnet_core/include/floodnet/pairing.hpp lib/floodnet_core/src/pairing.cpp test/test_pairing/
git commit -m "feat: pair GPS fixes with IMU samples inside a skew budget"
```

---

### Task 5: Mesh deduplication and TTL

Relay nodes rebroadcast what they hear.
Without suppression, a packet heard by three neighbours is rebroadcast three times, and the network floods itself.

**Files:**
- Create: `lib/floodnet_core/include/floodnet/mesh.hpp`
- Create: `lib/floodnet_core/src/mesh.cpp`
- Test: `test/test_mesh/test_main.cpp`

**Interfaces:**
- Consumes: `floodnet::Packet` from Task 2.
- Produces:
  - `floodnet::DedupTable` with `bool seen(uint16_t node_id, uint32_t seq)` returning whether the pair was already recorded, and recording it either way.
  - `floodnet::DEDUP_CAPACITY` == 32.
  - `bool floodnet::should_relay(DedupTable& table, const Packet& p)`
  - `bool floodnet::prepare_relay(Packet* p)` decrementing TTL, returning false when the packet has expired.

- [ ] **Step 1: Write the failing test**

`test/test_mesh/test_main.cpp`:

```cpp
#include <unity.h>
#include <floodnet/mesh.hpp>

using namespace floodnet;

void setUp(void) {}
void tearDown(void) {}

static Packet packet(uint16_t node_id, uint32_t seq, uint8_t ttl) {
    Packet p;
    p.node_id = node_id;
    p.seq = seq;
    p.ttl = ttl;
    return p;
}

void test_first_sighting_relays(void) {
    DedupTable table;
    TEST_ASSERT_TRUE(should_relay(table, packet(1, 100, 3)));
}

void test_duplicate_does_not_relay(void) {
    DedupTable table;
    should_relay(table, packet(1, 100, 3));
    TEST_ASSERT_FALSE(should_relay(table, packet(1, 100, 3)));
}

void test_same_sequence_from_another_node_relays(void) {
    DedupTable table;
    should_relay(table, packet(1, 100, 3));
    TEST_ASSERT_TRUE(should_relay(table, packet(2, 100, 3)));
}

void test_expired_packet_does_not_relay(void) {
    DedupTable table;
    TEST_ASSERT_FALSE(should_relay(table, packet(1, 100, 0)));
}

void test_expired_packet_is_not_recorded(void) {
    // A TTL check that consumed a table slot would let an expired sighting
    // suppress the same packet arriving later by a shorter path.
    DedupTable table;
    should_relay(table, packet(1, 100, 0));
    TEST_ASSERT_TRUE(should_relay(table, packet(1, 100, 3)));
}

void test_table_evicts_oldest_entry(void) {
    DedupTable table;
    for (uint32_t seq = 0; seq < DEDUP_CAPACITY; ++seq) {
        should_relay(table, packet(1, seq, 3));
    }
    // One more sighting evicts sequence 0.
    should_relay(table, packet(1, DEDUP_CAPACITY, 3));
    TEST_ASSERT_TRUE(should_relay(table, packet(1, 0, 3)));
}

void test_prepare_relay_decrements_ttl(void) {
    Packet p = packet(1, 100, 3);
    TEST_ASSERT_TRUE(prepare_relay(&p));
    TEST_ASSERT_EQUAL_UINT8(2, p.ttl);
}

void test_prepare_relay_fails_on_last_hop(void) {
    Packet p = packet(1, 100, 1);
    TEST_ASSERT_FALSE(prepare_relay(&p));
    TEST_ASSERT_EQUAL_UINT8(0, p.ttl);
}

int main(int, char **) {
    UNITY_BEGIN();
    RUN_TEST(test_first_sighting_relays);
    RUN_TEST(test_duplicate_does_not_relay);
    RUN_TEST(test_same_sequence_from_another_node_relays);
    RUN_TEST(test_expired_packet_does_not_relay);
    RUN_TEST(test_expired_packet_is_not_recorded);
    RUN_TEST(test_table_evicts_oldest_entry);
    RUN_TEST(test_prepare_relay_decrements_ttl);
    RUN_TEST(test_prepare_relay_fails_on_last_hop);
    return UNITY_END();
}
```

- [ ] **Step 2: Run the test to verify it fails**

Run: `pio test -e native -f test_mesh`
Expected: FAIL. The compiler reports that `floodnet/mesh.hpp` does not exist.

- [ ] **Step 3: Write the header**

`lib/floodnet_core/include/floodnet/mesh.hpp`:

```cpp
#ifndef FLOODNET_MESH_HPP
#define FLOODNET_MESH_HPP

#include <stddef.h>
#include <stdint.h>

#include <floodnet/packet.hpp>

namespace floodnet {

/// Sightings retained per node. Fixed and small: nodes have no heap, and a
/// packet older than this many sightings is no longer worth suppressing.
const size_t DEDUP_CAPACITY = 32;

/// Remembers recently seen (node_id, seq) pairs, evicting the oldest first.
class DedupTable {
  public:
    DedupTable();

    /// Records the pair and reports whether it had already been recorded.
    bool seen(uint16_t node_id, uint32_t seq);

  private:
    struct Entry {
        uint16_t node_id;
        uint32_t seq;
        bool used;
    };

    Entry entries_[DEDUP_CAPACITY];
    size_t next_;
};

/// True when `p` still has hops left and has not been seen before.
/// An expired packet is rejected without consuming a table slot.
bool should_relay(DedupTable &table, const Packet &p);

/// Decrements `p->ttl`. Returns false once the packet has no hops remaining.
bool prepare_relay(Packet *p);

}  // namespace floodnet

#endif  // FLOODNET_MESH_HPP
```

- [ ] **Step 4: Write the implementation**

`lib/floodnet_core/src/mesh.cpp`:

```cpp
#include <floodnet/mesh.hpp>

namespace floodnet {

DedupTable::DedupTable() : entries_(), next_(0) {
    for (size_t i = 0; i < DEDUP_CAPACITY; ++i) {
        entries_[i].node_id = 0;
        entries_[i].seq = 0;
        entries_[i].used = false;
    }
}

bool DedupTable::seen(uint16_t node_id, uint32_t seq) {
    for (size_t i = 0; i < DEDUP_CAPACITY; ++i) {
        if (entries_[i].used && entries_[i].node_id == node_id && entries_[i].seq == seq) {
            return true;
        }
    }

    entries_[next_].node_id = node_id;
    entries_[next_].seq = seq;
    entries_[next_].used = true;
    next_ = (next_ + 1) % DEDUP_CAPACITY;
    return false;
}

bool should_relay(DedupTable &table, const Packet &p) {
    if (p.ttl == 0) {
        return false;
    }
    return !table.seen(p.node_id, p.seq);
}

bool prepare_relay(Packet *p) {
    if (p == nullptr || p->ttl == 0) {
        return false;
    }
    p->ttl = static_cast<uint8_t>(p->ttl - 1);
    return p->ttl > 0;
}

}  // namespace floodnet
```

- [ ] **Step 5: Run the tests to verify they pass**

Run: `pio test -e native -f test_mesh`
Expected: PASS, `8 Tests 0 Failures 0 Ignored`.

- [ ] **Step 6: Commit**

```bash
git add lib/floodnet_core/include/floodnet/mesh.hpp lib/floodnet_core/src/mesh.cpp test/test_mesh/
git commit -m "feat: suppress duplicate relays with a bounded sighting table"
```

---

### Task 6: HAL interfaces and simulation fakes

The seam between logic and hardware.
The fakes are what make the next task's data-loss behaviour observable rather than asserted.

**Files:**
- Create: `lib/floodnet_hal/include/floodnet/hal/clock.hpp`
- Create: `lib/floodnet_hal/include/floodnet/hal/gps.hpp`
- Create: `lib/floodnet_hal/include/floodnet/hal/imu.hpp`
- Create: `lib/floodnet_hal/include/floodnet/hal/radio.hpp`
- Create: `test/support/sim_clock.hpp`
- Create: `test/support/fake_gps.hpp`
- Create: `test/support/fake_imu.hpp`
- Create: `test/support/fake_radio.hpp`
- Test: `test/test_fakes/test_main.cpp`

**Interfaces:**
- Consumes: `floodnet::ImuSample` from Task 1.
- Produces:
  - `floodnet::IClock` with `uint32_t now_ms()` and `void delay_ms(uint32_t)`.
  - `floodnet::IGpsSource` with `int read_byte()` returning `-1` when empty, and `uint16_t bytes_dropped() const`.
  - `floodnet::IImuSource` with `bool read(ImuSample* out)`.
  - `floodnet::IRadio` with `bool transmit(const uint8_t*, size_t)` and `int receive(uint8_t*, size_t)`.
  - `floodnet::ISimTick` with `void on_tick(uint32_t elapsed_ms)`.
  - Test doubles `SimClock`, `FakeGps`, `FakeImu`, `FakeRadio`.

The design point: blocking HAL calls advance `SimClock`, which ticks `FakeGps`, which feeds bytes into a small hardware-style FIFO that overflows when nobody reads it.
Data loss emerges from the simulated timing rather than being hard-coded.

- [ ] **Step 1: Write the HAL interfaces**

`lib/floodnet_hal/include/floodnet/hal/clock.hpp`:

```cpp
#ifndef FLOODNET_HAL_CLOCK_HPP
#define FLOODNET_HAL_CLOCK_HPP

#include <stdint.h>

namespace floodnet {

class IClock {
  public:
    virtual ~IClock() {}
    virtual uint32_t now_ms() = 0;
    virtual void delay_ms(uint32_t ms) = 0;
};

}  // namespace floodnet

#endif  // FLOODNET_HAL_CLOCK_HPP
```

`lib/floodnet_hal/include/floodnet/hal/gps.hpp`:

```cpp
#ifndef FLOODNET_HAL_GPS_HPP
#define FLOODNET_HAL_GPS_HPP

#include <stdint.h>

namespace floodnet {

class IGpsSource {
  public:
    virtual ~IGpsSource() {}

    /// Returns the next buffered byte, or -1 when none is available.
    virtual int read_byte() = 0;

    /// Bytes the receive hardware discarded because its FIFO was full.
    virtual uint16_t bytes_dropped() const = 0;
};

}  // namespace floodnet

#endif  // FLOODNET_HAL_GPS_HPP
```

`lib/floodnet_hal/include/floodnet/hal/imu.hpp`:

```cpp
#ifndef FLOODNET_HAL_IMU_HPP
#define FLOODNET_HAL_IMU_HPP

#include <floodnet/sample.hpp>

namespace floodnet {

class IImuSource {
  public:
    virtual ~IImuSource() {}

    /// Blocking read of the current orientation. False if the sensor did not answer.
    virtual bool read(ImuSample *out) = 0;
};

}  // namespace floodnet

#endif  // FLOODNET_HAL_IMU_HPP
```

`lib/floodnet_hal/include/floodnet/hal/radio.hpp`:

```cpp
#ifndef FLOODNET_HAL_RADIO_HPP
#define FLOODNET_HAL_RADIO_HPP

#include <stddef.h>
#include <stdint.h>

namespace floodnet {

class IRadio {
  public:
    virtual ~IRadio() {}

    /// Blocking transmit. Returns false if the radio reported a failure.
    virtual bool transmit(const uint8_t *data, size_t len) = 0;

    /// Non-blocking receive. Returns bytes written to `buf`, or -1 if nothing waited.
    virtual int receive(uint8_t *buf, size_t len) = 0;
};

}  // namespace floodnet

#endif  // FLOODNET_HAL_RADIO_HPP
```

- [ ] **Step 2: Write the failing test**

`test/test_fakes/test_main.cpp`:

```cpp
#include <string.h>
#include <unity.h>

#include "../support/fake_gps.hpp"
#include "../support/fake_imu.hpp"
#include "../support/fake_radio.hpp"
#include "../support/sim_clock.hpp"

using namespace floodnet;

void setUp(void) {}
void tearDown(void) {}

void test_clock_advances_on_delay(void) {
    SimClock clock;
    TEST_ASSERT_EQUAL_UINT32(0, clock.now_ms());
    clock.delay_ms(25);
    TEST_ASSERT_EQUAL_UINT32(25, clock.now_ms());
}

void test_gps_delivers_bytes_as_time_passes(void) {
    SimClock clock;
    FakeGps gps("$GPGGA\r\n", 1.0);  // one byte per millisecond
    clock.add_observer(&gps);

    clock.delay_ms(3);
    TEST_ASSERT_EQUAL_INT('$', gps.read_byte());
    TEST_ASSERT_EQUAL_INT('G', gps.read_byte());
    TEST_ASSERT_EQUAL_INT('P', gps.read_byte());
    TEST_ASSERT_EQUAL_INT(-1, gps.read_byte());
}

void test_gps_drops_bytes_when_fifo_overflows(void) {
    SimClock clock;
    FakeGps gps("$GPGGA\r\n", 1.0);
    clock.add_observer(&gps);

    // FIFO holds 64 bytes; nobody reads during these 100 ms.
    clock.delay_ms(100);
    TEST_ASSERT_GREATER_THAN_UINT16(0, gps.bytes_dropped());
}

void test_imu_read_costs_time(void) {
    SimClock clock;
    FakeImu imu(clock, 10);

    ImuSample sample;
    TEST_ASSERT_TRUE(imu.read(&sample));
    TEST_ASSERT_TRUE(sample.valid);
    TEST_ASSERT_EQUAL_UINT32(10, clock.now_ms());
}

void test_radio_records_transmissions_and_costs_time(void) {
    SimClock clock;
    FakeRadio radio(clock, 60);

    const uint8_t payload[] = {1, 2, 3};
    TEST_ASSERT_TRUE(radio.transmit(payload, sizeof(payload)));
    TEST_ASSERT_EQUAL_UINT(1, radio.sent_count());
    TEST_ASSERT_EQUAL_UINT(3, radio.last_length());
    TEST_ASSERT_EQUAL_UINT8(2, radio.last_payload()[1]);
    TEST_ASSERT_EQUAL_UINT32(60, clock.now_ms());
}

int main(int, char **) {
    UNITY_BEGIN();
    RUN_TEST(test_clock_advances_on_delay);
    RUN_TEST(test_gps_delivers_bytes_as_time_passes);
    RUN_TEST(test_gps_drops_bytes_when_fifo_overflows);
    RUN_TEST(test_imu_read_costs_time);
    RUN_TEST(test_radio_records_transmissions_and_costs_time);
    return UNITY_END();
}
```

- [ ] **Step 3: Run the test to verify it fails**

Run: `pio test -e native -f test_fakes`
Expected: FAIL. The compiler reports that `../support/fake_gps.hpp` does not exist.

- [ ] **Step 4: Write the fakes**

`test/support/sim_clock.hpp`:

```cpp
#ifndef FLOODNET_TEST_SIM_CLOCK_HPP
#define FLOODNET_TEST_SIM_CLOCK_HPP

#include <stddef.h>
#include <stdint.h>

#include <floodnet/hal/clock.hpp>

namespace floodnet {

/// Anything that needs to advance when simulated time passes.
class ISimTick {
  public:
    virtual ~ISimTick() {}
    virtual void on_tick(uint32_t elapsed_ms) = 0;
};

/// A clock that only moves when something asks it to, notifying observers.
/// This is what lets a blocking HAL call have a visible cost elsewhere.
class SimClock : public IClock {
  public:
    SimClock() : now_ms_(0), observer_count_(0) {}

    void add_observer(ISimTick *observer) {
        if (observer_count_ < MAX_OBSERVERS) {
            observers_[observer_count_++] = observer;
        }
    }

    uint32_t now_ms() override { return now_ms_; }

    void delay_ms(uint32_t ms) override {
        now_ms_ += ms;
        for (size_t i = 0; i < observer_count_; ++i) {
            observers_[i]->on_tick(ms);
        }
    }

  private:
    static const size_t MAX_OBSERVERS = 4;
    uint32_t now_ms_;
    ISimTick *observers_[MAX_OBSERVERS];
    size_t observer_count_;
};

}  // namespace floodnet

#endif  // FLOODNET_TEST_SIM_CLOCK_HPP
```

`test/support/fake_gps.hpp`:

```cpp
#ifndef FLOODNET_TEST_FAKE_GPS_HPP
#define FLOODNET_TEST_FAKE_GPS_HPP

#include <string.h>

#include <floodnet/hal/gps.hpp>

#include "sim_clock.hpp"

namespace floodnet {

/// Emits a sentence repeatedly at a fixed byte rate into a small FIFO,
/// modelling a UART peripheral that discards bytes nobody collected.
class FakeGps : public IGpsSource, public ISimTick {
  public:
    FakeGps(const char *sentence, double bytes_per_ms)
        : sentence_(sentence),
          sentence_len_(strlen(sentence)),
          source_index_(0),
          bytes_per_ms_(bytes_per_ms),
          pending_(0.0),
          head_(0),
          tail_(0),
          count_(0),
          dropped_(0) {}

    void on_tick(uint32_t elapsed_ms) override {
        pending_ += static_cast<double>(elapsed_ms) * bytes_per_ms_;
        while (pending_ >= 1.0) {
            push(sentence_[source_index_]);
            source_index_ = (source_index_ + 1) % sentence_len_;
            pending_ -= 1.0;
        }
    }

    int read_byte() override {
        if (count_ == 0) {
            return -1;
        }
        const uint8_t value = fifo_[tail_];
        tail_ = (tail_ + 1) % FIFO_DEPTH;
        --count_;
        return static_cast<int>(value);
    }

    uint16_t bytes_dropped() const override { return dropped_; }

  private:
    /// Matches the 64-byte software receive buffer Teensy 4.x HardwareSerial
    /// keeps, which is the buffer `TeensyGps` watches for overflow. At 9600
    /// baud it holds roughly 66 ms of traffic, so a loop that stalls longer
    /// than that loses bytes.
    static const size_t FIFO_DEPTH = 64;

    void push(char value) {
        if (count_ == FIFO_DEPTH) {
            if (dropped_ < 0xFFFF) {
                ++dropped_;
            }
            return;
        }
        fifo_[head_] = static_cast<uint8_t>(value);
        head_ = (head_ + 1) % FIFO_DEPTH;
        ++count_;
    }

    const char *sentence_;
    size_t sentence_len_;
    size_t source_index_;
    double bytes_per_ms_;
    double pending_;

    uint8_t fifo_[FIFO_DEPTH];
    size_t head_;
    size_t tail_;
    size_t count_;
    uint16_t dropped_;
};

}  // namespace floodnet

#endif  // FLOODNET_TEST_FAKE_GPS_HPP
```

`test/support/fake_imu.hpp`:

```cpp
#ifndef FLOODNET_TEST_FAKE_IMU_HPP
#define FLOODNET_TEST_FAKE_IMU_HPP

#include <floodnet/hal/imu.hpp>

#include "sim_clock.hpp"

namespace floodnet {

/// Returns a fixed orientation, charging `read_cost_ms` of simulated time
/// the way a real blocking I2C transaction would.
class FakeImu : public IImuSource {
  public:
    FakeImu(SimClock &clock, uint32_t read_cost_ms)
        : clock_(clock), read_cost_ms_(read_cost_ms), reads_(0) {}

    bool read(ImuSample *out) override {
        clock_.delay_ms(read_cost_ms_);
        ++reads_;

        out->time_ms = clock_.now_ms();
        out->yaw_cd = 1500;
        out->pitch_cd = -200;
        out->roll_cd = 50;
        out->valid = true;
        return true;
    }

    size_t reads() const { return reads_; }

  private:
    SimClock &clock_;
    uint32_t read_cost_ms_;
    size_t reads_;
};

}  // namespace floodnet

#endif  // FLOODNET_TEST_FAKE_IMU_HPP
```

`test/support/fake_radio.hpp`:

```cpp
#ifndef FLOODNET_TEST_FAKE_RADIO_HPP
#define FLOODNET_TEST_FAKE_RADIO_HPP

#include <string.h>

#include <floodnet/hal/radio.hpp>

#include "sim_clock.hpp"

namespace floodnet {

/// Records what was transmitted and charges the simulated airtime.
class FakeRadio : public IRadio {
  public:
    FakeRadio(SimClock &clock, uint32_t tx_cost_ms)
        : clock_(clock), tx_cost_ms_(tx_cost_ms), sent_count_(0), last_length_(0) {
        memset(last_payload_, 0, sizeof(last_payload_));
    }

    bool transmit(const uint8_t *data, size_t len) override {
        if (len > sizeof(last_payload_)) {
            return false;
        }
        clock_.delay_ms(tx_cost_ms_);
        memcpy(last_payload_, data, len);
        last_length_ = len;
        ++sent_count_;
        return true;
    }

    int receive(uint8_t *, size_t) override { return -1; }

    size_t sent_count() const { return sent_count_; }
    size_t last_length() const { return last_length_; }
    const uint8_t *last_payload() const { return last_payload_; }

  private:
    static const size_t MAX_PAYLOAD = 64;

    SimClock &clock_;
    uint32_t tx_cost_ms_;
    size_t sent_count_;
    size_t last_length_;
    uint8_t last_payload_[MAX_PAYLOAD];
};

}  // namespace floodnet

#endif  // FLOODNET_TEST_FAKE_RADIO_HPP
```

- [ ] **Step 5: Run the tests to verify they pass**

Run: `pio test -e native -f test_fakes`
Expected: PASS, `5 Tests 0 Failures 0 Ignored`.

- [ ] **Step 6: Commit**

```bash
git add lib/floodnet_hal/ test/support/ test/test_fakes/
git commit -m "feat: add HAL interfaces and time-driven simulation fakes"
```

---

### Task 7: Polling sampler

The superloop body, written as a class so it can be tested on the host.
This is the acquisition strategy the whole repository later measures against.

**Files:**
- Create: `src/sampler_polling.hpp`
- Create: `src/sampler_polling.cpp`
- Modify: `platformio.ini` (add `src` to the native environment's include path)
- Test: `test/test_polling/test_main.cpp`

**Interfaces:**
- Consumes: everything from Tasks 1 to 6.
- Produces: `floodnet::PollingSampler` with
  - `PollingSampler(IGpsSource&, IImuSource&, IRadio&, IClock&, uint16_t node_id, uint8_t ttl)`
  - `void step()`
  - `uint32_t packets_sent() const`
  - `DiagCounters diag() const`

Each `step()` does, in order: drain whatever GPS bytes are sitting in the FIFO, read the IMU (blocking), and if a complete valid sentence arrived, pair, encode, and transmit (blocking).
Bytes arriving during the two blocking calls have nowhere to go, which is the defect this milestone captures honestly.

- [ ] **Step 1: Add the src include path to the native environment**

In `platformio.ini`, extend `[env:native]`:

```ini
[env:native]
platform = native
lib_compat_mode = off
build_flags = ${env.build_flags} -I src
lib_deps =
    floodnet_core
    floodnet_hal
```

- [ ] **Step 2: Write the failing test**

`test/test_polling/test_main.cpp`:

```cpp
#include <unity.h>

#include <floodnet/packet.hpp>

#include "../support/fake_gps.hpp"
#include "../support/fake_imu.hpp"
#include "../support/fake_radio.hpp"
#include "../support/sim_clock.hpp"
#include "sampler_polling.hpp"

using namespace floodnet;

void setUp(void) {}
void tearDown(void) {}

// 9600 baud, 8N1, is about 0.96 bytes per millisecond.
static const double kGpsByteRate = 0.96;
static const char kSentence[] =
    "$GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,*47\r\n";

// Two timing profiles, because they demonstrate two different things.
//
// FAST keeps every blocking call short enough that the 64-byte receive buffer
// never overruns, so sentences arrive intact. It proves the loop works.
//
// REALISTIC uses a 10 ms IMU read and 60 ms of LoRa airtime. Together those
// exceed the ~66 ms the buffer can hold at 9600 baud, so bytes are lost. It
// proves the loop's defect. Mixing the two into one profile would mean either
// no packets or no drops, and the milestone needs to show both.
static const uint32_t kFastImuMs = 2;
static const uint32_t kFastRadioMs = 5;
static const uint32_t kRealisticImuMs = 10;
static const uint32_t kRealisticRadioMs = 60;

void test_emits_a_decodable_packet(void) {
    SimClock clock;
    FakeGps gps(kSentence, kGpsByteRate);
    clock.add_observer(&gps);
    FakeImu imu(clock, kFastImuMs);
    FakeRadio radio(clock, kFastRadioMs);

    PollingSampler sampler(gps, imu, radio, clock, 0x0042, 3);
    for (int i = 0; i < 400; ++i) {
        sampler.step();
    }

    TEST_ASSERT_GREATER_THAN_UINT32(0, sampler.packets_sent());

    Packet decoded;
    TEST_ASSERT_TRUE(decode_packet(radio.last_payload(), radio.last_length(), &decoded));
    TEST_ASSERT_EQUAL_UINT16(0x0042, decoded.node_id);
    TEST_ASSERT_EQUAL_UINT8(3, decoded.ttl);
    TEST_ASSERT_TRUE(decoded.record.gps.valid);
    TEST_ASSERT_EQUAL_INT32(481173000, decoded.record.gps.lat_1e7);
}

void test_sequence_numbers_increment(void) {
    SimClock clock;
    FakeGps gps(kSentence, kGpsByteRate);
    clock.add_observer(&gps);
    FakeImu imu(clock, kFastImuMs);
    FakeRadio radio(clock, kFastRadioMs);

    PollingSampler sampler(gps, imu, radio, clock, 1, 3);
    for (int i = 0; i < 400; ++i) {
        sampler.step();
    }

    Packet decoded;
    TEST_ASSERT_TRUE(decode_packet(radio.last_payload(), radio.last_length(), &decoded));
    TEST_ASSERT_EQUAL_UINT32(sampler.packets_sent() - 1, decoded.seq);
}

void test_blocking_calls_lose_gps_bytes(void) {
    // This is the defect the interrupt-driven rewrite exists to remove.
    // Asserting it here means the later fix has something concrete to beat.
    SimClock clock;
    FakeGps gps(kSentence, kGpsByteRate);
    clock.add_observer(&gps);
    FakeImu imu(clock, kRealisticImuMs);
    FakeRadio radio(clock, kRealisticRadioMs);

    PollingSampler sampler(gps, imu, radio, clock, 1, 3);
    for (int i = 0; i < 400; ++i) {
        sampler.step();
    }

    TEST_ASSERT_GREATER_THAN_UINT16(0, sampler.diag().drops);
}

void test_fast_loop_loses_nothing(void) {
    // The mirror of the test above: the loss is a consequence of the stall,
    // not something inherent to polling, and this pins that down.
    SimClock clock;
    FakeGps gps(kSentence, kGpsByteRate);
    clock.add_observer(&gps);
    FakeImu imu(clock, kFastImuMs);
    FakeRadio radio(clock, kFastRadioMs);

    PollingSampler sampler(gps, imu, radio, clock, 1, 3);
    for (int i = 0; i < 400; ++i) {
        sampler.step();
    }

    TEST_ASSERT_EQUAL_UINT16(0, sampler.diag().drops);
}

void test_reports_drops_in_the_packet(void) {
    SimClock clock;
    FakeGps gps(kSentence, kGpsByteRate);
    clock.add_observer(&gps);
    FakeImu imu(clock, kRealisticImuMs);
    FakeRadio radio(clock, kRealisticRadioMs);

    PollingSampler sampler(gps, imu, radio, clock, 1, 3);
    for (int i = 0; i < 400; ++i) {
        sampler.step();
    }

    Packet decoded;
    TEST_ASSERT_TRUE(decode_packet(radio.last_payload(), radio.last_length(), &decoded));
    TEST_ASSERT_GREATER_THAN_UINT16(0, decoded.record.diag.drops);
}

void test_no_transmission_without_a_complete_sentence(void) {
    SimClock clock;
    FakeGps gps("garbage without a dollar sign", kGpsByteRate);
    clock.add_observer(&gps);
    FakeImu imu(clock, kFastImuMs);
    FakeRadio radio(clock, kFastRadioMs);

    PollingSampler sampler(gps, imu, radio, clock, 1, 3);
    for (int i = 0; i < 100; ++i) {
        sampler.step();
    }

    TEST_ASSERT_EQUAL_UINT32(0, sampler.packets_sent());
}

int main(int, char **) {
    UNITY_BEGIN();
    RUN_TEST(test_emits_a_decodable_packet);
    RUN_TEST(test_sequence_numbers_increment);
    RUN_TEST(test_blocking_calls_lose_gps_bytes);
    RUN_TEST(test_fast_loop_loses_nothing);
    RUN_TEST(test_reports_drops_in_the_packet);
    RUN_TEST(test_no_transmission_without_a_complete_sentence);
    return UNITY_END();
}
```

- [ ] **Step 3: Run the test to verify it fails**

Run: `pio test -e native -f test_polling`
Expected: FAIL. The compiler reports that `sampler_polling.hpp` does not exist.

- [ ] **Step 4: Write the header**

`src/sampler_polling.hpp`:

```cpp
#ifndef FLOODNET_SAMPLER_POLLING_HPP
#define FLOODNET_SAMPLER_POLLING_HPP

#include <stddef.h>
#include <stdint.h>

#include <floodnet/hal/clock.hpp>
#include <floodnet/hal/gps.hpp>
#include <floodnet/hal/imu.hpp>
#include <floodnet/hal/radio.hpp>
#include <floodnet/nmea.hpp>
#include <floodnet/packet.hpp>
#include <floodnet/pairing.hpp>

namespace floodnet {

/// Acquisition by polling: each iteration visits every sensor in turn and
/// blocks on it. Simple to follow, and it cannot collect from one peripheral
/// while waiting on another, so bytes arriving during a blocking call are lost.
class PollingSampler {
  public:
    PollingSampler(IGpsSource &gps, IImuSource &imu, IRadio &radio, IClock &clock,
                   uint16_t node_id, uint8_t ttl);

    /// One pass of the superloop.
    void step();

    uint32_t packets_sent() const { return packets_sent_; }
    DiagCounters diag() const { return diag_; }

  private:
    void collect_gps_bytes(bool *sentence_ready);

    IGpsSource &gps_;
    IImuSource &imu_;
    IRadio &radio_;
    IClock &clock_;

    SamplePairer pairer_;
    uint16_t node_id_;
    uint8_t ttl_;
    uint32_t seq_;
    uint32_t packets_sent_;
    DiagCounters diag_;

    /// The single shared line buffer. One assembly area for one sentence at a
    /// time, which is all a strictly sequential loop can make use of.
    char line_[NMEA_MAX_SENTENCE + 1];
    size_t line_len_;
};

}  // namespace floodnet

#endif  // FLOODNET_SAMPLER_POLLING_HPP
```

- [ ] **Step 5: Write the implementation**

`src/sampler_polling.cpp`:

```cpp
#include "sampler_polling.hpp"

namespace floodnet {

namespace {
/// Skew budget between a GPS fix and the IMU sample paired with it.
const uint32_t PAIRING_SKEW_MS = 250;
}  // namespace

PollingSampler::PollingSampler(IGpsSource &gps, IImuSource &imu, IRadio &radio, IClock &clock,
                               uint16_t node_id, uint8_t ttl)
    : gps_(gps),
      imu_(imu),
      radio_(radio),
      clock_(clock),
      pairer_(PAIRING_SKEW_MS),
      node_id_(node_id),
      ttl_(ttl),
      seq_(0),
      packets_sent_(0),
      diag_(),
      line_len_(0) {
    line_[0] = '\0';
}

void PollingSampler::collect_gps_bytes(bool *sentence_ready) {
    *sentence_ready = false;

    for (;;) {
        const int value = gps_.read_byte();
        if (value < 0) {
            return;
        }

        const char c = static_cast<char>(value);
        if (c == '$') {
            line_len_ = 0;
        }

        if (c == '\r' || c == '\n') {
            if (line_len_ > 0) {
                line_[line_len_] = '\0';
                *sentence_ready = true;
                return;
            }
            continue;
        }

        if (line_len_ < NMEA_MAX_SENTENCE) {
            line_[line_len_++] = c;
        } else {
            // Overlong sentence: discard and resynchronise on the next '$'.
            line_len_ = 0;
        }
    }
}

void PollingSampler::step() {
    bool sentence_ready = false;
    collect_gps_bytes(&sentence_ready);

    GpsFix fix;
    const bool have_fix =
        sentence_ready && parse_gga(line_, line_len_, clock_.now_ms(), &fix) && fix.valid;
    if (sentence_ready) {
        line_len_ = 0;
    }

    // Blocking. Any GPS byte arriving now has no reader and no buffer.
    ImuSample sample;
    if (imu_.read(&sample)) {
        pairer_.submit_imu(sample);
    }

    if (!have_fix) {
        diag_.drops = gps_.bytes_dropped();
        return;
    }

    diag_.drops = gps_.bytes_dropped();

    Packet packet;
    packet.node_id = node_id_;
    packet.seq = seq_;
    packet.ttl = ttl_;
    packet.record = pairer_.pair(fix, diag_);

    uint8_t buffer[PACKET_SIZE];
    if (encode_packet(packet, buffer, sizeof(buffer)) != PACKET_SIZE) {
        return;
    }

    // Blocking, and the longest stall in the loop.
    if (radio_.transmit(buffer, PACKET_SIZE)) {
        ++seq_;
        ++packets_sent_;
    }

    diag_.drops = gps_.bytes_dropped();
}

}  // namespace floodnet
```

- [ ] **Step 6: Run the tests to verify they pass**

Run: `pio test -e native -f test_polling`
Expected: PASS, `6 Tests 0 Failures 0 Ignored`.

- [ ] **Step 7: Run the whole suite**

Run: `pio test -e native`
Expected: PASS across all seven suites.

- [ ] **Step 8: Commit**

```bash
git add platformio.ini src/sampler_polling.hpp src/sampler_polling.cpp test/test_polling/
git commit -m "feat: add polling sampler with in-band drop reporting"
```

---

### Task 8: Teensy drivers and entry points

The only code in this milestone that cannot be tested on the host.
Verification is that both firmware environments compile.

**Files:**
- Create: `src/hal/teensy_clock.hpp`
- Create: `src/hal/teensy_gps.hpp`
- Create: `src/hal/teensy_imu.hpp`
- Create: `src/hal/teensy_radio.hpp`
- Create: `src/main.cpp`
- Create: `src/gateway_main.cpp`

**Interfaces:**
- Consumes: the four HAL interfaces from Task 6, `PollingSampler` from Task 7, `decode_packet` from Task 2, `DedupTable` from Task 5.
- Produces: `floodnet::TeensyClock`, `floodnet::TeensyGps`, `floodnet::TeensyImu`, `floodnet::TeensyRadio`, plus `setup()` and `loop()` for both firmware images.

Pin assignments, fixed here and documented in the README in Task 9:

| Signal | Teensy 4.1 pin |
|---|---|
| GPS UART RX | 0 (Serial1 RX) |
| GPS UART TX | 1 (Serial1 TX) |
| IMU I2C SDA | 18 |
| IMU I2C SCL | 19 |
| Radio SPI CS | 10 |
| Radio reset | 9 |
| Radio DIO0 | 2 |

- [ ] **Step 1: Write the Teensy clock**

`src/hal/teensy_clock.hpp`:

```cpp
#ifndef FLOODNET_TEENSY_CLOCK_HPP
#define FLOODNET_TEENSY_CLOCK_HPP

#include <Arduino.h>

#include <floodnet/hal/clock.hpp>

namespace floodnet {

class TeensyClock : public IClock {
  public:
    uint32_t now_ms() override { return millis(); }
    void delay_ms(uint32_t ms) override { delay(ms); }
};

}  // namespace floodnet

#endif  // FLOODNET_TEENSY_CLOCK_HPP
```

- [ ] **Step 2: Write the Teensy GPS source**

`src/hal/teensy_gps.hpp`:

```cpp
#ifndef FLOODNET_TEENSY_GPS_HPP
#define FLOODNET_TEENSY_GPS_HPP

#include <Arduino.h>

#include <floodnet/hal/gps.hpp>

namespace floodnet {

/// Reads NMEA from Serial1. Counts bytes the driver's buffer overran, which is
/// the polling loop's loss showing up as a number rather than as silence.
class TeensyGps : public IGpsSource {
  public:
    explicit TeensyGps(HardwareSerial &port) : port_(port), dropped_(0) {}

    void begin(uint32_t baud) { port_.begin(baud); }

    int read_byte() override {
        if (port_.available() <= 0) {
            return -1;
        }
        return port_.read();
    }

    uint16_t bytes_dropped() const override { return dropped_; }

    /// Call once per loop pass: a full receive buffer means bytes were lost.
    void note_buffer_state() {
        if (port_.available() >= RX_BUFFER_BYTES && dropped_ < 0xFFFF) {
            ++dropped_;
        }
    }

  private:
    /// Teensy 4.x HardwareSerial keeps a 64-byte software receive buffer.
    static const int RX_BUFFER_BYTES = 64;

    HardwareSerial &port_;
    uint16_t dropped_;
};

}  // namespace floodnet

#endif  // FLOODNET_TEENSY_GPS_HPP
```

- [ ] **Step 3: Write the Teensy IMU source**

`src/hal/teensy_imu.hpp`:

```cpp
#ifndef FLOODNET_TEENSY_IMU_HPP
#define FLOODNET_TEENSY_IMU_HPP

#include <Adafruit_BNO055.h>
#include <Arduino.h>
#include <Wire.h>

#include <floodnet/hal/imu.hpp>

namespace floodnet {

class TeensyImu : public IImuSource {
  public:
    TeensyImu() : sensor_(55, BNO055_ADDRESS_A, &Wire), ready_(false) {}

    bool begin() {
        Wire.begin();
        Wire.setClock(400000);
        ready_ = sensor_.begin();
        return ready_;
    }

    bool read(ImuSample *out) override {
        if (!ready_ || out == nullptr) {
            return false;
        }

        sensors_event_t event;
        sensor_.getEvent(&event, Adafruit_BNO055::VECTOR_EULER);

        out->time_ms = millis();
        out->yaw_cd = static_cast<int16_t>(event.orientation.x * 100.0f);
        out->pitch_cd = static_cast<int16_t>(event.orientation.y * 100.0f);
        out->roll_cd = static_cast<int16_t>(event.orientation.z * 100.0f);
        out->valid = true;
        return true;
    }

  private:
    Adafruit_BNO055 sensor_;
    bool ready_;
};

}  // namespace floodnet

#endif  // FLOODNET_TEENSY_IMU_HPP
```

- [ ] **Step 4: Write the Teensy radio**

`src/hal/teensy_radio.hpp`:

```cpp
#ifndef FLOODNET_TEENSY_RADIO_HPP
#define FLOODNET_TEENSY_RADIO_HPP

#include <Arduino.h>
#include <RH_RF95.h>
#include <SPI.h>

#include <floodnet/hal/radio.hpp>

namespace floodnet {

class TeensyRadio : public IRadio {
  public:
    TeensyRadio(uint8_t cs_pin, uint8_t interrupt_pin, uint8_t reset_pin)
        : driver_(cs_pin, interrupt_pin), reset_pin_(reset_pin), ready_(false) {}

    bool begin(float frequency_mhz, int8_t tx_power_dbm) {
        pinMode(reset_pin_, OUTPUT);
        digitalWrite(reset_pin_, HIGH);
        digitalWrite(reset_pin_, LOW);
        delay(10);
        digitalWrite(reset_pin_, HIGH);
        delay(10);

        ready_ = driver_.init();
        if (!ready_) {
            return false;
        }
        if (!driver_.setFrequency(frequency_mhz)) {
            ready_ = false;
            return false;
        }
        // Long-range configuration: slower, more robust, less throughput.
        driver_.setModemConfig(RH_RF95::Bw125Cr48Sf4096);
        driver_.setTxPower(tx_power_dbm, false);
        return true;
    }

    bool transmit(const uint8_t *data, size_t len) override {
        if (!ready_) {
            return false;
        }
        if (!driver_.send(data, static_cast<uint8_t>(len))) {
            return false;
        }
        return driver_.waitPacketSent();
    }

    int receive(uint8_t *buf, size_t len) override {
        if (!ready_ || !driver_.available()) {
            return -1;
        }
        uint8_t length = static_cast<uint8_t>(len);
        if (!driver_.recv(buf, &length)) {
            return -1;
        }
        return static_cast<int>(length);
    }

    int16_t last_rssi() { return driver_.lastRssi(); }

  private:
    RH_RF95 driver_;
    uint8_t reset_pin_;
    bool ready_;
};

}  // namespace floodnet

#endif  // FLOODNET_TEENSY_RADIO_HPP
```

- [ ] **Step 5: Write the node entry point**

`src/main.cpp`:

```cpp
#include <Arduino.h>

#include "hal/teensy_clock.hpp"
#include "hal/teensy_gps.hpp"
#include "hal/teensy_imu.hpp"
#include "hal/teensy_radio.hpp"
#include "sampler_polling.hpp"

namespace {

const uint16_t NODE_ID = 1;
const uint8_t PACKET_TTL = 3;
const uint32_t GPS_BAUD = 9600;
const float RADIO_FREQUENCY_MHZ = 915.0f;
const int8_t RADIO_TX_POWER_DBM = 20;

const uint8_t RADIO_CS_PIN = 10;
const uint8_t RADIO_RESET_PIN = 9;
const uint8_t RADIO_DIO0_PIN = 2;

floodnet::TeensyClock g_clock;
floodnet::TeensyGps g_gps(Serial1);
floodnet::TeensyImu g_imu;
floodnet::TeensyRadio g_radio(RADIO_CS_PIN, RADIO_DIO0_PIN, RADIO_RESET_PIN);

floodnet::PollingSampler g_sampler(g_gps, g_imu, g_radio, g_clock, NODE_ID, PACKET_TTL);

}  // namespace

void setup() {
    Serial.begin(115200);
    g_gps.begin(GPS_BAUD);

    if (!g_imu.begin()) {
        Serial.println("imu: init failed");
    }
    if (!g_radio.begin(RADIO_FREQUENCY_MHZ, RADIO_TX_POWER_DBM)) {
        Serial.println("radio: init failed");
    }

    Serial.println("floodnet node: polling sampler");
}

void loop() {
    g_gps.note_buffer_state();
    g_sampler.step();
}
```

- [ ] **Step 6: Write the gateway entry point**

`src/gateway_main.cpp`:

```cpp
#include <Arduino.h>

#include <floodnet/mesh.hpp>
#include <floodnet/packet.hpp>

#include "hal/teensy_clock.hpp"
#include "hal/teensy_radio.hpp"

namespace {

const float RADIO_FREQUENCY_MHZ = 915.0f;
const int8_t RADIO_TX_POWER_DBM = 20;

const uint8_t RADIO_CS_PIN = 10;
const uint8_t RADIO_RESET_PIN = 9;
const uint8_t RADIO_DIO0_PIN = 2;

floodnet::TeensyClock g_clock;
floodnet::TeensyRadio g_radio(RADIO_CS_PIN, RADIO_DIO0_PIN, RADIO_RESET_PIN);
floodnet::DedupTable g_dedup;
uint16_t g_crc_errors = 0;

/// One line per packet, comma separated, so the host pipeline can read it
/// without a framing layer.
void print_record(const floodnet::Packet &p, int16_t rssi) {
    Serial.print("REC,");
    Serial.print(p.node_id);
    Serial.print(',');
    Serial.print(p.seq);
    Serial.print(',');
    Serial.print(p.record.gps.time_ms);
    Serial.print(',');
    Serial.print(p.record.gps.lat_1e7);
    Serial.print(',');
    Serial.print(p.record.gps.lon_1e7);
    Serial.print(',');
    Serial.print(p.record.gps.alt_mm);
    Serial.print(',');
    Serial.print(p.record.gps.satellites);
    Serial.print(',');
    Serial.print(p.record.imu.yaw_cd);
    Serial.print(',');
    Serial.print(p.record.imu.pitch_cd);
    Serial.print(',');
    Serial.print(p.record.imu.roll_cd);
    Serial.print(',');
    Serial.print(p.record.diag.drops);
    Serial.print(',');
    Serial.print(p.record.diag.crc_errors);
    Serial.print(',');
    Serial.println(rssi);
}

}  // namespace

void setup() {
    Serial.begin(115200);
    if (!g_radio.begin(RADIO_FREQUENCY_MHZ, RADIO_TX_POWER_DBM)) {
        Serial.println("radio: init failed");
    }
    Serial.println("floodnet gateway");
}

void loop() {
    uint8_t buffer[floodnet::PACKET_SIZE];
    const int received = g_radio.receive(buffer, sizeof(buffer));
    if (received != static_cast<int>(floodnet::PACKET_SIZE)) {
        return;
    }

    floodnet::Packet packet;
    if (!floodnet::decode_packet(buffer, floodnet::PACKET_SIZE, &packet)) {
        if (g_crc_errors < 0xFFFF) {
            ++g_crc_errors;
        }
        Serial.print("ERR,crc,");
        Serial.println(g_crc_errors);
        return;
    }

    if (g_dedup.seen(packet.node_id, packet.seq)) {
        return;
    }

    print_record(packet, g_radio.last_rssi());
}
```

- [ ] **Step 7: Build both firmware environments**

Run: `pio run -e node_polling -e gateway`
Expected: SUCCESS for both. PlatformIO downloads the Adafruit BNO055 and RadioHead libraries on first run.

If the BNO055 header reports a missing `Adafruit_Sensor.h`, add `adafruit/Adafruit Unified Sensor@^1.1.14` to `lib_deps` under `[teensy_base]` and rebuild.

- [ ] **Step 8: Confirm the host suite still passes**

Run: `pio test -e native`
Expected: PASS across all seven suites. The Teensy code is excluded from the native build, so nothing here should have disturbed it.

- [ ] **Step 9: Commit**

```bash
git add src/hal/ src/main.cpp src/gateway_main.cpp
git commit -m "feat: add Teensy drivers and node and gateway entry points"
```

---

### Task 9: README and hardware documentation

What a reader sees first.
For a repository whose purpose is to be read by someone deciding whether to interview you, this is not an afterthought.

**Files:**
- Create: `README.md`
- Create: `docs/hardware.md`

**Interfaces:**
- Consumes: everything built in Tasks 1 to 8.
- Produces: no code.

- [ ] **Step 1: Write `docs/hardware.md`**

```markdown
# Hardware

## Bill of materials

| Component | Part | Interface |
|---|---|---|
| MCU | Teensy 4.1 | - |
| GPS | u-blox NEO-M8N | UART, NMEA 0183, 9600 baud |
| IMU | Bosch BNO055 | I2C at 400 kHz |
| Radio | HopeRF RFM95W (SX1276) | SPI, 915 MHz |

## Pin assignment

| Signal | Teensy 4.1 pin |
|---|---|
| GPS UART RX | 0 (Serial1 RX) |
| GPS UART TX | 1 (Serial1 TX) |
| IMU I2C SDA | 18 |
| IMU I2C SCL | 19 |
| Radio SPI CS | 10 |
| Radio reset | 9 |
| Radio DIO0 | 2 |

## Radio configuration

The link uses `Bw125Cr48Sf4096`: 125 kHz bandwidth, 4/8 coding rate, spreading factor 12.
This is the most robust of RadioHead's stock configurations and the slowest.
The trade is deliberate. Nodes report at most a few times per minute, so throughput is not the constraint; link margin is.
```

- [ ] **Step 2: Write `README.md`**

```markdown
# FloodNet

Firmware for a distributed sensor network that maps flooding.

Battery-powered nodes sample GPS position and IMU orientation, pack each observation into a
45-byte frame, and relay it over LoRa to a gateway.
The gateway validates what it receives and writes one line per observation to a serial link.

## Status

Milestone 1: polling-based acquisition.
Each pass of the main loop visits every sensor in turn and blocks on it.
This is the starting point, not the destination. See [Known limitations](#known-limitations).

## Layout

| Path | Contents |
|---|---|
| `lib/floodnet_core/` | Parsing, packing, and relay logic. No hardware dependency. |
| `lib/floodnet_hal/` | Hardware interfaces. |
| `src/` | Teensy drivers, the sampler, and the two entry points. |
| `test/` | Host-runnable unit tests and simulation fakes. |
| `docs/` | Hardware notes and design documents. |

Nothing under `lib/` includes an Arduino header.
That constraint is what lets the majority of this repository be tested on a development machine
with no hardware attached.

## Building

```bash
pio run -e node_polling    # sensor node
pio run -e gateway         # gateway
pio test -e native         # unit tests, no hardware needed
```

## Wire format

45 bytes, little-endian, CRC16-CCITT-FALSE over the first 43.
Carries a node identifier, a sequence number, a TTL, a GPS fix, an IMU sample, and the node's
drop and CRC error counters.
Those counters travel in-band deliberately: a receiver can see loss at the source rather than
inferring it from gaps.

Full layout is in `docs/superpowers/specs/2026-09-20-floodnet-firmware-design.md`.

## Known limitations

The polling loop blocks on the IMU and on radio transmission.
GPS bytes arriving during those stalls land in a 64-byte hardware buffer, and when that buffer
fills, they are gone.
`test/test_polling/` asserts that this loss happens rather than papering over it, and the loss
is reported in every outgoing packet.

Removing it is the next milestone.

## Hardware

See [docs/hardware.md](docs/hardware.md).

## About this repository

A clean rewrite of firmware I built for riva labs between 2024 and 2026.
The original is on lab infrastructure and is not public, so this is written from scratch against
the same requirements.
```

- [ ] **Step 3: Verify the README's commands actually work**

Run each of the three commands from the Building section.
Expected: all three succeed. Fix the README if any command is wrong.

- [ ] **Step 4: Commit**

```bash
git add README.md docs/hardware.md
git commit -m "docs: describe the system, wire format, and known limitations"
```

---

## Milestone Completion Check

Run before declaring milestone 1 done:

- [ ] `pio test -e native` passes all seven suites
- [ ] `pio run -e node_polling -e gateway` builds both images
- [ ] `git log --oneline` shows nine commits, none with a co-author trailer
- [ ] No file under `lib/` includes `Arduino.h`, verifiable with `grep -r "Arduino.h" lib/` returning nothing
- [ ] README's three build commands all run as written

## Self-Review Notes

Spec coverage for milestone 1, section by section:

| Spec element | Task |
|---|---|
| `floodnet_core` with no hardware dependency | 1 to 5, enforced by the completion check |
| `ring_buffer.hpp` | Deferred to milestone 2. Milestone 1's single shared line buffer is the baseline a ring buffer replaces. |
| `packet.hpp` with CRC16 and sequence numbers | 2 |
| `nmea.hpp` | 3 |
| `sample.hpp` | 1 |
| `mesh.hpp` with dedup and TTL | 5 |
| `node_state.hpp` | Deferred to milestone 3, which introduces sleep and duty cycling. |
| HAL interfaces with two implementations | 6 (interfaces, fakes) and 8 (Teensy) |
| Polling sampler | 7 |
| Drop, CRC, and sequence counters | 2 (wire format), 7 (drops), 8 (gateway CRC counter) |
| Gateway writing framed records to serial | 8 |
| Host unit tests | 1 to 7 |
| CI compiling environments and running tests | 1 |

Sequence-gap counting is a receiver-side calculation over the `seq` field and belongs with the host pipeline in milestone 5, so no task here implements it.
The field it needs is in the wire format from Task 2 onward.
