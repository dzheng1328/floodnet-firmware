# FloodNet

Firmware for a distributed sensor network that maps flooding.

Battery-powered nodes sample GPS position and IMU orientation, pack each observation into a 45-byte frame, and relay it over LoRa to a gateway.
The gateway validates what it receives and writes one line per observation to a serial link.

## Status

Milestone 1: polling-based acquisition.
Each pass of the main loop visits every sensor in turn and blocks on it.
This is the starting point, not the destination.
See [Known limitations](#known-limitations).

## Layout

| Path | Contents |
|---|---|
| `lib/floodnet_core/` | Parsing, packing, and relay logic. No hardware dependency. |
| `lib/floodnet_hal/` | Hardware interfaces. |
| `src/` | Teensy drivers, the sampler, and the two entry points. |
| `test/` | Host-runnable unit tests and simulation fakes. |
| `docs/` | Hardware notes and design documents. |

Nothing under `lib/` includes an Arduino header.
That constraint is what lets the majority of this repository be tested on a development machine with no hardware attached.
CI enforces it directly: a step in `.github/workflows/ci.yml` greps `lib/` for `Arduino.h`, `Wire.h`, and `SPI.h` and fails the build if any of them appear.

## Building

```bash
pio run -e node_polling    # sensor node
pio run -e gateway         # gateway
pio test -e native         # unit tests, no hardware needed
```

## Wire format

45 bytes, little-endian, CRC16-CCITT-FALSE over the first 43.
Carries a node identifier, a sequence number, a TTL, a GPS fix, an IMU sample, and the node's drop counter.
That counter travels in-band deliberately: a receiver can see loss at the source rather than inferring it from gaps.
A second counter reserved for CRC errors travels alongside it but is always zero in milestone 1, since a node only transmits and never decodes a CRC of its own; see the byte table below.

The byte layout below is read directly from `encode_packet()` in `lib/floodnet_core/src/packet.cpp`, which is the only authoritative description of it.
`test/test_packet/test_main.cpp` has a golden-vector test, `test_golden_vector_pins_byte_layout`, that encodes a fully specified packet and asserts the exact 45 bytes.
If that test ever fails, this table is wrong and needs to be regenerated from the code, not the other way around.

| Offset | Size | Field | Notes |
|---|---|---|---|
| 0 | 1 | magic | `0xFD` |
| 1 | 1 | version | `0x01` |
| 2 | 2 | node_id | uint16 |
| 4 | 4 | seq | uint32 |
| 8 | 1 | ttl | uint8 |
| 9 | 1 | flags | uint8 |
| 10 | 4 | gps.time_ms | uint32 |
| 14 | 4 | gps.lat_1e7 | int32, degrees x 1e7 |
| 18 | 4 | gps.lon_1e7 | int32, degrees x 1e7 |
| 22 | 4 | gps.alt_mm | int32, millimetres above mean sea level |
| 26 | 1 | gps.satellites | uint8 |
| 27 | 1 | gps.valid | 0 or 1 |
| 28 | 4 | imu.time_ms | uint32 |
| 32 | 2 | imu.yaw_cd | int16, centidegrees |
| 34 | 2 | imu.pitch_cd | int16, centidegrees |
| 36 | 2 | imu.roll_cd | int16, centidegrees |
| 38 | 1 | imu.valid | 0 or 1 |
| 39 | 2 | diag.drops | uint16 |
| 41 | 2 | diag.crc_errors | uint16, reserved, always 0 in milestone 1 |
| 43 | 2 | crc16 | uint16, CRC16-CCITT-FALSE over bytes 0-42 |

## Gateway serial output

The gateway writes one line per event to its USB serial port at 115200 baud, comma-separated, so a host pipeline can parse it without a framing layer.
A radio receive that finds nothing waiting produces no output at all; that case is intentionally silent, not unlogged.

A successfully decoded, non-duplicate packet produces a `REC` line with these fields in order:

```
REC,node_id,seq,gps_time_ms,lat_1e7,lon_1e7,alt_mm,satellites,yaw_cd,pitch_cd,roll_cd,drops,crc_errors,rssi,gps_valid,imu_valid
```

`gps_valid` and `imu_valid` are appended at the end rather than inserted among the existing fields, so the field order a consumer already depends on stays stable.
Each is `1` or `0`.
`crc_errors` is always `0` here in milestone 1, for the same reason it is reserved in the wire format: a node only transmits and never decodes a CRC of its own.
`imu_valid` is `0` when `SamplePairer::pair` could not find an IMU sample within the pairing skew budget; without this flag, an unpaired record's `yaw_cd,pitch_cd,roll_cd` of `0,0,0` is indistinguishable from a genuinely level node.

A packet that fails to decode produces:

```
ERR,decode,decode_error_count
```

`decode_packet()` rejects a packet on bad magic, bad version, or bad CRC, and returns a single bool, so these three causes are not distinguishable from the gateway's side of that call.
The line is named `ERR,decode` rather than `ERR,crc` so it does not claim a cause it cannot identify.
Separating the three causes is a later milestone's job if it proves necessary.

A radio receive that returns the wrong number of bytes produces:

```
ERR,short,bytes_received,short_read_count
```

`decode_error_count` and `short_read_count` are running totals kept by the gateway since power-on, capped at 65535.
The authoritative source for this format is `print_record()` and `loop()` in `src/gateway_main.cpp`.

## Known limitations

The polling loop blocks on the IMU and on radio transmission.
GPS bytes arriving during those stalls land in a 64-byte software receive buffer, and when that buffer fills, they are gone.
That buffer is not a passive hardware register: it is filled by the Teensy UART receive interrupt, and the polling loop's `read_byte()` only drains what the interrupt already collected.
`test/test_polling/` asserts that this loss happens rather than papering over it, and the loss is reported in every outgoing packet's `drops` field, which carries the node's `IGpsSource::rx_overflows()` value.

That counter's unit is implementation-defined, and the two implementations in this repository do not measure the same thing.
The simulated GPS source in `test/support/fake_gps.hpp` counts one overflow per individual byte it discards, because it knows exactly what it threw away.
The Teensy driver in `src/hal/teensy_gps.hpp` counts one overflow per loop pass that finds the UART buffer already saturated, because real hardware exposes no lost-byte count.
Do not compare a drop count from the simulation against a drop count from hardware; they are different quantities that happen to share a name.
See the contract comment on `IGpsSource::rx_overflows()` in `lib/floodnet_hal/include/floodnet/hal/gps.hpp`.

The simulated GPS source also transmits back-to-back at line rate with no gaps between sentences, which represents a high-output-rate NMEA configuration rather than a real NEO-M8N's 1 Hz default.
That choice affects every figure below: a receiver idling between fixes at 1 Hz would see far less contention for the buffer than the simulation models.
The figures are a stress test of the polling strategy, not a prediction of field behavior at the default configuration.

### Radio timing profiles

`test/test_polling/` and `test/test_benchmark/` run the same sampler code against three simulated radio timing profiles, named for what they are:

| Profile | IMU read | Radio transmit | What it represents |
|---|---|---|---|
| CONTROL | 2 ms | 5 ms | Synthetic. No real radio is this fast. Isolates whether loss tracks stall duration. |
| SF7 | 10 ms | 92 ms | The fastest practical LoRa setting (SF7/BW125/CR4-5). |
| SF12 | 10 ms | 3023 ms | What `src/hal/teensy_radio.hpp` actually programs (`Bw125Cr48Sf4096`, SF12/BW125/CR4-8). |

The SF12 and SF7 airtime figures are computed, not guessed, from the standard LoRa airtime formula for a 45-byte explicit-header packet:

```
Tsym = 2^SF / BW
T_preamble = (8 + 4.25) * Tsym
n_payload_symbols = 8 + ceil((8*PL - 4*SF + 28 + 16) / (4*(SF - 2*DE))) * (CR + 4)
T_payload = n_payload_symbols * Tsym
airtime = T_preamble + T_payload
```

At SF12/BW125/CR4-8 this comes to approximately 3022.8 ms; at SF7/BW125/CR4-5, approximately 92.4 ms.

`test/test_benchmark/test_main.cpp` runs each profile for at least 60 simulated seconds, rather than a fixed step count, so the three are directly comparable, and prints one line per profile in a stable, greppable format.
These are simulated measurements from the host test suite, not field data.
Reproduce them with:

```bash
pio test -e native -f test_benchmark -v
```

The run that produced the figures below printed:

```
BENCH,CONTROL,60001,859,0,14.32,0.00
BENCH,SF7,60060,360,12204,5.99,203.20
BENCH,SF12,61800,20,56948,0.32,921.49
```

| Profile | Packets/sec | Overflows/sec |
|---|---|---|
| CONTROL | 14.32 | 0.00 |
| SF7 | 5.99 | 203.20 |
| SF12 | 0.32 | 921.49 |

The point is not the absolute numbers; it is that identical parsing and radio code loses nothing at a synthetic control speed, and loses steadily worse as radio airtime grows, at both a realistic and the deployed spreading factor.
No practical LoRa setting lets this superloop keep up: SF7 loses data too, just at a lower rate than SF12.
The loss is a property of the blocking strategy, not of the NMEA parser, the radio driver, or the choice of spreading factor.
Removing it is the next milestone.

### What is and is not already interrupt-driven

Framing milestone 1 versus a later interrupt-driven milestone as simply "polling versus interrupt-driven" overstates what changes, so this is worth stating precisely.

The GPS path is already interrupt-served today.
The Teensy UART receive interrupt fills the 64-byte software buffer described above, and the polling superloop only drains it with `read_byte()`.
What milestone 1 actually lacks on that path is a buffer deep enough to cover a multi-second stall, and a drain schedule that does not block on other sensors while bytes are arriving.

The radio path is also already interrupt-served today.
RadioHead's `RH_RF95::init()` attaches a hardware interrupt to the pin this firmware passes as `RADIO_DIO0_PIN`, and that interrupt sets `_rxBufValid` (the flag `available()` reads) on receive and clears `_mode` (the flag `waitPacketSent()` spins on) once transmission completes.
What milestone 1 actually lacks on that path is that `TeensyRadio::transmit()` calls `waitPacketSent()`, which spins the superloop waiting on a completion the interrupt has already recorded, instead of returning to other work and being notified when it happens.

The IMU is the one peripheral that is genuinely polled today.
`TeensyImu::read()` performs a blocking I2C `getEvent()` call, and the BNO055 data-ready line is not wired to anything.
This is the only path where the next milestone adds an interrupt that does not currently exist at all.

Put together: two of the three peripherals already have interrupt service.
The defect in milestone 1 is not an absence of interrupts; it is a superloop that blocks waiting on them anyway.
Milestone 2's real change is making acquisition non-blocking and adding the one genuinely missing interrupt path, on the IMU.

## Deferred to later milestones

- **Mesh relay.** `should_relay()` and `prepare_relay()` in `lib/floodnet_core/include/floodnet/mesh.hpp` are complete and unit-tested, but nothing outside `test/test_mesh/` calls them; no node currently relays another node's packet.
  Wiring receive-and-relay into the node loop needs a duty-cycle policy so nodes do not transmit over each other, and that policy belongs with the power management work planned for milestone 3.
- **Ring buffer.** Deferred to milestone 2, alongside the non-blocking drain path it is meant to back.
- **Node state machine and sleep/duty cycling.** Deferred to milestone 3, alongside the mesh relay duty-cycle policy above.
- **Sequence-gap counting.** Deferred to milestone 5.
  It is a receiver-side concern and belongs with the gateway/host tooling work planned there.
- **Hardware watchdog.** The spec's error-handling section calls for a watchdog that resets a node that stops making progress; none exists yet.
  A node whose radio wedges currently stays dead until power-cycled.
  `TeensyRadio::transmit()`'s 5000 ms `waitPacketSent()` timeout (see Radio timing profiles and `src/hal/teensy_radio.hpp`) reduces that exposure but does not eliminate it, since nothing currently forces a reset if the node keeps retrying a dead radio indefinitely.
- **Host HAL trace replay.** The design spec calls for a host HAL implementation that replays recorded sensor traces from disk.
  What shipped instead is the header-only synthetic fakes under `test/support/`, which repeat one hardcoded NMEA sentence at a fixed byte rate.
  That substitution is a reasonable milestone-1 choice, sufficient for the timing-driven tests this milestone needs, but it is not what the spec describes.
  Recorded-trace replay is planned to arrive with the hardware-in-the-loop tooling in milestone 4.

## Hardware

See [docs/hardware.md](docs/hardware.md).

## About this repository

A clean rewrite of firmware I built for a Duke research lab between 2024 and 2026.
The original is on lab infrastructure and is not public, so this is written from scratch against the same requirements.
