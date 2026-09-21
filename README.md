# FloodNet

Firmware for a distributed sensor network that maps flooding.

Battery-powered nodes sample GPS position and IMU orientation, pack each observation into a 45-byte frame, and relay it over LoRa to a gateway.
The gateway validates what it receives and writes one line per observation to a serial link.

## Status

Milestone 2: interrupt-driven acquisition.
The main loop no longer blocks on any peripheral, and both acquisition strategies remain selectable build targets so the comparison between them stays reproducible.

Milestone 1's polling strategy is still built and tested as `node_polling`, and its original 64-byte-buffer baseline is still measured on every run, in the host simulation.
See [Results](#results) for what changed and [Known limitations](#known-limitations) for what did not.

**The two build targets differ in acquisition strategy, not in GPS buffer depth.**
`TeensyGps` supplies its 4096-byte receive buffer unconditionally, so `node_polling` ships as polling with the deep buffer, not as the original milestone 1 configuration.
The milestone 1 baseline, polling with the original 64-byte buffer, survives only as the `polling,64` row in the host simulation; see [Known limitations](#known-limitations).

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
pio run -e node_polling     # sensor node, milestone 1 polling loop
pio run -e node_interrupt   # sensor node, milestone 2 non-blocking loop
pio run -e gateway          # gateway
pio test -e native          # unit tests, no hardware needed
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

## Results

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

`test/test_benchmark/test_main.cpp` runs each profile for at least 60 simulated seconds, rather than a fixed step count, so rows are directly comparable on a per-second basis, and prints one line per combination in a stable, greppable format.
These are simulated measurements from the host test suite, not field data.
Reproduce them with:

```bash
pio test -e native -f test_benchmark -v
```

### The 2x2

Milestone 2 changed two things at once: the loop stopped blocking, and the GPS receive buffer grew from 64 to 4096 bytes.
Reporting a single before-and-after would not say which change bought what, so the benchmark runs both acquisition strategies at both buffer depths, across all three radio profiles.

Each line printed by the benchmark is `BENCH,strategy,gps_buffer_bytes,profile,sim_duration_ms,packets_sent,gps_rx_overflows,tx_queue_drops,packets_per_sec,gps_rx_overflows_per_sec,tx_queue_high_water`.
The run that produced the figures below printed:

```
BENCH,polling,64,CONTROL,60001,859,0,0,14.32,0.00,0
BENCH,polling,64,SF7,60060,360,12204,0,5.99,203.20,0
BENCH,polling,64,SF12,61800,20,56948,0,0.32,921.49,0
BENCH,polling,4096,CONTROL,60001,859,0,0,14.32,0.00,0
BENCH,polling,4096,SF7,60046,588,0,0,9.79,0.00,0
BENCH,polling,4096,SF12,60730,20,0,0,0.33,0.00,0
BENCH,interrupt,64,CONTROL,60000,859,0,0,14.32,0.00,1
BENCH,interrupt,64,SF7,60000,544,0,306,9.07,0.00,8
BENCH,interrupt,64,SF12,60000,19,0,831,0.32,0.00,8
BENCH,interrupt,4096,CONTROL,60000,859,0,0,14.32,0.00,1
BENCH,interrupt,4096,SF7,60000,544,0,306,9.07,0.00,8
BENCH,interrupt,4096,SF12,60000,19,0,831,0.32,0.00,8
```

| Strategy | Buffer | Profile | pkt/s | GPS ovf/s | tx drops |
|---|---|---|---|---|---|
| polling | 64 | CONTROL | 14.32 | 0.00 | 0 |
| polling | 64 | SF7 | 5.99 | 203.20 | 0 |
| polling | 64 | SF12 | 0.32 | 921.49 | 0 |
| polling | 4096 | CONTROL | 14.32 | 0.00 | 0 |
| polling | 4096 | SF7 | 9.79 | 0.00 | 0 |
| polling | 4096 | SF12 | 0.33 | 0.00 | 0 |
| interrupt | 64/4096 | CONTROL | 14.32 | 0.00 | 0 |
| interrupt | 64/4096 | SF7 | 9.07 | 0.00 | 306 |
| interrupt | 64/4096 | SF12 | 0.32 | 0.00 | 831 |

Three things to read out of it:

1. **Either change alone eliminates GPS byte loss. They are redundant for that purpose.**
   Polling with a 4096-byte buffer reports zero overflows at all three profiles, and the interrupt build reports zero at both buffer depths.
   The buffer was sized at 4096 bytes to cover 3023 ms x 0.96 B/ms = 2902 bytes of SF12 airtime, and it turns out that alone is sufficient without a non-blocking loop.
   "Interrupt-driven acquisition eliminated the data loss" is not a claim this data supports.
   A bigger `HardwareSerial` buffer would have done it.

2. **At SF7 the deeper buffer alone is the fastest configuration measured, and faster than the interrupt build.**
   Polling at 4096 reaches 9.79 packets/sec against the interrupt build's 9.07 at either depth, from a 5.99 baseline.
   The milestone's throughput claim is therefore withdrawn rather than softened; see [Why the interrupt build is slower at SF7](#why-the-interrupt-build-is-slower-at-sf7).

3. **At SF12 nothing moves the packet rate**, because the radio is the bottleneck: every configuration lands between 0.32 and 0.33 packets/sec.

**What the non-blocking loop actually buys**, none of which is throughput:

- At SF12, 831 counted packet drops replace 56948 silent byte-level overflows.
  The loss is the same physics; what changes is that it is explicit, chosen by a stated policy, and attributable.
- Sequence numbers are assigned when a fix is queued rather than when it is sent, so a receiver sees a gap and can count what the node discarded.
- The outbound queue keeps the freshest fix rather than the oldest, which matters for position data.
- The loop is free to do other work.
  Milestones 3 through 5 need relay, duty cycling and sleep, and a superloop that blocks for three seconds on a transmit structurally cannot host them.

CONTROL is a null control rather than a result.
It is limited by the GPS sentence rate, not the radio, so a non-blocking loop has nothing to win there.
`test_control_profile_is_a_null_control` in `test/test_benchmark/` fails if that row gains more than 10%, because a gain there would mean the simulation was flattering the new sampler.

## Known limitations

Milestone 1's polling loop blocked on the IMU and on radio transmission.
GPS bytes arriving during those stalls landed in `HardwareSerial`'s default 64-byte software receive buffer, and when that buffer filled, they were gone.
That buffer is not a passive hardware register: it is filled by the Teensy UART receive interrupt, and the polling loop's `read_byte()` only drains what the interrupt already collected.
`test/test_polling/` asserts that this loss happens rather than papering over it, and the loss is reported in every outgoing packet's `drops` field, which carries the node's `IGpsSource::rx_overflows()` value.
That fixture keeps the 64-byte depth so the `polling,64` benchmark row remains comparable with milestone 1.

**`node_polling` as built today does not reproduce this.**
`TeensyGps::begin()` in `src/hal/teensy_gps.hpp` calls `addMemoryForRead()` unconditionally, and `src/main.cpp` constructs the single `TeensyGps` instance outside the `FLOODNET_SAMPLER_INTERRUPT`/`FLOODNET_SAMPLER_POLLING` branch, so both build targets supply the same 4096-byte buffer.
The two shipped firmware images therefore differ only in acquisition strategy (blocking vs. non-blocking), not in GPS buffer depth.
The milestone 1 configuration, polling with a 64-byte buffer, exists only as the `polling,64` row in the host simulation below, not as anything you can flash.
Someone who flashes `node_polling` expecting milestone 1's loss behaviour will not see it; they will see the `polling,4096` row instead.

That counter's unit is implementation-defined, and the two implementations in this repository do not measure the same thing.
The simulated GPS source in `test/support/fake_gps.hpp` counts one overflow per individual byte it discards, because it knows exactly what it threw away.
The Teensy driver in `src/hal/teensy_gps.hpp` counts one overflow per loop pass that finds the UART buffer already saturated, because real hardware exposes no lost-byte count.
Do not compare a drop count from the simulation against a drop count from hardware; they are different quantities that happen to share a name.
See the contract comment on `IGpsSource::rx_overflows()` in `lib/floodnet_hal/include/floodnet/hal/gps.hpp`.

The simulated GPS source also transmits back-to-back at line rate with no gaps between sentences, which represents a high-output-rate NMEA configuration rather than a real NEO-M8N's 1 Hz default.
That choice affects every figure in [Results](#results): a receiver idling between fixes at 1 Hz would see far less contention for the buffer than the simulation models.
The figures are a stress test, not a prediction of field behavior at the default configuration.

See [Results](#results) for the radio timing profiles and the measured figures.

### What is and is not already interrupt-driven

Framing this as simply "polling versus interrupt-driven" overstates what changed, so this is worth stating precisely for each of the three peripherals.

The GPS path is served by `HardwareSerial`'s own receive interrupt, filling a 4096-byte buffer.
`TeensyGps::begin()` hands that buffer to the interrupt handler through `addMemoryForRead()` rather than implementing a receive interrupt of its own; see [docs/hardware.md](docs/hardware.md) and the milestone 2 design doc for why a hand-rolled ring buffer cannot sit ahead of it.
`InterruptSampler::collect_gps()` drains whatever the interrupt has already collected on every pass, instead of only between blocking calls to other sensors.

The radio is started and then left alone rather than waited on.
`TeensyRadio::begin_transmit()` calls `RH_RF95::send()`, which loads the FIFO and returns; the DIO0 interrupt RadioHead attached during `init()` clears the driver's mode once the packet is on the air.
`InterruptSampler::service_radio()` checks `tx_busy()` on a later pass instead of spinning inside `waitPacketSent()`, so a three-second SF12 transmission no longer stalls the loop.

The IMU is flagged by a 100 Hz Teensy `IntervalTimer` rather than by the sensor.
`TeensyImu::begin()` arms the timer at the BNO055's fixed NDOF fusion rate; the handler sets a flag and returns, and `InterruptSampler::collect_imu()` performs the blocking I2C read only when `data_ready()` reports a sample.
See [docs/hardware.md](docs/hardware.md), "IMU sample timing", for why this is a timer and not the sensor's own INT pin.

Put together: the loop no longer blocks waiting on any of the three peripherals.
None of this makes the radio faster: at SF12 it remains the bottleneck under every configuration measured, and even at SF7 the interrupt build is not the fastest one measured; see [Results](#results).

### What the measurement does and does not prove

The simulation treats the MCU as infinitely fast.
Time advances only through modelled hardware timing: 9600 baud byte arrival, the BNO055's 100 Hz fusion rate, and LoRa airtime from the formula in [Results](#results).
No CPU cost per byte parsed or packet encoded is charged to either sampler.

That was a deliberate choice.
On a 600 MHz Teensy 4.1 those costs sit roughly four orders of magnitude below the 3023 ms of airtime that dominates the result, so modelling them would mean inventing constants in order to change nothing.

The consequence is a narrower claim than it may first appear: these figures show that **blocking was the loss mechanism**.
They do not show that the Teensy has the cycles to keep up.
Establishing that needs hardware, which milestone 2 did not have.

### Why the interrupt build is slower at SF7

`FakeImu` charges a 10 ms read on nearly every pass, and the interrupt sampler detects transmit completion only at pass boundaries, so a completion is noticed up to 10 ms late.
The polling loop serialises the IMU read and the transmit (92 + 10 = 102 ms, a 9.80/sec ceiling) but never spends a pass without making progress.

A real BNO055 read over I2C at 400 kHz is closer to 1 ms than 10 ms, at which point the two builds would be near-identical.
That 10 ms figure was not changed, because it is milestone 1's published value and altering it would invalidate the baseline this milestone is measured against.

Calling `service_radio()` before `collect_imu()` would detect completion sooner and would likely close the gap.
It has deliberately not been done.
It was identified only after seeing a result that disfavoured the design, and changing code to improve a number after seeing that number is tuning to the benchmark.
It is recorded here as future work so that "did anything change after you saw the results?" has a one-word answer.

### Not validated on hardware

No board was available for this milestone.
Every figure here is from the host simulation, and the firmware is compiled but never run on a Teensy in CI.
The `IntervalTimer` IMU path in particular is exercised only against a fake.

## Deferred to later milestones

- **Mesh relay.** `should_relay()` and `prepare_relay()` in `lib/floodnet_core/include/floodnet/mesh.hpp` are complete and unit-tested, but nothing outside `test/test_mesh/` calls them; no node currently relays another node's packet.
  Wiring receive-and-relay into the node loop needs a duty-cycle policy so nodes do not transmit over each other, and that policy belongs with the power management work planned for milestone 3.
- **Ring buffer.** `lib/floodnet_core/include/floodnet/ring_buffer.hpp` exists, is lock-free, and is fully tested, but it sits on no data path.
  Mapping each acquisition path onto the hardware left it without a consumer: `HardwareSerial` owns the GPS receive interrupt, RadioHead owns the radio's, the IMU handler can only set a flag because retrieving a sample needs a blocking I2C transaction, and the outbound queue needs drop-oldest, which single-producer single-consumer ordering forbids.
  It is kept and labelled rather than quietly wired into a path that does not need it.
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
