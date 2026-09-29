# FloodNet

Firmware for a distributed sensor network that maps flooding.

Battery-powered nodes sample GPS position and IMU orientation, pack each observation into a 45-byte frame, and relay it over LoRa to a gateway.
The gateway validates what it receives and writes one line per observation to a serial link.

## Status

Milestone 3: power management and fault recovery.
A new node build puts the milestone 2 sampler under a state machine that powers each peripheral only while it is needed, reports once every 5 minutes, sleeps the Teensy in between, and recovers from a wedged radio and a hung bus.
The result is measured as sensor downtime, in the host simulation; see [Milestone 3: downtime](#milestone-3-downtime).

There are three node build targets, and all three remain selectable so every comparison stays reproducible:

| Target | What it is | Wire version |
|---|---|---|
| `node_polling` | Milestone 1's blocking polling loop, always powered, no watchdog. | v0x01 |
| `node_interrupt` | Milestone 2's non-blocking loop, always powered, no watchdog. | v0x01 |
| `node_duty_cycled` | Milestone 3: `DutyCycledNode` runs the milestone 2 sampler for one fix per 5-minute slot, sleeps the Teensy between slots, and runs the WDOG1 watchdog. | v0x02 |

Milestone 1's original 64-byte-buffer baseline is still measured on every run, in the host simulation.
See [Results](#results) for what changed and [Known limitations](#known-limitations) for what did not.

**`node_polling` and `node_interrupt` differ in acquisition strategy, not in GPS buffer depth.**
`TeensyGps` supplies its 4096-byte receive buffer unconditionally, so `node_polling` ships as polling with the deep buffer, not as the original milestone 1 configuration.
The milestone 1 baseline, polling with the original 64-byte buffer, survives only as the `polling,64` row in the host simulation; see [Known limitations](#known-limitations).

## Layout

| Path | Contents |
|---|---|
| `lib/floodnet_core/` | Parsing, packing, and relay logic. No hardware dependency. |
| `lib/floodnet_hal/` | Hardware interfaces. |
| `src/` | Teensy drivers, the two samplers, `DutyCycledNode`, and the two entry points. |
| `test/` | Host-runnable unit tests and simulation fakes. |
| `docs/` | Hardware notes and design documents. |

Nothing under `lib/` includes an Arduino header.
That constraint is what lets the majority of this repository be tested on a development machine with no hardware attached.
CI enforces it directly: a step in `.github/workflows/ci.yml` greps `lib/` for `Arduino.h`, `Wire.h`, and `SPI.h` and fails the build if any of them appear.

## Building

```bash
pio run -e node_polling     # sensor node, blocking polling loop
pio run -e node_interrupt   # sensor node, milestone 2 non-blocking loop
pio run -e node_duty_cycled # sensor node, milestone 3 duty-cycled with watchdog
pio run -e gateway          # gateway
pio test -e native          # unit tests, no hardware needed
```

## Wire format

Two versions share one frame size: 45 bytes, little-endian, magic `0xFD`, a version byte, and CRC16-CCITT-FALSE over the first 43.
`node_polling` and `node_interrupt` send v0x01.
`node_duty_cycled` sends v0x02.
The gateway dispatches on the version byte and decodes both; any other version is rejected.

Both carry a node identifier, a sequence number, a TTL, a GPS fix, an IMU sample, and the node's drop counter.
That counter travels in-band deliberately: a receiver can see loss at the source rather than inferring it from gaps.

The byte layouts below are read directly from `encode_packet()` and `encode_packet_v2()` in `lib/floodnet_core/src/packet.cpp`, which are the only authoritative description of them.
`test/test_packet/test_main.cpp` pins each with a golden-vector test, `test_golden_vector_pins_byte_layout` for v0x01 and `test_v2_golden_vector_pins_byte_layout` for v0x02, that encodes a fully specified packet and asserts the exact 45 bytes.
If either test ever fails, the matching table is wrong and needs to be regenerated from the code, not the other way around.

### v0x01

A second counter reserved for CRC errors travels alongside the drop counter but is always zero, since a node only transmits and never decodes a CRC of its own.

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
| 41 | 2 | diag.crc_errors | uint16, reserved, always 0 |
| 43 | 2 | crc16 | uint16, CRC16-CCITT-FALSE over bytes 0-42 |

### v0x02

v0x02 carries three facts the gateway needs to attribute downtime: `boot_count`, `tx_timeouts`, and `battery_mv`.
It funds their 6 bytes from fields that carried nothing, so the frame stays at 45 bytes.
`diag.crc_errors` is dropped, the two validity bytes become bits of `flags`, and the 32-bit `imu.time_ms` becomes a 16-bit offset from the GPS time.

| Offset | Size | Field | Notes |
|---|---|---|---|
| 0 | 1 | magic | `0xFD` |
| 1 | 1 | version | `0x02` |
| 2 | 2 | node_id | uint16 |
| 4 | 4 | seq | uint32, restarts at 0 after every reset |
| 8 | 1 | ttl | uint8 |
| 9 | 1 | flags | bit 0 `gps_valid`, bit 1 `imu_valid`, bits 2-7 passed through |
| 10 | 4 | gps.time_ms | uint32 |
| 14 | 4 | gps.lat_1e7 | int32, degrees x 1e7 |
| 18 | 4 | gps.lon_1e7 | int32, degrees x 1e7 |
| 22 | 4 | gps.alt_mm | int32, millimetres above mean sea level |
| 26 | 1 | gps.satellites | uint8 |
| 27 | 2 | imu_skew_ms | int16, `imu.time_ms - gps.time_ms`; 0 when `imu_valid` is 0 |
| 29 | 2 | imu.yaw_cd | int16, centidegrees; 0 when `imu_valid` is 0 |
| 31 | 2 | imu.pitch_cd | int16, centidegrees; 0 when `imu_valid` is 0 |
| 33 | 2 | imu.roll_cd | int16, centidegrees; 0 when `imu_valid` is 0 |
| 35 | 2 | diag.drops | uint16 |
| 37 | 2 | boot_count | uint16, incremented at every boot, saturating at 0xFFFF |
| 39 | 2 | tx_timeouts | uint16, saturating at 0xFFFF |
| 41 | 2 | battery_mv | uint16 |
| 43 | 2 | crc16 | uint16, CRC16-CCITT-FALSE over bytes 0-42 |

If the IMU-GPS offset does not fit in an int16, the encoder clears `imu_valid` and sends the IMU fields as 0 rather than truncating the offset.
Pairing already bounds the offset to 250 ms either way, so this path is a guard, not an expected case.

The frame stays at 45 bytes because of airtime, not tidiness.
With the airtime formula in [Results](#results) at SF12, BW125, CR4/8 with low-data-rate optimisation, the payload term is `ceil((8*PL - 4) / 40)` blocks of 8 symbols.
At 45 bytes that is 9 blocks, 80 payload symbols in total; anywhere from 46 to 50 bytes it is 10 blocks, 88 symbols.
Eight symbols at 32.768 ms each is 262 ms more airtime, and transmit energy, on every report for the life of the node.

## Gateway serial output

The gateway writes one line per event to its USB serial port at 115200 baud, comma-separated, so a host pipeline can parse it without a framing layer.
A radio receive that finds nothing waiting produces no output at all; that case is intentionally silent, not unlogged.

A successfully decoded, non-duplicate packet produces a `REC` line with these fields in order:

```
REC,node_id,seq,gps_time_ms,lat_1e7,lon_1e7,alt_mm,satellites,yaw_cd,pitch_cd,roll_cd,drops,crc_errors,rssi,gps_valid,imu_valid,boot_count,tx_timeouts,battery_mv
```

`gps_valid` and `imu_valid` (milestone 2), then `boot_count,tx_timeouts,battery_mv` (milestone 3), are appended at the end rather than inserted among the existing fields, so the field order a consumer already depends on stays stable.
`gps_valid` and `imu_valid` are each `1` or `0`.
`crc_errors` is always `0` here, for the same reason it is reserved in the wire format: a node only transmits and never decodes a CRC of its own.
v0x02 does not carry `crc_errors` at all and decodes it as `0`.
v0x01 does not carry the three milestone 3 fields, so v0x01 records print `0,0,0` for them.
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

A decoded packet the gateway has already seen produces no output.
Duplicates are keyed on `(node_id, boot_count, seq)`, in a 32-entry table.
`seq` restarts at 0 after every reset, so keyed on `(node_id, seq)` alone, the first packets after a quick watchdog reset would match entries still in the table from the previous boot and be discarded as duplicates.
Milestones 1 and 2 had no watchdog, so that took a power cycle; milestone 3 makes resets routine, which turned the edge case into a bug.
`boot_count` is kept in the SNVS low-power general purpose register, incremented at every boot, and sent in every v0x02 packet, so a new boot starts a new key space.
v0x01 packets decode with `boot_count = 0`, so for them the key behaves exactly as before.
`test_same_sequence_after_a_reboot_relays` in `test/test_mesh/` covers the reboot case.

`decode_error_count` and `short_read_count` are running totals kept by the gateway since power-on, capped at 65535.
The authoritative source for this format is `print_record()` and `loop()` in `src/gateway_main.cpp`.

## Results

### Radio timing profiles

`test/test_polling/`, `test/test_interrupt/` and `test/test_benchmark/` run both samplers against the same three simulated radio timing profiles, named for what they are:

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

The `tx drops` column is structurally zero for every polling row, not measured as zero: `PollingSampler` has no outbound queue, so there is nothing for it to count.
That sampler does lose fixes, though.
`PollingSampler::collect_gps_bytes()` drains the whole receive buffer, parses every complete sentence, and keeps only the last one; every earlier fix in that drain is discarded and counted nowhere.
With a 64-byte buffer barely one sentence survives a stall, so this was invisible in milestone 1; with 4096 bytes it dominates.
The figures below are derived from the published rows, not measured by any counter.
Sentences reaching the parser = sim_duration_ms x 0.96 B/ms / 68 bytes per sentence, which holds because `gps_rx_overflows = 0` means every byte reached the parser.

| Row | Sentences parsed (derived) | Packets sent | Fixes lost |
|---|---|---|---|
| `polling,4096,SF7` | ~847 | 588 | ~259, uncounted |
| `interrupt,4096,SF7` | ~847 | 544 | 306, counted |
| `polling,4096,SF12` | ~857 | 20 | ~837, uncounted |
| `interrupt,4096,SF12` | ~847 | 19 | 831, counted |

Three things to read out of it:

1. **Either change alone eliminates GPS byte loss. They are redundant for that purpose.**
   Polling with a 4096-byte buffer reports zero overflows at all three profiles, and the interrupt build reports zero at both buffer depths.
   The buffer was sized at 4096 bytes to cover 3023 ms x 0.96 B/ms = 2902 bytes of SF12 airtime, and it turns out that alone is sufficient without a non-blocking loop.
   "Interrupt-driven acquisition eliminated the data loss" is not a claim this data supports.
   A bigger `HardwareSerial` buffer would have done it.
   But the two are not equivalent in the way the zeros suggest: polling at 4096 swaps byte-level loss for an equally large fix-level loss that no counter reports.
   At SF12 it silently discards about 837 fixes, against the interrupt build's 831 counted drops.

2. **At SF7 the deeper buffer alone is the fastest configuration measured, and faster than the interrupt build.**
   Polling at 4096 reaches 9.79 packets/sec against the interrupt build's 9.07 at either depth, from a 5.99 baseline.
   The milestone's throughput claim is therefore withdrawn rather than softened; see [Why the interrupt build is slower at SF7](#why-the-interrupt-build-is-slower-at-sf7).

3. **At SF12 nothing moves the packet rate**, because the radio is the bottleneck: every configuration lands between 0.32 and 0.33 packets/sec.

**What the non-blocking loop actually buys**, none of which is throughput:

- At SF12, the interrupt build's 831 counted packet drops stand against about 837 fixes that `polling,4096` discards with no counter at all (derived above).
  Milestone 1's `polling,64` configuration lost 56948 bytes to overflow at SF12; those were counted, in every packet's `drops` field, but as bytes rather than as fixes.
  The loss is the same physics; what changes is that it is explicit, chosen by a stated policy, and attributable.
- Sequence numbers are assigned when a fix is queued rather than when it is sent, so a receiver sees a gap and can count what the node discarded.
  One caveat: a packet the radio refuses is re-queued at the tail, so it goes out behind newer packets and a receiver must tolerate out-of-order sequence numbers rather than read every inversion as loss.
- The outbound queue keeps the freshest fix rather than the oldest, which matters for position data.
- The loop is free to do other work.
  Milestones 3 through 5 need relay, duty cycling and sleep, and a superloop that blocks for three seconds on a transmit structurally cannot host them.

CONTROL is a null control rather than a result.
It is limited by the GPS sentence rate, not the radio, so a non-blocking loop has nothing to win there.
`test_control_profile_is_a_null_control` in `test/test_benchmark/` fails if that row gains more than 10%, because a gain there would mean the simulation was flattering the new sampler.

### Milestone 3: downtime

These are simulated measurements from the host test suite, not field data.
Reproduce them with:

```bash
pio test -e native -f test_power_bench -v
```

The suite is deterministic.
The lines below are from its first run.
The only deliberate rerun, to verify a harness assertion added after review (see [The null control](#the-null-control)), printed every `LIFE` and `DOWNTIME` line byte-identical to these.

#### What is measured

A node is **down** at time *t* if the gateway's newest record from it with `gps_valid = 1` arrived more than 600 000 ms (two 5-minute report intervals) before *t*, and from time 0 until its first valid record.
The rule is strictly "more than": a gap of exactly 600 000 ms is not down, so one missed report is tolerated by definition.
A record with `gps_valid = 1` and `imu_valid = 0` is not down, because the position, which is what flood mapping needs, still arrived.

Each down interval is attributed to one cause:

| Cause | How it is identified |
|---|---|
| `startup` | before the node's first valid record |
| `battery` | the node died; known to the simulation, not inferable at the gateway |
| `no_gps` | `gps_valid = 0` heartbeats arrived during the interval |
| `radio` | `tx_timeouts` is higher in the record that ends the interval |
| `reboot` | `boot_count` is higher in the record that ends the interval |
| `silent` | none of the above: the node is down and nothing in-band explains why |

Two builds are compared, both on the same simulated hardware as the milestone 2 benchmark (SF12, 3023 ms airtime, the same GPS sentence and byte rate):

- `interrupt` is the milestone 2 `node_interrupt` build: always powered, streaming fixes back-to-back, no watchdog, no heartbeat.
- `duty_cycled` is the milestone 3 `node_duty_cycled` build: one fix per 5-minute slot, Teensy asleep between slots, WDOG1 running.

Three experiments, each reported separately, because a single blended percentage is mostly a function of the scenario:

1. **Battery life.** No faults, 3200 mAh, run until the battery dies, at both Teensy sleep currents.
2. **Fault recovery.** Unlimited battery; each fault injected alone into an otherwise clean 24-hour run, at hour 2.
3. **Combined 30 days.** 3200 mAh, 6 mA Teensy sleep current, and this schedule: day 2 a lost DIO0 edge, day 5 a 2-hour sky blockage, day 8 IMU reads failing for 6 hours, day 11 a radio that wedges until power-cycled, day 14 an I2C hang.

The definitions, current table, battery, fault schedule and experiments were committed in the [milestone 3 design doc](docs/superpowers/specs/2026-09-29-milestone-3-power-management-design.md) before any measurement code ran.
No definition, current, schedule, threshold or run length was changed after seeing a result.
The changes made during implementation are listed, with their reasons, under "Revisions made while planning" in that document; none was made to move a result.

The output formats are:

```
LIFE,build,sleep_current_ua,died_at_ms,days
DOWNTIME,experiment,build,sleep_current_ua,cause,down_ms,scenario_ms,recovered
```

`sleep_current_ua` is 0 for the `interrupt` build, which never sleeps.
`recovered` is `1` or `0` in experiment 2 and `-` elsewhere.

#### The null control

With no faults and an unlimited battery, both builds must be down only at startup.
The run printed:

```
DOWNTIME,null_control,interrupt,0,startup,29130,86400000,-
DOWNTIME,null_control,interrupt,0,battery,0,86400000,-
DOWNTIME,null_control,interrupt,0,no_gps,0,86400000,-
DOWNTIME,null_control,interrupt,0,radio,0,86400000,-
DOWNTIME,null_control,interrupt,0,reboot,0,86400000,-
DOWNTIME,null_control,interrupt,0,silent,0,86400000,-
DOWNTIME,null_control,duty_cycled,6000,startup,29123,86400000,-
DOWNTIME,null_control,duty_cycled,6000,battery,0,86400000,-
DOWNTIME,null_control,duty_cycled,6000,no_gps,0,86400000,-
DOWNTIME,null_control,duty_cycled,6000,radio,0,86400000,-
DOWNTIME,null_control,duty_cycled,6000,reboot,0,86400000,-
DOWNTIME,null_control,duty_cycled,6000,silent,0,86400000,-
```

Both builds are down only for `startup`: 29130 ms for `interrupt` and 29123 ms for `duty_cycled`, consistent with the simulated GPS's 26 s cold start plus one 3023 ms SF12 transmission.
`test_null_control_shows_no_downtime_after_the_first_report` fails if either build shows any other cause, because that would mean the harness, not the firmware, is producing the number.
It also fails if the run ends down.
That second check was added after review: every interval before the first valid record is `startup`, so without it a harness that never delivered a single record would pass.
A deliberately broken run that delivered nothing confirmed the test now fails.

#### Experiment 1: battery life

```
LIFE,interrupt,0,45204206,0.52
LIFE,duty_cycled,6000,1323301745,15.32
LIFE,duty_cycled,25860,407101031,4.71
```

| Build | Teensy sleep current | Died at (ms) | Days |
|---|---|---|---|
| `interrupt` | never sleeps | 45204206 | 0.52 |
| `duty_cycled` | 6 mA | 1323301745 | 15.32 |
| `duty_cycled` | 25.86 mA | 407101031 | 4.71 |

The Teensy 4.1's Snooze `deepSleep` current has no data sheet value; the two figures are the two forum measurements the design doc cites, and the result is printed at both rather than at a chosen one.

**Where the duty-cycled build's energy goes.**
The output does not print an energy breakdown, but the two `duty_cycled` lines determine one, because the two runs differ only in the Teensy's sleep current.
Average current is capacity over lifetime:

- at 6 mA sleep: 3200 mAh x 3 600 000 ms/h / 1323301745 ms = 8.705 mA
- at 25.86 mA sleep: 3200 mAh x 3 600 000 ms/h / 407101031 ms = 28.298 mA

Raising the sleep current by 19.86 mA raised the average by 19.592 mA, so the Teensy is asleep 19.592 / 19.86 = 98.65% of the time.
That makes the Teensy asleep 0.9865 x 6 = 5.919 mA of the 8.705 mA average, **68.0% of the energy at 6 mA**, and 0.9865 x 25.86 = 25.511 mA of 28.298 mA, **90.2% at 25.86 mA**.
Everything else, the Teensy awake and every peripheral in every state, is the remaining 2.786 mA.
The awake 1.35% is about 4.05 s of every 300 s cycle, consistent with a 1 s GPS hot start plus 3023 ms of SF12 airtime.
Counting the Teensy awake at its 100 mA as well (100 x 0.01349 = 1.349 mA), the MCU draws 7.268 of the 8.705 mA, 83.5%, even at the lower sleep figure.
This derivation assumes the two runs spend the same share of time asleep; the one difference, a single 26 s cold start averaged over a shorter life in the 25.86 mA run, is not corrected for.

**So the Teensy 4.1's sleep current is the finding.**
The design doc committed in advance to saying so if the Teensy's sleep current dominated the energy budget, and it does.
Even with every peripheral and every awake second removed, a 3200 mAh cell under a sleeping Teensy lasts 3200 / 6 = 533 h, 22.2 days, at 6 mA, and 3200 / 25.86 = 124 h, 5.16 days, at 25.86 mA.
No change to the report schedule can take this build past those ceilings, and which ceiling applies depends on a current for which the design doc found no data sheet value.
Both sleep figures are whole-board forum measurements, and the 6 mA one was taken from a 5 V supply.
Whether that current comes from the MCU itself, the board around it (its regulator and power LED), or the sleep mode Snooze makes available is not separable here.
The sleep mode is itself a firmware choice: Snooze `hibernate()` was rejected in the design doc because it is reported never to wake on Teensy 4.1.
This milestone does not measure any alternative part, board or sleep mode.

For scale, the `interrupt` build averages 3200 mAh x 3 600 000 ms/h / 45204206 ms = 254.84 mA, consistent with the Teensy, GPS, IMU and a nearly always transmitting radio all on at once.
The `duty_cycled` build lasts 1323301745 / 45204206 = 29.3 times as long at 6 mA, and 407101031 / 45204206 = 9.0 times as long at 25.86 mA.
That compares a build designed to sleep against one that was never meant to; it measures the cost of never sleeping, not a flaw in the milestone 2 design.

#### Experiment 2: fault recovery

```
DOWNTIME,fault_lost_completion,interrupt,0,startup,29130,86400000,1
DOWNTIME,fault_lost_completion,interrupt,0,battery,0,86400000,1
DOWNTIME,fault_lost_completion,interrupt,0,no_gps,0,86400000,1
DOWNTIME,fault_lost_completion,interrupt,0,radio,0,86400000,1
DOWNTIME,fault_lost_completion,interrupt,0,reboot,0,86400000,1
DOWNTIME,fault_lost_completion,interrupt,0,silent,0,86400000,1
DOWNTIME,fault_sky_blockage,interrupt,0,startup,29130,86400000,1
DOWNTIME,fault_sky_blockage,interrupt,0,battery,0,86400000,1
DOWNTIME,fault_sky_blockage,interrupt,0,no_gps,0,86400000,1
DOWNTIME,fault_sky_blockage,interrupt,0,radio,0,86400000,1
DOWNTIME,fault_sky_blockage,interrupt,0,reboot,0,86400000,1
DOWNTIME,fault_sky_blockage,interrupt,0,silent,6578310,86400000,1
DOWNTIME,fault_imu_failing,interrupt,0,startup,29130,86400000,1
DOWNTIME,fault_imu_failing,interrupt,0,battery,0,86400000,1
DOWNTIME,fault_imu_failing,interrupt,0,no_gps,0,86400000,1
DOWNTIME,fault_imu_failing,interrupt,0,radio,0,86400000,1
DOWNTIME,fault_imu_failing,interrupt,0,reboot,0,86400000,1
DOWNTIME,fault_imu_failing,interrupt,0,silent,0,86400000,1
DOWNTIME,fault_radio_wedge,interrupt,0,startup,29130,86400000,0
DOWNTIME,fault_radio_wedge,interrupt,0,battery,0,86400000,0
DOWNTIME,fault_radio_wedge,interrupt,0,no_gps,0,86400000,0
DOWNTIME,fault_radio_wedge,interrupt,0,radio,0,86400000,0
DOWNTIME,fault_radio_wedge,interrupt,0,reboot,0,86400000,0
DOWNTIME,fault_radio_wedge,interrupt,0,silent,78599510,86400000,0
DOWNTIME,fault_hang,interrupt,0,startup,29130,86400000,0
DOWNTIME,fault_hang,interrupt,0,battery,0,86400000,0
DOWNTIME,fault_hang,interrupt,0,no_gps,0,86400000,0
DOWNTIME,fault_hang,interrupt,0,radio,0,86400000,0
DOWNTIME,fault_hang,interrupt,0,reboot,0,86400000,0
DOWNTIME,fault_hang,interrupt,0,silent,78602550,86400000,0
DOWNTIME,fault_lost_completion,duty_cycled,6000,startup,29123,86400000,1
DOWNTIME,fault_lost_completion,duty_cycled,6000,battery,0,86400000,1
DOWNTIME,fault_lost_completion,duty_cycled,6000,no_gps,0,86400000,1
DOWNTIME,fault_lost_completion,duty_cycled,6000,radio,0,86400000,1
DOWNTIME,fault_lost_completion,duty_cycled,6000,reboot,0,86400000,1
DOWNTIME,fault_lost_completion,duty_cycled,6000,silent,0,86400000,1
DOWNTIME,fault_sky_blockage,duty_cycled,6000,startup,29123,86400000,1
DOWNTIME,fault_sky_blockage,duty_cycled,6000,battery,0,86400000,1
DOWNTIME,fault_sky_blockage,duty_cycled,6000,no_gps,6925010,86400000,1
DOWNTIME,fault_sky_blockage,duty_cycled,6000,radio,0,86400000,1
DOWNTIME,fault_sky_blockage,duty_cycled,6000,reboot,0,86400000,1
DOWNTIME,fault_sky_blockage,duty_cycled,6000,silent,0,86400000,1
DOWNTIME,fault_imu_failing,duty_cycled,6000,startup,29123,86400000,1
DOWNTIME,fault_imu_failing,duty_cycled,6000,battery,0,86400000,1
DOWNTIME,fault_imu_failing,duty_cycled,6000,no_gps,0,86400000,1
DOWNTIME,fault_imu_failing,duty_cycled,6000,radio,0,86400000,1
DOWNTIME,fault_imu_failing,duty_cycled,6000,reboot,0,86400000,1
DOWNTIME,fault_imu_failing,duty_cycled,6000,silent,0,86400000,1
DOWNTIME,fault_radio_wedge,duty_cycled,6000,startup,29123,86400000,1
DOWNTIME,fault_radio_wedge,duty_cycled,6000,battery,0,86400000,1
DOWNTIME,fault_radio_wedge,duty_cycled,6000,no_gps,0,86400000,1
DOWNTIME,fault_radio_wedge,duty_cycled,6000,radio,600000,86400000,1
DOWNTIME,fault_radio_wedge,duty_cycled,6000,reboot,0,86400000,1
DOWNTIME,fault_radio_wedge,duty_cycled,6000,silent,0,86400000,1
DOWNTIME,fault_hang,duty_cycled,6000,startup,29123,86400000,1
DOWNTIME,fault_hang,duty_cycled,6000,battery,0,86400000,1
DOWNTIME,fault_hang,duty_cycled,6000,no_gps,0,86400000,1
DOWNTIME,fault_hang,duty_cycled,6000,radio,0,86400000,1
DOWNTIME,fault_hang,duty_cycled,6000,reboot,0,86400000,1
DOWNTIME,fault_hang,duty_cycled,6000,silent,0,86400000,1
```

Every row also carries the build's `startup` interval (29130 or 29123 ms), which is not the fault's cost and is left out of the table.
Minutes are `down_ms / 60000`, to two decimal places.

| Fault | Build | Recovered | Down (ms) | Down (min) | Cause |
|---|---|---|---|---|---|
| lost DIO0 edge | `interrupt` | 1 | 0 | 0 | - |
| 2-hour sky blockage | `interrupt` | 1 | 6578310 | 109.64 | `silent` |
| IMU failing 6 hours | `interrupt` | 1 | 0 | 0 | - |
| radio wedge | `interrupt` | **0** | 78599510 | not a result | `silent` |
| hang | `interrupt` | **0** | 78602550 | not a result | `silent` |
| lost DIO0 edge | `duty_cycled` | 1 | 0 | 0 | see below |
| 2-hour sky blockage | `duty_cycled` | 1 | 6925010 | 115.42 | `no_gps` |
| IMU failing 6 hours | `duty_cycled` | 1 | 0 | 0 | - |
| radio wedge | `duty_cycled` | 1 | 600000 | 10.00 | `radio` |
| hang | `duty_cycled` | 1 | 0 | 0 | see below |

**The `recovered` column is the result.**
`duty_cycled` recovers from all five faults.
`interrupt` recovers from three and does not recover from a radio that fails every attempt or from a hang, because it has neither a radio power cycle nor a watchdog.
For those two rows the minutes are only the time left in the 24-hour run after the fault, so they measure the run length, not the build, and are not reported as a result.

Three zeros need their explanation beside them, because read bare they claim more than they show:

- **`duty_cycled`, lost DIO0 edge, 0 down: a boundary result.**
  A diagnostic probe run during implementation, not part of the printed output, found the gap between the records either side of the lost report is exactly 600 000 ms, twice the 300 000 ms interval.
  The rule is strictly "more than 600 000 ms", so that gap is not down.
  The zero shows that a lost completion costs this build exactly one report, which the definition tolerates; it is not evidence of fast recovery, and a gap one millisecond longer would have printed 1.
- **`duty_cycled`, hang, `reboot` 0: the reboot happened.**
  The same probe saw `boot_count` go from 1 to 2, so WDOG1 did reset the node.
  The longest gap between its records was about 390 s, which never exceeded the 600 000 ms stale window, so no time was down and no time could be attributed to `reboot`.
  The zero means the reset cost no downtime, not that no reset occurred.
- **Both builds, IMU failing, 0 down: the definition working, not resilience.**
  Fixes kept arriving with `gps_valid = 1` and `imu_valid = 0`, which the definition does not count as down.
  This row confirms the IMU is excluded from downtime as designed; it does not measure IMU recovery.

The nonzero rows:

- **Sky blockage.** Both builds are down for most of the 2-hour blockage.
  If reports resumed the instant the sky cleared, the definition would give 7200000 - 600000 = 6600000 ms; `interrupt` measured 6578310 and `duty_cycled` 6925010.
  The output does not show why each lands where it does, and it has not been instrumented, so the 346700 ms difference between them is not interpreted here.
  What does differ by design is the cause: `duty_cycled` sends `gps_valid = 0` heartbeats, so the gateway sees `no_gps` and knows the node is alive; `interrupt` sends nothing, so the same outage is `silent`.
- **Radio wedge, `duty_cycled`: 600000 ms, set by policy.**
  The radio is power-cycled after `RADIO_FAIL_LIMIT` = 3 consecutive failed transmit states.
  Three lost reports make a gap of four intervals, 1200000 ms, of which the first 600000 ms is tolerated, which is exactly the printed figure; the diagnostic probe saw `tx_timeouts` reach 3 and one radio power cycle.
  The figure is set by `RADIO_FAIL_LIMIT` and the report interval, so it measures the recovery policy rather than the simulation.
  It is attributed to `radio` because `tx_timeouts` is higher in the record that ends it.
- **Lost DIO0 edge, `interrupt`: 0.** The existing 5000 ms `TX_TIMEOUT_MS` abandons the lost packet, and a build that transmits back-to-back loses seconds, far inside the stale window.

#### Experiment 3: combined 30 days

```
DOWNTIME,combined_30d,interrupt,0,startup,29130,2592000000,-
DOWNTIME,combined_30d,interrupt,0,battery,2546196470,2592000000,-
DOWNTIME,combined_30d,interrupt,0,no_gps,0,2592000000,-
DOWNTIME,combined_30d,interrupt,0,radio,0,2592000000,-
DOWNTIME,combined_30d,interrupt,0,reboot,0,2592000000,-
DOWNTIME,combined_30d,interrupt,0,silent,0,2592000000,-
DOWNTIME,combined_30d,duty_cycled,6000,startup,29123,2592000000,-
DOWNTIME,combined_30d,duty_cycled,6000,battery,1291405787,2592000000,-
DOWNTIME,combined_30d,duty_cycled,6000,no_gps,6925010,2592000000,-
DOWNTIME,combined_30d,duty_cycled,6000,radio,600000,2592000000,-
DOWNTIME,combined_30d,duty_cycled,6000,reboot,0,2592000000,-
DOWNTIME,combined_30d,duty_cycled,6000,silent,0,2592000000,-
```

| Cause | `interrupt` (ms) | `duty_cycled`, 6 mA (ms) |
|---|---|---|
| `startup` | 29130 | 29123 |
| `battery` | 2546196470 | 1291405787 |
| `no_gps` | 0 | 6925010 |
| `radio` | 0 | 600000 |
| `reboot` | 0 | 0 |
| `silent` | 0 | 0 |
| total | 2546225600 (98.23%) | 1298959920 (50.11%) |

Totals and percentages are sums of the printed causes over `scenario_ms` = 2592000000.
Both totals are mostly battery, which is why they are not the headline.

`battery` time starts at the later of the node's death and the moment it would first count as down, 600 000 ms after its last valid record (`compute_downtime()` in `lib/floodnet_core/src/downtime.cpp`).
So `scenario_ms - battery` is whichever of those two came later.

- **The `interrupt` row measures battery only.**
  `scenario_ms - battery` = 2592000000 - 2546196470 = 45803530 ms.
  Experiment 1 is the same run up to day 2, and it puts this build's death earlier than that, at 45204206 ms.
  So 45803530 must be the last valid record plus 600 000 ms: the last valid record arrived at 45203530 ms, 0.52 days.
  The first fault is on day 2, so none of the five faults ever reached this build; its zeros for `no_gps`, `radio`, `reboot` and `silent` are not evidence of anything.
- **The `duty_cycled` row saw every fault.**
  `scenario_ms - battery` = 2592000000 - 1291405787 = 1300594213 ms.
  Its only down time other than `startup` and `battery` is `no_gps` 6925010 and `radio` 600000, identical to experiment 2's single-fault figures, so it was never stale between the day 11 recovery and its death.
  Its last valid record therefore arrived at 1300594213 - 600000 = 1299994213 ms, 15.05 days, after the day 14 hang, and it died between that and 1300594213 ms.
  The lost edge, the IMU failure and the hang again cost nothing, for the reasons given under experiment 2.
  It died between 22707532 and 23307532 ms, about 0.26 to 0.27 days, sooner than experiment 1's 1323301745 ms; the only difference between the runs is the faults, so that is the energy they cost, and the output does not break it down by fault.

## Known limitations

Milestone 1's polling loop blocked on the IMU and on radio transmission.
GPS bytes arriving during those stalls landed in `HardwareSerial`'s default 64-byte software receive buffer, and when that buffer filled, they were gone.
That buffer is not a passive hardware register: it is filled by the Teensy UART receive interrupt, and the polling loop's `read_byte()` only drains what the interrupt already collected.
`test/test_polling/` asserts that this loss happens rather than papering over it, and the loss is reported in every outgoing packet's `drops` field, which carries the node's `IGpsSource::rx_overflows()` value.
That fixture keeps the 64-byte depth so the `polling,64` benchmark row remains comparable with milestone 1.

**`node_polling` as built today does not reproduce this.**
`TeensyGps::begin()` in `src/hal/teensy_gps.hpp` calls `addMemoryForRead()` unconditionally, and `src/main.cpp` constructs the single `TeensyGps` instance outside the `FLOODNET_SAMPLER_INTERRUPT`/`FLOODNET_SAMPLER_POLLING` branch, so both build targets supply the same 4096-byte buffer.
The two shipped firmware images therefore differ only in acquisition strategy (blocking vs. non-blocking), not in GPS buffer depth.
The milestone 1 configuration, polling with a 64-byte buffer, exists only as the `polling,64` row in the host simulation in [Results](#results), not as anything you can flash.
Someone who flashes `node_polling` expecting milestone 1's loss behaviour will not see it; they will see the `polling,4096` row instead.

The `drops` counter's unit is implementation-defined, and the two implementations in this repository do not measure the same thing.
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
A transmission that never reports completion is abandoned after 5000 ms and counted in `tx_timeouts()`, so a lost DIO0 edge costs one packet rather than the node.

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

### Milestone 3 simulation

- **Currents are taken at face value.** Every current in `test/support/current_model.hpp` is a typical data sheet value drawn straight from the cell, with no regulator efficiency and no regulator quiescent current, power LED or leakage.
  The Teensy 4.1's sleep current is not a data sheet value at all: it is a PJRC forum measurement, which is why experiment 1 is reported at two values, 6 mA and 25.86 mA.
  The 6 mA figure was measured from a 5 V supply (see the comment on it in `current_model.hpp`) and is charged to the cell unchanged.
- **Snooze's `millis()` compensation is wrong on Teensy 4, and `TeensyPower` corrects it.** In the bundled Snooze 6.3.9, `src/hal/TEENSY_40/SnoozeTimer.cpp` stores `period = seconds * 32768` and, on wake, advances `systick_millis_count` by `period / 1000`: 32.768 ms per slept second instead of 1000 ms.
  Left alone, a node would believe a 60 s sleep lasted about 2 s and its 5-minute schedule would stretch about thirty-fold.
  `TeensyPower::sleep_until()` in `src/hal/teensy_power.hpp` therefore sleeps whole seconds only, adds the missing milliseconds to `systick_millis_count` itself after each wake, and covers the sub-second remainder with `wfi`.
  The library is not patched or vendored, and the effect is read from source, not observed on a board.
- **Faults land up to 60 s late for `duty_cycled`.** `PowerRig` applies faults between node steps, and a sleep chunk of up to `SLEEP_CHUNK_MS` (60 000 ms) is a single step, so a fault scheduled inside a sleep starts when that chunk ends.
  The `interrupt` build never sleeps, so its faults start on the millisecond.
- **`battery_mv` in simulation is a placeholder.** It falls in a straight line from 4200 mV at full to 3000 mV at empty.
  That is telemetry plumbing, not a discharge model, and no result above depends on it.
- **The experiments run on every push.** CI runs `pio test -e native`, which includes `test_power_bench`.
  In the saved experiment output `test_power_bench` took 18.44 s, and the whole native suite of 189 test cases took about 24 to 28 s across the runs recorded on the development machine; a CI runner's time will differ.

### Recovery exists only in `node_duty_cycled`

A radio that wedges on every attempt, rather than losing one completion, is now recovered, but only by `node_duty_cycled`.
It power-cycles and re-initialises the radio after `RADIO_FAIL_LIMIT` = 3 consecutive transmit states that end with no completed transmit.
It also runs WDOG1 with a 90 000 ms timeout, kicked on every state transition and on nothing else, so a loop that keeps spinning while stuck in one state is still reset; the 5-minute sleep is taken in 60 s chunks to stay under WDOG1's 128 s ceiling.

`node_polling` and `node_interrupt` have neither, and a persistently wedged radio or a hang still leaves them dead until power-cycled; experiment 2's `recovered` column shows it.
Both still bound a single lost completion.
In `node_polling`, `TeensyRadio::transmit()` gives up after its 5000 ms `waitPacketSent()` timeout (see `src/hal/teensy_radio.hpp`).
`node_interrupt` does not use that path.
`TeensyRadio::tx_busy()` returns `driver_.mode() == RHModeTx`, which only RadioHead's DIO0 handler clears, so without a deadline a single lost DIO0 edge would leave the node permanently mute.
`InterruptSampler` therefore abandons any transmission still in flight after `TX_TIMEOUT_MS` (5000 ms, the same bound), forces the radio idle through `IAsyncRadio::abort_transmit()`, and counts it in `tx_timeouts()`.
`node_duty_cycled` inherits that path from the sampler it wraps.
`test/test_interrupt/` drops one completion edge and asserts the node recovers and the sequence accounting stays closed.

### Not validated on hardware

No board has been available for milestones 2 or 3.
Every figure here is from the host simulation, and the firmware is compiled but never run on a Teensy in CI.
Specifically unverified:

- The `IntervalTimer` IMU path and `TeensyRadio::abort_transmit()`'s `setModeIdle()` recovery, exercised only against fakes.
- **Whether WDOG1 counts through a Snooze `deepSleep` at all.** This is the first bench test to run.
  In the bundled Snooze 6.3.9, `hal_deepSleep()` in `src/hal/TEENSY_40/hal.c` (line 777) rewrites `CCM_CCGR3` at line 805, keeping only the ACMP1-4 clock gates and `0x10000000`.
  That clears `CCM_CCGR3_WDOG1`, bits 17-16 per `imxrt.h`, gating WDOG1's clock until line 851 restores the register on wake.
  If WDOG1 stops counting while gated, a GPT wake that never fires would hang the node with no reset.
  That is the failure the design meant to rule out by leaving WDOG1's low-power suspend bit, `WCR[WDZST]`, clear; the clock gate may reintroduce it by another route.
  This is read from source; it has not been observed on a board.
- Snooze `deepSleep` timer wake alongside WDOG1 on current Teensyduino in general.
- The real sleep current of this board with these peripherals attached, which experiment 1 shows sets the battery life.
- `UBX-RXM-PMREQ` software backup and hot start on the actual NEO-M8N.
- SNVS `LPGPR` register retention across a WDOG1 reset, which `boot_count` and therefore the gateway's dedup key depend on.
- The data sheet currents themselves: they are typical values, and a real board adds regulator quiescent current, the power LED, and leakage.

## Deferred to later milestones

- **Mesh relay.** `should_relay()` and `prepare_relay()` in `lib/floodnet_core/include/floodnet/mesh.hpp` are complete and unit-tested, but nothing outside `test/test_mesh/` calls them; no node currently relays another node's packet.
  Milestone 3 duty-cycles a node's own reports only, and powers the radio only to transmit.
  A relay also needs a receive schedule, so nodes can hear each other without transmitting over each other, and milestone 3 did not build one.
- **Ring buffer.** `lib/floodnet_core/include/floodnet/ring_buffer.hpp` exists, is lock-free, and is fully tested, but it sits on no data path.
  Mapping each acquisition path onto the hardware left it without a consumer: `HardwareSerial` owns the GPS receive interrupt, RadioHead owns the radio's, the IMU handler can only set a flag because retrieving a sample needs a blocking I2C transaction, and the outbound queue needs drop-oldest, which single-producer single-consumer ordering forbids.
  It is kept and labelled rather than quietly wired into a path that does not need it.
- **Sequence-gap counting.** Deferred to milestone 5.
  It is a receiver-side concern and belongs with the gateway/host tooling work planned there.
- **Low-battery load shedding.** Deliberately left out of milestone 3: a policy such as "report every 20 minutes below 3.5 V" would move the downtime figure, and one fixed policy measured honestly is worth more than a tunable one.
  `battery_mv` travels in-band in v0x02 so a later milestone has the data to design one, though in simulation it is still only the placeholder described above.
- **Host HAL trace replay.** The design spec calls for a host HAL implementation that replays recorded sensor traces from disk.
  What shipped instead is the header-only synthetic fakes under `test/support/`, which repeat one hardcoded NMEA sentence at a fixed byte rate.
  That substitution is a reasonable milestone-1 choice, sufficient for the timing-driven tests this milestone needs, but it is not what the spec describes.
  Recorded-trace replay is planned to arrive with the hardware-in-the-loop tooling in milestone 4.
- **Hardware-in-the-loop validation** of everything listed in [Not validated on hardware](#not-validated-on-hardware), starting with whether WDOG1 counts through `deepSleep`.

## Hardware

See [docs/hardware.md](docs/hardware.md).

## About this repository

A clean rewrite of firmware I built for a Duke research lab between 2024 and 2026.
The original is on lab infrastructure and is not public, so this is written from scratch against the same requirements.
