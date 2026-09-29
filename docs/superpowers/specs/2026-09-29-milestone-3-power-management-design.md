# FloodNet Milestone 3: Power Management and Fault Recovery

Date: 2026-09-29
Status: implemented
Parent spec: `docs/superpowers/specs/2026-09-20-floodnet-firmware-design.md`

## Purpose

Milestone 2 left a node that never blocks but never sleeps either.
It streams every fix it can, keeps every peripheral powered, and has no way back from a wedged radio or a hung bus short of a power cycle.

Milestone 3 adds the node state machine the parent spec calls for in `node_state.hpp`, drives sleep and duty cycling from it, adds the hardware watchdog the parent spec's error-handling section requires, and measures the result as sensor downtime.

No hardware is available.
Every figure this milestone produces comes from the host simulation, and every current it uses comes from a datasheet or a named measurement, cited next to the constant.

## Definitions

**Report interval.** A node delivers one fix every 5 minutes.
Flood water rises over tens of minutes, so a 5-minute cadence tracks it without spending energy on redundant fixes.

**Down.** A node is down at time *t* if the gateway's newest record from it with `gps_valid = 1` was received more than 10 minutes (two report intervals) before *t*.
One lost report is tolerated; two consecutive are not.
A node is also down from time 0 until its first valid record arrives.

**Downtime.** Total down time divided by scenario length.

A record with `gps_valid = 1` and `imu_valid = 0` is not down.
The node's position, which is what flood mapping needs, still arrived.
IMU failures are therefore reported by their own rate, not as downtime, and this document does not pretend otherwise.

## Architecture

The same three-layer split as milestones 1 and 2.
Nothing under `lib/` includes a hardware header, and CI's existing grep keeps enforcing that.

### floodnet_core

**`node_state.hpp` / `node_state.cpp`: `NodeStateMachine`.**
A pure transition function: it takes the current state, an event, and the current time, and returns the next state plus a set of actions.
It holds no HAL reference, so every transition is unit-tested with hand-built event sequences, the way `mesh.cpp` is.

| State | Powered | Leaves on | Next state |
|---|---|---|---|
| `BOOT` | all | peripherals initialised | `ACQUIRE` |
| `ACQUIRE` | GPS, IMU | one valid fix queued, or `ACQUIRE_TIMEOUT_MS` elapsed | `TRANSMIT` |
| `TRANSMIT` | radio | queue empty and nothing in flight, or `TRANSMIT_TIMEOUT_MS` elapsed | `SLEEP` |
| `SLEEP` | nothing | next scheduled slot, or a watchdog chunk boundary | `ACQUIRE`, or `SLEEP` again |

Events: `PeripheralsReady`, `FixQueued`, `FixQueuedNoImu`, `TxSucceeded`, `TxFailed`, and `Tick`, which carries the time and is how timeouts, slot wakes, and watchdog chunk boundaries are detected.
Actions: the power plan for the new state, `queue_heartbeat`, `kick_watchdog`, `sleep_until_ms`, `power_cycle_radio`, `power_cycle_imu`.

Decisions inside the state machine:

- **One fix per wake.** `ACQUIRE` ends in the pass where the sampler's `next_seq()` advances.
  The simulated GPS completes at most one sentence per 1 ms pass, and a real NEO-M8N outputs at 1 Hz, so exactly one packet is queued.
  Queuing more would cost 3 s of SF12 airtime each.
- **Anchored schedule.** The next slot is the previous slot plus 5 minutes, not the end of this cycle's work plus 5 minutes.
  A slow acquisition therefore delays one report, not every report after it.
- **Heartbeat on timeout.** If `ACQUIRE_TIMEOUT_MS` passes with no valid fix, the node queues a packet with `gps_valid = 0` and transmits it.
  It still counts as down, but the gateway can tell a node that cannot see the sky from a node that is dead.
- **Every state transition kicks the watchdog, and nothing else does.** A loop that keeps spinning while stuck in one state is still reset.

**`downtime.hpp` / `downtime.cpp`.**
A pure function over the gateway's timestamped records that returns total down time and its split by cause (see "Measurement").
Unit-tested with hand-built record sequences, independent of any simulation.

**`packet.hpp` / `packet.cpp`.**
Gains a v0x02 encoder and a version-dispatching decoder (see "Wire format").
The v0x01 encoder is unchanged.

**`mesh.hpp` / `mesh.cpp`.**
`DedupTable` keys on `(node_id, boot_count, seq)` instead of `(node_id, seq)` (see "Identity across reboots").

### floodnet_hal

Three new interfaces.

**`IPower`.**

```cpp
enum class Peripheral : uint8_t { Gps, Imu, Radio };
enum class PowerState : uint8_t { On, Low };

class IPower {
  public:
    virtual void set_power(Peripheral p, PowerState s) = 0;
    virtual bool power_cycle(Peripheral p) = 0;       // off, on, and re-initialise the driver
    virtual void sleep_until(uint32_t wake_ms) = 0;  // MCU lowest RAM-retaining mode
    virtual uint16_t battery_mv() = 0;
};
```

`Low` means each part's own low-power command, not a load switch, so the bill of materials does not change:

| Part | `Low` implementation | Why that mode |
|---|---|---|
| NEO-M8N | software backup via `UBX-RXM-PMREQ` | keeps ephemeris in backup RAM, so the next wake is a hot start |
| BNO055 | suspend mode through the `PWR_MODE` register | lowest mode that keeps configuration |
| RFM95W | `RH_RF95::sleep()` | datasheet sleep mode |

**`IWatchdog`.** `begin(timeout_ms)` and `kick()`.

**`IPersistentStore`.** `read_u32(slot)` and `write_u32(slot, value)` for values that must survive a watchdog reset.
One slot is used, for `boot_count`.

### Application

**`src/duty_cycled_node.hpp` / `.cpp`: `DutyCycledNode`.**
Composes `NodeStateMachine`, `InterruptSampler`, and the three new HAL interfaces.
While awake it calls `sampler.step()` exactly as `node_interrupt` does today; the state machine decides only what is powered and when the sampler runs.
Before powering the GPS on it drains every byte still in the receive buffer, so a sentence left over from the previous wake cannot be parsed and stamped with the new wake's time.

`InterruptSampler` gains four additive members and no changed behaviour: a constructor argument selecting the wire version (defaulting to v0x01), `set_node_status(boot_count, battery_mv)`, `enqueue_heartbeat()`, and `last_record_imu_valid()`.
The sampler encodes packets inside `service_radio()`, so a v0x02 node cannot be built around it without some change, and these four are the smallest set that serves it.
With the defaults, `node_interrupt` behaves exactly as before; the benchmark rerun, which must reproduce every published row bit for bit, is the check.

**Teensy implementations** in `src/hal/`: `TeensyPower`, `TeensyWatchdog`, `TeensyPersistentStore`.

- `TeensyWatchdog` uses WDOG1.
  Its timeout field `WCR[WT]` counts in 0.5 s steps up to 128 s (the Linux `imx2_wdt` driver's `IMX2_WDT_MAX_TIME` is 128).
- `TeensyPersistentStore` uses the SNVS low-power general purpose registers (`SNVS->LPGPR`), which the i.MX RT1060 reference manual, section 19.4.2.1, documents as retained across resets while SNVS is powered.
- `TeensyPower::sleep_until()` uses the Snooze library's `deepSleep()` with a timer wake.
  Snooze `hibernate()` is rejected: on Teensy 4.1 it is reported to sleep and never wake (PJRC forum, "Teensy 4.1 deep sleep and watchdog").
  Powering off through the On/Off pin with an RTC alarm is also rejected: every wake would be a cold boot, which would put a reboot into every report cycle and change `boot_count` from a fault signal into a cycle counter.
  Whether Snooze's timer wake works cleanly alongside WDOG1 on current Teensyduino is unverified without a board, and is recorded as a hardware risk.

**Build target.** A new PlatformIO environment, `node_duty_cycled`, defines `FLOODNET_NODE_DUTY_CYCLED` and sends v0x02 packets.
`node_polling` and `node_interrupt` are unchanged and keep sending v0x01.
CI compiles all four firmware targets.

## Watchdog timing

| Constant | Value | Reason |
|---|---|---|
| `ACQUIRE_TIMEOUT_MS` | 60 000 | Over twice the NEO-M8N's 26 s typical cold-start time to first fix (GPS + GLONASS, NEO-M8 datasheet UBX-15031086, TTFF table). A hot start is 1 s. |
| `TX_TIMEOUT_MS` | 5 000 | Existing sampler constant, unchanged. |
| `TRANSMIT_TIMEOUT_MS` | 10 000 | Twice `TX_TIMEOUT_MS`, so a radio that refuses `begin_transmit()` outright cannot hold the node in `TRANSMIT`. Counts as a failed cycle. |
| `WATCHDOG_TIMEOUT_MS` | 90 000 | Above the longest legitimate time between two transitions (`ACQUIRE_TIMEOUT_MS`), below WDOG1's 128 s ceiling. |
| `SLEEP_CHUNK_MS` | 60 000 | A 5-minute sleep exceeds the watchdog ceiling, so `SLEEP` wakes every 60 s, kicks, and sleeps again. |

WDOG1 can instead be suspended in low-power modes (`WCR[WDZST]`, exposed by the WDT_T4 library as `lp_suspend`).
That is rejected: a wake timer that never fires would then hang the node with the watchdog paused, which is exactly the failure the watchdog exists for.
The cost of the chosen approach is four extra wakes per report cycle, each only long enough to kick, and the simulation charges them.

## Faults and recovery

| Fault | Detected by | Recovery | Visible at the gateway as |
|---|---|---|---|
| Lost DIO0 edge, one packet | sampler's existing `TX_TIMEOUT_MS` | abandon the packet, sleep as normal | missing report; `tx_timeouts` increments in-band |
| Radio wedged, every attempt fails | `RADIO_FAIL_LIMIT` = 3 consecutive `TRANSMIT` states that end with no completed transmit | `PowerCycle(Radio)`, then `begin()` | reports resume; `tx_timeouts` shows the gap's cause |
| IMU reads fail | the wake's fix is queued with `imu_valid = 0` | send the fix with `imu_valid = 0`; `PowerCycle(Imu)` after 3 consecutive failed wakes | `imu_valid = 0` |
| No GPS lock | `ACQUIRE_TIMEOUT_MS` | heartbeat, keep the schedule | `gps_valid = 0` heartbeats |
| Hang, for example an I2C read that never returns | WDOG1 | full MCU reset | `boot_count` increments |

Out of scope, deliberately: low-battery load shedding.
A policy such as "report every 20 minutes below 3.5 V" would move the downtime figure, and one fixed policy measured honestly is worth more than a tunable one.
`battery_mv` travels in-band so a later milestone has the data to design one.

## Identity across reboots

`seq` restarts at 0 after any reset, and the gateway's `DedupTable` keys on `(node_id, seq)`.
After a quick reboot, the first packets can match entries still in the 32-entry table and be discarded as duplicates.
Milestones 1 and 2 had no watchdog, so a reset needed a power cycle; milestone 3 makes resets routine, which turns this from an edge case into a bug.

The fix: `boot_count` is incremented at every boot in `IPersistentStore`, travels in every v0x02 packet, and joins the dedup key as `(node_id, boot_count, seq)`.
A test reproduces the suppression with the current key before the key changes.

## Wire format v0x02

Three new facts must travel in-band for the gateway to attribute downtime: `boot_count`, `tx_timeouts`, and `battery_mv`.

Growing the packet costs airtime.
With the formula in the README at SF12, BW125, CR4/8 and low-data-rate optimisation, the payload term is `ceil((8*PL - 4) / 40)` blocks of 8 symbols.
At PL = 45 that is 9 blocks (80 symbols in total); anywhere from 46 to 50 bytes it is 10 blocks (88 symbols), 262 ms more airtime and transmit energy on every report for the life of the node.

v0x02 therefore stays at exactly 45 bytes, funding the 6 new bytes from fields that carry nothing:

| Removed from v0x01 | Bytes | Why it is safe |
|---|---|---|
| `diag.crc_errors` | 2 | Reserved and always 0 in every milestone. |
| `gps.valid`, `imu.valid` bytes | 2 | Become bits 0 and 1 of `flags`, which is always 0 and never read today. |
| `imu.time_ms` (u32), replaced by `imu_skew_ms` (int16) | 2 | IMU time minus GPS time. Pairing already bounds it to ±250 ms; 0 when `imu_valid = 0`. |

The six freed bytes carry `boot_count` (u16), `tx_timeouts` (u16, saturating at 0xFFFF), and `battery_mv` (u16).
Exact offsets are fixed by the implementation plan and pinned by a new golden-vector test written before the encoder, and the README's byte table is regenerated from the code as its existing rule requires.

`tx_queue_drops` stays out of band: with one fix per wake the queue cannot overflow.

## Gateway

- Dispatches on the version byte and decodes v0x01 and v0x02.
- Appends `boot_count,tx_timeouts,battery_mv` to the end of the `REC` line, keeping the existing field order stable.
  v0x01 records print `0,0,0`.
- Dedups on `(node_id, boot_count, seq)`; v0x01 packets have `boot_count = 0`.

## Simulation

### Powered fakes

`FakeGps`, `FakeImu`, and `FakeRadio` gain a powered flag, defaulting to on.
Unpowered, a fake emits nothing, reports `pending() == false`, and ignores elapsed time.
Because the default is on, milestone 1 and 2 tests and benchmark rows must be bit-identical; the benchmark rerun is the check, not an assumption.

`FakeGps` models time to first fix.
After power-on it emits `valid = 0` sentences for 1 s (hot start) if its last valid fix is less than 2 hours old, and for 26 s (cold start) otherwise.
Ephemeris age, not time spent in backup, is what decides a hot start, which matters after the 2-hour sky blockage.
The NEO-M8 datasheet gives the two TTFF figures but not an ephemeris lifetime, so 2 hours is a stated assumption.
A sky-blockage fault makes it emit only `valid = 0` sentences for its duration.

### Fast-forward

`SimClock` gains `advance_to(t)`, which delivers one `on_tick(t - now)` instead of `t - now` one-millisecond ticks.
Awake time still advances 1 ms at a time, exactly as today.
A 30-day scenario is about 8 640 report cycles of a few seconds awake each, plus the 60 s watchdog chunks, so a run takes seconds.

### Energy accounting

`FakePower` integrates `current(component, state) × time` into a battery, in charge (mAh).
Currents are taken at face value from the battery; regulator efficiency and quiescent current are not modelled, and the README says so.
When the battery reaches zero the node stops and never restarts.

`CurrentModel`, one table, each constant cited in a comment:

| Component | State | Current | Source |
|---|---|---|---|
| Teensy 4.1 | awake, 600 MHz | 100 mA | PJRC Teensy 4.1 product page (the sentence there names the Teensy 4.0; same MCU and clock) |
| Teensy 4.1 | Snooze `deepSleep` | 6 mA | PJRC forum measurement, "Teensy 4.1 deep sleep and watchdog". Not a datasheet value. |
| NEO-M8N | acquisition | 25 mA | NEO-M8 datasheet, Table 11, GPS, average from start-up to first fix |
| NEO-M8N | tracking | 23 mA | NEO-M8 datasheet, Table 11, continuous mode, GPS |
| NEO-M8N | software backup | 30 µA | NEO-M8 datasheet, `I_SWBCKP`, VCC = 3 V |
| BNO055 | NDOF normal | 12.3 mA | BNO055 datasheet BST-BNO055-DS000 |
| BNO055 | suspend | 40 µA | BNO055 datasheet BST-BNO055-DS000 |
| RFM95W | transmit, +20 dBm PA_BOOST | 120 mA | RFM95/96/97/98W datasheet, Table 51 |
| RFM95W | standby | 1.6 mA | RFM95/96/97/98W datasheet, Table 51 |
| RFM95W | sleep | 0.2 µA | RFM95/96/97/98W datasheet, Table 51 |
| Battery | capacity | 3200 mAh | Panasonic NCR18650B datasheet, rated minimum |

The Teensy's sleep current is the least certain number here and has no official source.
A second forum measurement of Snooze `deepSleep` reports 25.86 mA.
The battery-life result is therefore printed at both measured values, 6 mA and 25.86 mA.

### Faults and resets

A `FaultSchedule` injects faults into the fakes at scripted times.
A hang is modelled as an IMU read that never returns: `FakeImu::read()` keeps advancing the clock until `FakeWatchdog` expires, then returns with a reset flag set.
The harness sees the flag, destroys the node, and reconstructs it, keeping only `FakePersistentStore`, the way RAM is lost and SNVS survives on the real chip.
A node without a watchdog (the milestone 2 build) stays hung until the scenario ends.

## Measurement

### Cause attribution

Each down interval is attributed to one cause:

| Cause | How it is identified |
|---|---|
| `battery` | node died; known to the simulation, not inferable at the gateway |
| `no_gps` | `gps_valid = 0` heartbeats arriving during the interval |
| `radio` | node alive, nothing arriving, `tx_timeouts` higher in the next record |
| `reboot` | `boot_count` higher in the next record |
| `silent` | none of the above: the node is down and nothing in-band explains why |

### Experiments

Three experiments, each run on the milestone 2 `node_interrupt` build and the new `node_duty_cycled` build.
They are reported separately because a single blended percentage is mostly a function of the scenario.

1. **Battery life.** No faults, 3200 mAh, run until the battery dies.
   Reported in days, at both Teensy sleep currents.
2. **Fault recovery.** Unlimited battery, so a fault's cost is not hidden behind an early battery death.
   Each fault injected alone into an otherwise clean 24-hour run, at hour 2.
   Reported as downtime caused by that fault, in minutes, plus whether the node recovered before the run ended.
   For a build that never recovers, the minutes are only the time left in the run, so the `recovered` column is the result and the minutes are not.
3. **Combined 30 days.** 3200 mAh and this fault schedule:
   - day 2: lost DIO0 edge
   - day 5: 2-hour sky blockage
   - day 8: IMU reads fail for 6 hours
   - day 11: radio wedges until power-cycled
   - day 14: I2C hang

   Reported as total downtime and its split by cause, with the scenario printed alongside.

Output lines follow the `BENCH` convention:

```
DOWNTIME,experiment,build,sleep_current_ua,cause,down_ms,scenario_ms,recovered
```

`sleep_current_ua` is 0 for the milestone 2 build, which never sleeps.
`recovered` is `1` or `0` in experiment 2 and `-` elsewhere.

### Rules

1. **Frozen before the first run.** The definitions, current table, battery, fault schedule, and experiments above are committed in this document before any measurement code runs.
   Anything changed after seeing a result is recorded in the README as changed after seeing the data, with the reason.
2. **Null control.** With no faults and an unlimited battery, both builds must show zero downtime after their first report.
   A test fails if either build shows downtime there, because that would mean the harness, not the firmware, is producing the number.
3. **The number is whatever comes out.** It is published as measured.
   If the Teensy's sleep current dominates the energy budget, that is the finding, and the README says so.

## Testing

- `NodeStateMachine`: every transition, the anchored schedule across a slow acquisition, watchdog chunking, `millis()` wrap during sleep, and each recovery path.
- `downtime`: hand-built record sequences for each cause, the start-of-run case, and a record exactly at the 10-minute boundary.
- Packet v0x02: a golden vector written before the encoder, round trips, v0x01 still decoded, and corruption rejected.
- `DedupTable`: the reboot suppression reproduced under the old key, then fixed.
- Powered fakes: unpowered fakes ignore time; TTFF hot and cold; the existing suites unchanged.
- `DutyCycledNode` against fakes: one packet per wake, heartbeat on timeout, each fault recovered.
- Benchmark: milestone 1 and 2 rows bit-identical to the README's published figures.

## Not validated on hardware

Nothing in this milestone runs on a Teensy.
Specifically unverified: Snooze `deepSleep` wake alongside WDOG1 on current Teensyduino, the real sleep current of this board with these peripherals attached, `UBX-RXM-PMREQ` backup and hot start on the actual module, and the SNVS register retention across a WDOG1 reset.
Datasheet currents are typical values, and a real board adds regulator quiescent current, the power LED, and leakage.

## Revisions made while planning

Recorded here because the design above was approved before these were found.
All were made before any measurement code existed except the last three, which say when they were made in their own entries.
None changes the frozen scenario.

- **`InterruptSampler` is extended, not left untouched.** See "Application". The guarantee for milestone 2's figures moves from "by construction" to "by default arguments, confirmed by a bit-identical benchmark rerun".
- **`IPower::power_cycle()`.** Re-initialising a driver after power-cycling it is driver-specific, so it belongs behind the interface rather than in `DutyCycledNode`.
- **`TRANSMIT_TIMEOUT_MS`.** Without it, a radio that refuses every `begin_transmit()` would keep the node in `TRANSMIT` until the watchdog fired.
- **Cause `hung` renamed `silent` and redefined.** The gateway cannot tell a hung node from any other silent one, and a milestone 2 node under a sky blockage is silent too, since it sends no heartbeat.
- **`recovered` column** in experiment 2, for the reason given there.
- **IMU failure detection** uses the queued record's `imu_valid` rather than a read-failure counter, which also covers a sensor that answers but never inside the pairing window.
- **GPS drain before power-on**, described under "Application".
- **Snooze `millis()` compensation is wrong on Teensy 4.** Found by reading the bundled Snooze 6.3.9 source, `src/hal/TEENSY_40/SnoozeTimer.cpp`: `setTimer()` takes whole seconds and stores `period = seconds * 32768`, and the wake handler advances `systick_millis_count` by `period / 1000`, which is 32.768 ms per slept second instead of 1000 ms.
  Left alone, a node would believe a 60 s sleep lasted about 2 s and its 5-minute schedule would stretch about thirty-fold.
  `TeensyPower::sleep_until()` therefore sleeps whole seconds only, adds the missing milliseconds to `systick_millis_count` itself after each wake, and covers the sub-second remainder with `wfi`.
  The library is not patched or vendored.
  The effect is read from source, not observed on a board.
- **Battery sense.** `battery_mv` needs a divider the bill of materials did not have: two 100 kΩ resistors from the cell to Teensy pin 14 (A0).
  In simulation, `battery_mv` falls linearly from 4200 mV at full to 3000 mV at empty.
  That line is a telemetry placeholder, not a discharge model, and no reported result depends on it.
- **Cause `startup`.** The definition counts a node as down until its first valid record; `startup` names that interval so it is not reported as `silent`.
- **`LIFE` lines.** Experiment 1 reports a lifetime, printed as `LIFE,build,sleep_current_ua,died_at_ms,days`, rather than forcing it into the `DOWNTIME` shape.
- **`FakeGps` fix detection.** The fake marks a fix as observed when the `\r` that ends the fix sentence is emitted, not on the wrap back to byte 0.
  `NmeaLineAssembler` completes a sentence on that `\r`, so the plan's literal condition left the fake in cold start forever whenever the node cut GPS power the instant it saw a fix.
  Made during implementation, before any measurement code ran.
- **Experiments 2 and 3 assert completion only.** Each run asserts that it reached its end time or a depleted battery, and nothing about a measured value.
  A test that asserts nothing is a defect, and asserting on a measured value would be tuning.
  Made while writing the experiment code, before its first run.
- **Null control also asserts the run ended fresh.** It checks `down_at_end` is false as well as all downtime being `startup`.
  Without it, a harness that never delivers a valid record passes vacuously, because every interval before the first valid record is attributed to `startup`.
  Added after review of the experiment code; the rerun that verified it printed every `LIFE` and `DOWNTIME` line byte-identical to the first run, so no measured value changed.
- **Benchmark identity check compares the first 12 `BENCH` lines.** `pio test -v` echoes 4 of the existing rows a second time, so an unfiltered diff reports 4 lines even on an untouched tree.
  Made during implementation; it changes how the check reads the output, not what the benchmark measures.

## Out of scope

Mesh relay wiring, sequence-gap counting at the gateway, low-battery load shedding, host trace replay, and hardware-in-the-loop tooling.
