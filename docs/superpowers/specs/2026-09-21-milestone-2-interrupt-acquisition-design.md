# FloodNet Milestone 2: Interrupt-Driven Acquisition

Date: 2026-09-21
Status: approved, pending implementation
Parent spec: `docs/superpowers/specs/2026-09-20-floodnet-firmware-design.md`

## Purpose

Milestone 1 shipped a polling superloop that blocks on each sensor in turn and loses GPS bytes while it waits.
Milestone 2 makes acquisition non-blocking, adds the one interrupt path the hardware does not currently have, and measures the result against the milestone 1 baseline.

Both acquisition strategies remain selectable build targets, as the parent spec requires.
The comparison between them is the reason this repository exists, so neither build may be allowed to rot.

## What is actually changing

The parent spec's implementation sequence calls this milestone "interrupt-driven acquisition", which overstates it.
Two of the three peripherals are already interrupt-served in milestone 1, as the README's "What is and is not already interrupt-driven" section records.

What milestone 2 actually does:

1. Replaces the blocking drain with per-stream SPSC ring buffers and a non-blocking loop.
2. Adds the one genuinely missing interrupt path, on the BNO055 data-ready line.
3. Makes radio transmission asynchronous so the loop stops spinning on a completion an ISR has already recorded.
4. Replaces silent, byte-level loss with explicit, counted, policy-chosen packet drops.

Item 4 is the substance of the milestone, and it is not a throughput claim.
See "What this milestone claims" below.

## Corrections to the parent spec

**Ring buffer overflow policy.**
The parent spec states that overflow "drops the oldest data and increments the counter rather than blocking the producer, because the producer may be an interrupt handler".
Dropping the oldest is not implementable in a lock-free SPSC ring buffer.
It requires the producer to advance the consumer's index, which is exactly what the single-producer single-consumer discipline forbids, and here the producer is an interrupt handler that cannot take a lock.

The buffer drops the **newest** element when full and increments a counter.
The parent spec's stated reason for not blocking the producer still holds; only the choice of which element to discard changes.

**Counters in the packet stream.**
The parent spec states that overflow, CRC, and sequence-gap counters "are part of the packet stream rather than debug-only output".
Milestone 2 partially defers this.
See "Wire format" below for the reasoning and what is deferred.

## Decisions taken

### Simulated time model: event-driven idle

`SimClock` currently advances only inside `delay_ms()`, which only `FakeImu::read()` and `FakeRadio::transmit()` call.
That works for a sampler that blocks on the IMU every pass.
A sampler that deliberately never blocks would spin with `now_ms()` frozen at zero and report an unbounded packet rate.

`IClock` gains `wait_for_event(uint32_t max_ms)`.
`SimClock` implements it by advancing simulated time until a registered producer reports pending data or the cap expires.
Time therefore advances only through hardware timing that is derived from a datasheet or a published formula: 9600 baud byte arrival, the BNO055's 100 Hz fusion output rate, and LoRa airtime from the formula already recorded in the README.
No constant in this model is invented.

This change is additive.
`PollingSampler` blocks on the IMU every pass and so never calls `wait_for_event`, and `FakeGps::on_tick` semantics are untouched.
The milestone 1 figures published in the README therefore stay bit-identical rather than merely probably unchanged, and the benchmark run at the end of this milestone must confirm that.

**Rejected: charging per-work-unit CPU cost to both samplers.**
Modelling a finite MCU by charging a cost per byte parsed and per packet encoded would be more faithful in principle.
In practice every one of those constants would be a guess, and on a 600 MHz Teensy 4.1 they sit roughly four orders of magnitude below the 3023 ms of radio airtime that dominates the result.
It would mean inventing numbers in order to change nothing.
This is recorded rather than silently skipped because "why didn't you model CPU time" is a fair question to ask of this repository.

**Disclosed limitation.**
The model treats the MCU as infinitely fast.
It can therefore demonstrate that blocking was the loss mechanism.
It cannot demonstrate that the Teensy has the cycles to keep up.
That narrower claim is the one the README will make.

### Wire format: unchanged at 45 bytes

The interrupt build produces two counters the polling build has no equivalent for: outbound queue drops and IMU ring overflows.
Carrying one more `uint16` in-band would grow the packet from 45 to 47 bytes.

At SF12/BW125/CR4-8 with low-data-rate optimisation, the payload symbol count rises from 80 to 88, and airtime rises from approximately 3022.8 ms to approximately 3285 ms, a 8.7% increase.
That would invalidate the milestone 1 baseline figures published in the README, which were measured at 45 bytes.

The wire format stays at 45 bytes for this milestone.
The counter the headline claim rests on, GPS receive overflow, is already carried in-band as `diag.drops`.
The two new counters are reported through the benchmark output and the gateway serial line for now, and fold into a version 0x02 format in a later milestone that has its own reason to change the layout.

### Radio interface: extend rather than migrate

`IRadio::transmit()` keeps its existing blocking contract.
A new `IAsyncRadio : public IRadio` adds `begin_transmit()` and `tx_busy()`.

`PollingSampler` and the gateway continue to compile against the unchanged base interface.
This guarantees that the milestone 1 build is untouched by milestone 2's work, rather than requiring a re-measurement to establish that it was.
The cost is a wider interface with two transmit paths, which is accepted deliberately: a trustworthy before-and-after comparison is this repository's purpose.

## Architecture

### floodnet_core

`ring_buffer.hpp`, new.
Single-producer single-consumer, lock-free, templated on element type and a power-of-two capacity.
Head and tail are `std::atomic<uint32_t>` with acquire/release ordering, which is correct on both the host and a Cortex-M7 and does not require a hardware header, so the CI seam check is unaffected.

The buffer exposes a drop counter and a high-water mark.
The high-water mark is free to maintain and is what an embedded reviewer actually wants to see, so it is reported alongside the rates rather than kept internal.

### floodnet_hal

`IClock` gains `wait_for_event(uint32_t max_ms)`.
This is not a simulation artifact.
On Teensy it is `__WFI()`, sleep until an interrupt arrives or the cap expires, and it is also the primitive milestone 3's duty cycling will build on.

`IImuSource` gains `bool data_ready()`.
The interrupt service routine on the BNO055 INT pin sets a flag and nothing more.
The I2C read stays in main context, because a blocking I2C transaction inside an interrupt handler is the wrong answer regardless of what it would do to the benchmark.
The improvement over milestone 1 is that the loop stops issuing a speculative I2C read on every pass and reads only when the sensor says a sample exists.

`IAsyncRadio` is new, as described under Decisions taken.

### Application

`src/sampler_interrupt.{hpp,cpp}`, new.
`step()` drains the GPS ring buffer, parses completed sentences, reads the IMU when `data_ready()` reports a sample, starts a transmit when the radio is idle, and calls `wait_for_event()` when there is no work.
It never blocks.

`src/main.cpp` selects a sampler on the `FLOODNET_SAMPLER_POLLING` and `FLOODNET_SAMPLER_INTERRUPT` build flags.

### Buffer capacities

Both are derived rather than chosen.

GPS ring: 4096 bytes.
9600 baud 8N1 is 960 bytes per second.
Covering the worst stall this firmware can program, 3023 ms of SF12 airtime, requires approximately 2902 bytes.
The next power of two is 4096, which is 4 KB against the Teensy 4.1's 1 MB of RAM.

IMU ring: 16 samples.
`SamplePairer` consumes only the newest sample, so additional depth buys nothing.
IMU ring overflow is therefore expected and benign in normal operation, which is precisely why it must be counted separately from GPS overflow rather than folded into a single drop figure.

Outbound packet queue: 8 packets, and it drops the **oldest**.

This is the opposite policy from the two ring buffers above, and the difference is not arbitrary.
The GPS and IMU rings are fed by interrupt handlers, so they are SPSC and the producer cannot touch the consumer's index, which forces drop-newest.
The outbound queue is produced and consumed entirely in main context, so it is under no such constraint and can drop whichever end is less useful.

For position fixes the older packet is the less useful one: a receiver would rather have the node's current position than its position three seconds ago.
So the queue keeps the freshest and discards the stalest, and counts every discard in `tx_queue_drops`.

Depth 8 absorbs a short burst without hoarding stale fixes.
No finite depth can do better at SF12, where fixes arrive roughly forty times faster than the radio can send them, and a deeper queue would only mean transmitting older data.

## Drop accounting

Three distinct counters, deliberately not merged:

| Counter | Meaning | Expected in the interrupt build |
|---|---|---|
| `gps_rx_overflows` | Bytes lost before the parser saw them | Zero. This going to zero is the milestone's core result. |
| `imu_ring_overflows` | Samples discarded because only the newest is used | Non-zero and benign by design |
| `tx_queue_drops` | Complete packets the node chose not to send | Non-zero at SF12. This is the honest cost. |

Collapsing these into one number would make the interrupt build look lossy at SF12 for a reason that has nothing to do with acquisition.
Keeping them separate is what makes the result readable.

## What this milestone claims

At SF12, which is what `src/hal/teensy_radio.hpp` actually programs, the polling baseline already runs at roughly 97% of what the radio physically permits: 0.32 packets per second against a ceiling of 1/3.023 = 0.33.
Interrupt-driven acquisition cannot move that, and the milestone will not claim it does.

Fixes arrive at roughly 14 per second and the radio can send one every three seconds.
The surplus has to go somewhere.
In milestone 1 it goes into a 64-byte UART buffer that silently corrupts an unknown number of sentences.
In milestone 2 it goes into an outbound queue that drops complete packets under an explicit policy and counts every one.

**The claim is therefore that loss moves from silent and unquantified to explicit, counted, and chosen, not that less data is lost.**

At SF7 there is real headroom and a genuine throughput result is expected.
The polling baseline measured 5.99 packets per second against a ceiling of approximately 9.80 (92 ms airtime serialised with a 10 ms IMU read).
Removing the IMU read from the transmit path raises that ceiling to approximately 10.87 (92 ms airtime alone).

These are predictions, not results.
The benchmark publishes whatever it measures, including if it refutes them.

**CONTROL is a null control.**
At 5 ms airtime the polling baseline is already limited by the GPS sentence rate of approximately 14.12 per second, not by the radio, and it measured 14.32.
The interrupt build should show essentially no gain there.
A large CONTROL gain would mean the simulation is flattering the new sampler, and is to be treated as a bug in the measurement rather than a result.

## Build targets

`platformio.ini` gains `node_interrupt`, extending `teensy_base` with `-D FLOODNET_SAMPLER_INTERRUPT`.
The `native` environment's `build_src_filter` extends to include `sampler_interrupt.cpp`.
CI builds `node_polling`, `node_interrupt`, and `gateway`, and runs the native suite.

## Measurement

`test/test_benchmark/` runs both samplers across all three existing radio profiles and prints one line per combination in the form `BENCH,strategy,profile,...`, extending the current stable greppable format with a strategy column.
Buffer high-water marks are reported alongside packets per second and overflows per second.

The README gains a comparison table replacing the single-strategy results table, and keeps the existing reproduction command.

## Testing

New suites:

- Ring buffer: fill, wrap, drop-newest overflow policy, drop counter, high-water mark, and a simulated interleaved producer and consumer.
- `SimClock::wait_for_event`: advances to the next pending producer, respects the cap, and does not advance past an already-pending producer.
- Interrupt sampler: mirrors `test/test_polling/`, and asserts `gps_rx_overflows` is zero at SF12, which is the assertion milestone 1 could not make.

All 46 existing tests stay green, and the benchmark must reproduce the published milestone 1 figures unchanged.

## Out of scope

Deferred to later milestones as already recorded in the README, and not reopened here: mesh relay wiring, node state machine and sleep, sequence-gap counting, hardware watchdog, and host trace replay.

Hardware validation is out of scope for this milestone.
No board is available, so the IMU data-ready interrupt is written and compiled but exercised only against a fake.
The README must say so plainly rather than let the reader assume otherwise.

## Open questions

- Whether to name the research lab in public-facing documentation, carried over unresolved from the parent spec.
