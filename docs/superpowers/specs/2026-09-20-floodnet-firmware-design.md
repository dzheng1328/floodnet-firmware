# FloodNet Firmware Design

Date: 2026-09-20
Status: approved, pending implementation

## Purpose

FloodNet is a distributed sensor network for flood mapping.
Battery-powered nodes sample GPS position and IMU orientation, packetize the readings, and relay them over LoRa to a gateway.
The gateway writes framed records to a serial link, where a host-side pipeline ingests, reconstructs, and visualizes the resulting dataset.

This repository holds the node and gateway firmware, the hardware-in-the-loop test tooling, and the host pipeline.

The codebase is a clean rewrite of firmware built for a Duke research lab between August 2024 and May 2026.
The original lives on lab infrastructure and is not available here, so every line in this repository is written from scratch against the same requirements.

## Goals

1. A working polling-based firmware that acquires, packetizes, and relays sensor data.
2. An interrupt-driven acquisition path that replaces it, with both retained as selectable build targets.
3. Measured evidence that the second outperforms the first, produced by a benchmark anyone can rerun.
4. Automated regression and signal-integrity testing across a hardware-in-the-loop setup.
5. A host pipeline that turns serial logs into reconstructed tracks and plots.

## Non-goals

- Field deployment tooling, enclosure design, or mechanical mounting.
- A general-purpose mesh networking stack. Routing here is store-and-forward with deduplication, nothing more.
- Support for boards other than Teensy, beyond the host build used for testing.

## Hardware

The original node hardware is not documented here, so the firmware targets a representative build.
The exact parts are listed so that pin assignments, bus choices, and timing budgets are concrete rather than hypothetical.

| Component | Part | Interface |
|---|---|---|
| MCU | Teensy 4.1 | - |
| GPS | u-blox NEO-M8N | UART, NMEA 0183, 9600 baud default |
| IMU | Bosch BNO055 | I2C at 400 kHz, data-ready interrupt on GPIO |
| Radio | HopeRF RFM95W (SX1276) | SPI, 915 MHz ISM band, DIO0 interrupt |

## Architecture

The system splits into three layers.
The split exists so that the majority of the logic can be compiled and tested on a development machine with no hardware attached.

### floodnet_core

Platform-independent logic with no Arduino or Teensy dependency.
This constraint is absolute: no file under `lib/floodnet_core` may include a hardware header.
Everything here is unit-tested on the host.

- `ring_buffer.hpp`: single-producer single-consumer lock-free ring buffer, safe for a producer running in interrupt context.
- `packet.hpp`: wire format, CRC16 computation and validation, sequence numbering.
- `nmea.hpp`: NMEA sentence parser expressed as a pure function over a byte span.
- `sample.hpp`: `GpsFix`, `ImuSample`, and timestamped record types.
- `mesh.hpp`: store-and-forward relay logic, duplicate suppression keyed on node ID and sequence number, time-to-live decrement.
- `node_state.hpp`: duty-cycle and sleep state machine.

### floodnet_hal

Interfaces describing what the core needs from hardware, with two implementations behind them.

The Teensy implementation drives real UART, I2C, and SPI peripherals.
The host implementation replays recorded sensor traces from disk, which is what allows continuous integration to exercise the full acquisition path without a board.

Interfaces cover GPS, IMU, radio, clock, and power.

### Application

`src/main.cpp` wires the HAL to the core and selects an acquisition strategy at compile time.

Two strategies exist:

- `sampler_polling.cpp` blocks on each sensor in turn inside a superloop.
- `sampler_interrupt.cpp` fills ring buffers from UART receive, IMU data-ready, and radio DIO0 interrupt handlers, leaving the main loop to drain them.

Both feed the same core and emit identical packets.
The only difference is how bytes reach the buffers.

## Data flow

GPS bytes arrive over UART and IMU samples over I2C, each landing in its own ring buffer.
The core drains both buffers, parses NMEA sentences into fixes, and pairs each fix with the nearest IMU sample by timestamp.
The paired record becomes a packet carrying a node identifier, a sequence number, and a CRC16.

The radio transmits the packet.
Relay nodes receive, check the deduplication table, decrement the time-to-live, and retransmit anything they have not already seen.
The gateway writes framed records to its serial port.

## Error handling

Every node maintains counters for ring buffer overflow, CRC failure, and sequence gap.
These counters are the mechanism by which acquisition strategies are compared, so they are part of the packet stream rather than debug-only output.

Additional handling:

- NMEA sentences failing checksum validation are discarded and counted.
- Radio transmit and receive operations carry timeouts.
- A hardware watchdog resets a node that stops making progress.
- Ring buffer overflow drops the oldest data and increments the counter rather than blocking the producer, because the producer may be an interrupt handler.

## Measurement

Performance claims about this firmware are reproducible rather than remembered.

**Throughput and data loss.**
`tools/hil/benchmark.py` replays a single recorded sensor trace through both firmware builds and reports samples per second and drop counts for each.
Whatever difference the two builds show is the difference that gets documented.

**Bit error rate and noise floor.**
The hardware-in-the-loop tool transmits known pseudo-random payloads between two radios, compares the received bits against the transmitted ones, and samples RSSI and SNR to characterize the noise floor.

**Range.**
Range is established by a documented link-budget calculation rather than measurement, because a multi-mile field test is out of scope for this repository.
The calculation and its assumptions are written down so the estimate can be checked.

## Testing

Host unit tests cover:

- Ring buffer behavior under a simulated concurrent producer, including the overflow path.
- NMEA parsing against a table of valid, malformed, and truncated sentences.
- Packet encode and decode round trips, including deliberate corruption.
- Mesh deduplication and time-to-live handling.
- Node state machine transitions.

Continuous integration compiles all four PlatformIO environments and runs the native test suite on every push.

## Repository layout

```
floodnet-firmware/
├── platformio.ini          # envs: node_polling, node_interrupt, gateway, native
├── lib/
│   ├── floodnet_core/
│   └── floodnet_hal/
├── src/
│   ├── main.cpp
│   ├── sampler_polling.cpp
│   └── sampler_interrupt.cpp
├── test/
├── tools/
│   ├── hil/
│   └── pipeline/
├── docs/
└── .github/workflows/ci.yml
```

## Implementation sequence

Each milestone is a self-contained increment leaving the repository in a working state.

1. **Polling baseline.**
   Project skeleton, HAL interfaces with both implementations, sensor drivers, NMEA parser, packet codec, mesh relay, blocking superloop over a single shared buffer, host unit tests, continuous integration.
   Complete and functional, with the data loss characteristic latent rather than disguised.

2. **Interrupt-driven acquisition.**
   Per-stream SPSC ring buffers, a non-blocking drain that replaces the blocking waits on the already-ISR-served GPS and radio paths, a new interrupt path for the IMU data-ready line, the second build environment, drop counters surfaced in the packet stream.

3. **Power management.**
   Sleep modes and duty cycling driven by the node state machine.

4. **Hardware-in-the-loop tooling.**
   Regression harness and signal-integrity measurement covering noise and bit error rate.

5. **Host pipeline.**
   Serial log ingestion, track reconstruction, visualization.

6. **Benchmark and results.**
   The comparison harness, plus a results document recording what the two acquisition strategies actually measured.

## Open questions

- Whether to name the research lab in public-facing documentation. The flood-mapping work may be unpublished, so this is settled with the lab before the repository becomes public.
