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

## Building

```bash
pio run -e node_polling    # sensor node
pio run -e gateway         # gateway
pio test -e native         # unit tests, no hardware needed
```

## Wire format

45 bytes, little-endian, CRC16-CCITT-FALSE over the first 43.
Carries a node identifier, a sequence number, a TTL, a GPS fix, an IMU sample, and the node's drop and CRC error counters.
Those counters travel in-band deliberately: a receiver can see loss at the source rather than inferring it from gaps.

Full layout is in `docs/superpowers/specs/2026-09-20-floodnet-firmware-design.md`.

## Gateway serial output

The gateway writes one line per event to its USB serial port at 115200 baud, comma-separated, so a host pipeline can parse it without a framing layer.

A successfully decoded, non-duplicate packet produces a `REC` line with these fields in order:

```
REC,node_id,seq,gps_time_ms,lat_1e7,lon_1e7,alt_mm,satellites,yaw_cd,pitch_cd,roll_cd,drops,crc_errors,rssi
```

A packet that fails CRC validation produces:

```
ERR,crc,crc_error_count
```

A radio receive that returns the wrong number of bytes produces:

```
ERR,short,bytes_received,short_read_count
```

`crc_error_count` and `short_read_count` are running totals kept by the gateway since power-on, capped at 65535.
The authoritative source for this format is `print_record()` and `loop()` in `src/gateway_main.cpp`.

## Known limitations

The polling loop blocks on the IMU and on radio transmission.
GPS bytes arriving during those stalls land in a 64-byte hardware buffer, and when that buffer fills, they are gone.
`test/test_polling/` asserts that this loss happens rather than papering over it, and the loss is reported in every outgoing packet's `drops` field, which carries the node's `IGpsSource::rx_overflows()` value.

That counter's unit is implementation-defined, and the two implementations in this repository do not measure the same thing.
The simulated GPS source in `test/support/fake_gps.hpp` counts one overflow per individual byte it discards, because it knows exactly what it threw away.
The Teensy driver in `src/hal/teensy_gps.hpp` counts one overflow per loop pass that finds the UART buffer already saturated, because real hardware exposes no lost-byte count.
Do not compare a drop count from the simulation against a drop count from hardware; they are different quantities that happen to share a name.
See the contract comment on `IGpsSource::rx_overflows()` in `lib/floodnet_hal/include/floodnet/hal/gps.hpp`.

Two timing profiles run against the simulated hardware in `test/test_polling/` for 400 steps of the same sampler code, and the results are simulated measurements, not field data:

| Profile | IMU read | Radio transmit | Packets sent | Overflows |
|---|---|---|---|---|
| FAST | 2 ms | 5 ms | 12 | 0 |
| REALISTIC | 10 ms | 60 ms | 46 | 148 |

The point is not the absolute numbers; it is that identical parsing and radio code loses nothing when the loop keeps up and loses steadily when it stalls.
The loss is a property of the blocking strategy, not of the NMEA parser or the radio driver.
The arithmetic is consistent with that: the IMU read and radio transmit together stall the loop for 70 ms in the REALISTIC profile, and at 9600 baud (roughly 0.96 bytes/ms) that stall lets in about 67 bytes, the length of one NMEA sentence, arriving into a 64-byte buffer.
That is roughly 3 bytes lost per productive iteration, and 148 overflows over 46 packets is 3.2 per packet, which matches.

Removing it is the next milestone.

## Hardware

See [docs/hardware.md](docs/hardware.md).

## About this repository

A clean rewrite of firmware I built for a Duke research lab between 2024 and 2026.
The original is on lab infrastructure and is not public, so this is written from scratch against the same requirements.
