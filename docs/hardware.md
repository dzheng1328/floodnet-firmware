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
The trade is deliberate.
Nodes report at most a few times per minute, so throughput is not the constraint.
Link margin is.

## Build configuration notes

Two choices in `platformio.ini` are not obvious from reading the file alone, so a pointer here saves someone re-deriving them.
The reasoning is spelled out in full as a comment above `[teensy_base]` in `platformio.ini`; this is a summary, not a substitute.

RadioHead is deliberately not pinned in `lib_deps`.
`framework-arduinoteensy` bundles its own copy (currently 1.112), and adding a registry pin makes PlatformIO's dependency finder compile headers from one copy against sources from the other.
If a framework upgrade changes the bundled version, check `RH_RF95`'s API against `src/hal/teensy_radio.hpp` rather than adding a pin.

Adafruit BNO055 is scoped to the `node_polling` environment rather than shared across environments.
The gateway has no IMU, and pulling the library into that build makes Adafruit BusIO fail to resolve `SPI.h`.
