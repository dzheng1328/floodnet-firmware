# Hardware

## Bill of materials

| Component | Part | Interface |
|---|---|---|
| MCU | Teensy 4.1 | - |
| GPS | u-blox NEO-M8N | UART, NMEA 0183, 9600 baud |
| IMU | Bosch BNO055 | I2C at 400 kHz. No data-ready interrupt; see below. |
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

## IMU sample timing

The BNO055 has no data-ready interrupt.
Its INT pin offers motion-triggered sources only: any-motion, slow/no-motion and high-g on the accelerometer, any-motion and high-rate on the gyroscope.
None of them signal "a fresh fusion sample is available", which is what periodic sampling needs.
`Adafruit_BNO055::write8` is private besides, so the INT pin could not be configured through that library even if a suitable source existed.

`src/hal/teensy_imu.hpp` therefore drives sampling from a Teensy `IntervalTimer` at 100 Hz, the sensor's fixed NDOF fusion output rate.
The handler sets a flag and returns; the blocking I2C read stays in main context, where a transaction of that length belongs.

The INT pin is left unconnected, which is why it does not appear in the pin assignment table.

## Build configuration notes

Two choices in `platformio.ini` are not obvious from reading the file alone, so a pointer here saves someone re-deriving them.
The reasoning is spelled out in full as a comment above `[teensy_base]` in `platformio.ini`; this is a summary, not a substitute.

RadioHead is deliberately not pinned in `lib_deps`.
`framework-arduinoteensy` bundles its own copy (currently 1.112), and adding a registry pin makes PlatformIO's dependency finder compile headers from one copy against sources from the other.
If a framework upgrade changes the bundled version, check `RH_RF95`'s API against `src/hal/teensy_radio.hpp` rather than adding a pin.

Adafruit BNO055 is scoped to the node builds rather than shared across all of them.
The gateway has no IMU, and pulling the library into that build makes Adafruit BusIO fail to resolve `SPI.h`.
