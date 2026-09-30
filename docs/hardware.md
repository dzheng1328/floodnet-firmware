# Hardware

## Bill of materials

| Component | Part | Interface |
|---|---|---|
| MCU | Teensy 4.1 | - |
| GPS | u-blox NEO-M8N | UART, NMEA 0183, 9600 baud |
| IMU | Bosch BNO055 | I2C at 400 kHz. No data-ready interrupt; see below. |
| Radio | HopeRF RFM95W (SX1276) | SPI, 915 MHz |
| Battery sense | 2 x 100 kΩ divider | Cell to Teensy pin 14 (A0), for `battery_mv` |

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
| Battery sense (divided) | 14 (A0) |

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

## IMU orientation fields

The BNO055's Euler registers are heading (`0x1A`), roll (`0x1C`) and pitch (`0x1E`), each an int16 in 1/16 degree.
`Adafruit_BNO055::getEvent` reads them in one burst and copies them into `orientation.x`, `.y` and `.z` in that order, so `.y` is roll and `.z` is pitch; both facts are read from the vendored library 1.6.4 (`Adafruit_BNO055.h` and `getVector()`), not observed on a board.
The library's `UNIT_SEL` write is commented out, so the sensor keeps its reset units, degrees and Android orientation, in which the BNO055 data sheet (BST-BNO055-DS000) gives heading 0 to 360 degrees, roll -90 to +90 and pitch -180 to +180.

Each angle travels as an int16 in centidegrees, and 360 degrees is 36000 cd, which does not fit.
`bno055_euler_to_sample()` in `lib/floodnet_core/src/orientation.cpp` therefore rounds each angle to the nearest centidegree and wraps it into [-18000, 18000): a heading of 180 degrees or more arrives negative, so 350 degrees is sent as -1000 cd.
The wrap does not depend on the data sheet ranges above being exact; any finite angle lands in range.

Before this conversion existed, the driver cast `heading * 100.0f` straight to `int16_t`.
That is undefined behaviour for every heading above 327.67 degrees, about 9% of the circle, and it also wrote roll into `pitch_cd` and pitch into `roll_cd`.
Every record sent before the fix has those two fields swapped.

## Build configuration notes

Two choices in `platformio.ini` are not obvious from reading the file alone, so a pointer here saves someone re-deriving them.
The reasoning is spelled out in full as a comment above `[teensy_base]` in `platformio.ini`; this is a summary, not a substitute.

RadioHead is deliberately not pinned in `lib_deps`.
`framework-arduinoteensy` bundles its own copy (currently 1.112), and adding a registry pin makes PlatformIO's dependency finder compile headers from one copy against sources from the other.
If a framework upgrade changes the bundled version, check `RH_RF95`'s API against `src/hal/teensy_radio.hpp` rather than adding a pin.

Adafruit BNO055 is scoped to the node builds rather than shared across all of them.
The gateway has no IMU, and pulling the library into that build makes Adafruit BusIO fail to resolve `SPI.h`.

## Power modes

Milestone 3 puts each part into its own low-power mode rather than switching its supply, so the only hardware addition is the battery divider above.

| Part | Low-power mode | Driver call |
|---|---|---|
| NEO-M8N | software backup, woken by UART RX activity | `TeensyGps::enter_backup()` / `wake()`, using `UBX-RXM-PMREQ` |
| BNO055 | suspend | `TeensyImu::suspend()` / `resume()` |
| RFM95W | sleep | `TeensyRadio::sleep()` / `wake()` |
| Teensy 4.1 | Snooze `deepSleep()` with a GPT timer wake | `TeensyPower::sleep_until()` |

The bundled Snooze 6.3.9 miscounts time on Teensy 4: after a timer wake it advances `millis()` by 32.768 ms per slept second instead of 1000.
`TeensyPower::sleep_until()` adds the difference itself; the comment there cites the library line.
This is read from the library source and has not been observed on a board.

The watchdog is WDOG1 at 90 s, programmed directly through `imxrt.h`.
Both of its low-power suspend bits in `WDOG_WCR` are left clear: `WDW` (bit 7), which suspends it in WAIT mode, and `WDZST` (bit 0), which suspends it in STOP and DOZE.
The bundled Snooze 6.3.9 enters WAIT mode: `hal_deepSleep()` in `src/hal/TEENSY_40/hal.c` sets `CCM_CLPCR_LPM(0x01)` at line 789, so `WDW` is the bit that matters for this build.
The `WDW`-to-WAIT mapping is a reading of the RT1060 reference manual's `WDOG_WCR` description and has not been verified on a board.

**First bench test: does WDOG1 count while its `CCM_CCGR3` gate is off during `deepSleep`?**
The same function rewrites `CCM_CCGR3` at `hal.c:805`, keeping only the ACMP1-4 gates and `0x10000000`, which clears `CCM_CCGR3_WDOG1` (bits 17-16 in `imxrt.h`), and restores the register on wake at `hal.c:851`.
If WDOG1 stops counting while its clock is gated, a timer wake that never fires would hang the node with nothing to reset it, whatever `WDW` and `WDZST` say.
This is read from source and is the first thing to measure on a board.

The boot counter lives in `SNVS_LPGPR3`, which survives resets while SNVS is powered.
It is not in `LPGPR0`, because Snooze's `SnoozeAlarm` writes `SNVS_LPGPR` (offset 0x68, the legacy alias of `LPGPR0`) and would overwrite it.

None of this has run on hardware.
