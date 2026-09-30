#ifndef FLOODNET_TEENSY_POWER_HPP
#define FLOODNET_TEENSY_POWER_HPP

#include <Arduino.h>
#include <Snooze.h>

#include <floodnet/hal/power.hpp>

#include "teensy_gps.hpp"
#include "teensy_imu.hpp"
#include "teensy_radio.hpp"

namespace floodnet {

class TeensyPower : public IPower {
  public:
    /// A0, behind a divider of two 100 kOhm resistors from the cell. See
    /// docs/hardware.md.
    static const uint8_t BATTERY_PIN = 14;

    TeensyPower(TeensyGps &gps, TeensyImu &imu, TeensyRadio &radio, float frequency_mhz,
                int8_t tx_power_dbm)
        : gps_(gps),
          imu_(imu),
          radio_(radio),
          frequency_mhz_(frequency_mhz),
          tx_power_dbm_(tx_power_dbm),
          timer_(),
          block_(timer_) {}

    void set_power(Peripheral p, PowerState s) override {
        const bool on = s == PowerState::On;
        switch (p) {
        case Peripheral::Gps:
            if (on) {
                gps_.wake();
            } else {
                gps_.enter_backup();
            }
            break;
        case Peripheral::Imu:
            if (on) {
                imu_.resume();
            } else {
                imu_.suspend();
            }
            break;
        case Peripheral::Radio:
            if (on) {
                radio_.wake();
            } else {
                radio_.sleep();
            }
            break;
        }
    }

    bool power_cycle(Peripheral p) override {
        switch (p) {
        case Peripheral::Gps:
            gps_.enter_backup();
            gps_.wake();
            return true;
        case Peripheral::Imu:
            imu_.suspend();
            return imu_.begin();  // begin() resets the BNO055
        case Peripheral::Radio:
            return radio_.begin(frequency_mhz_, tx_power_dbm_);  // pulses the reset pin
        }
        return false;
    }

    void sleep_until(uint32_t wake_ms) override {
        const int32_t remaining = static_cast<int32_t>(wake_ms - millis());
        if (remaining <= 0) {
            return;
        }

        // Snooze's Teensy 4 timer takes whole seconds.
        const uint32_t whole_s = static_cast<uint32_t>(remaining) / 1000;
        if (whole_s > 0) {
            timer_.setTimer(whole_s);
            // Push out anything still queued on USB serial (the TX line of
            // the transmit that just ended) before the clocks stop. Whether
            // plain Serial survives deepSleep on a Teensy 4.1 is unverified;
            // Snooze's own example uses its SnoozeUSBSerial driver instead.
            Serial.flush();
            Snooze.deepSleep(block_);
            // Snooze 6.3.9 (src/hal/TEENSY_40/SnoozeTimer.cpp) stores
            // period = seconds * 32768 and on wake adds period / 1000 to
            // systick_millis_count: 32.768 ms per slept second instead of
            // 1000. Add the rest, or the schedule stretches about thirty-fold.
            // Read from the library source; not observed on a board.
            // The += is a read-modify-write of a counter the SysTick ISR
            // also increments, so a tick landing between the load and the
            // store would be lost: mask interrupts around it.
            const uint32_t correction = whole_s * 1000 - (whole_s * 32768) / 1000;
            __disable_irq();
            systick_millis_count += correction;
            __enable_irq();
        }

        // The sub-second remainder, and any early wake, on the ordinary tick.
        while (static_cast<int32_t>(wake_ms - millis()) > 0) {
            __asm__ volatile("wfi" ::: "memory");
        }
    }

    uint16_t battery_mv() override {
        // 10-bit ADC against 3.3 V, doubled for the divider.
        const uint32_t raw = static_cast<uint32_t>(analogRead(BATTERY_PIN));
        return static_cast<uint16_t>(raw * 3300UL * 2UL / 1023UL);
    }

  private:
    TeensyGps &gps_;
    TeensyImu &imu_;
    TeensyRadio &radio_;
    float frequency_mhz_;
    int8_t tx_power_dbm_;
    SnoozeTimer timer_;
    SnoozeBlock block_;
};

}  // namespace floodnet

#endif  // FLOODNET_TEENSY_POWER_HPP
