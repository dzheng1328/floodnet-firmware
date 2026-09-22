#ifndef FLOODNET_TEENSY_IMU_HPP
#define FLOODNET_TEENSY_IMU_HPP

#include <Adafruit_BNO055.h>
#include <Arduino.h>
#include <Wire.h>

#include <floodnet/hal/imu.hpp>

namespace floodnet {

/// BNO055 over I2C, with sample timing driven by an interrupt.
///
/// The interrupt is a Teensy IntervalTimer, not a pin on the sensor, and that
/// is a hardware constraint rather than a shortcut. The BNO055's INT output
/// supports motion-triggered sources only: any-motion, slow/no-motion and
/// high-g on the accelerometer, any-motion and high-rate on the gyroscope. It
/// has no fusion-output data-ready interrupt. (The parent design spec's
/// hardware table claims otherwise and is wrong; see the milestone 2 design
/// doc.) Adafruit_BNO055::write8 is private besides, so the INT pin could not
/// be configured through that library even if a suitable source existed.
///
/// A timer at the sensor's fixed 100 Hz NDOF fusion rate is the honest
/// equivalent: the handler sets a flag and returns, and the blocking I2C read
/// stays in main context where it belongs.
class TeensyImu : public IImuSource {
  public:
    /// 100 Hz, the BNO055's NDOF fusion output rate.
    static const uint32_t SAMPLE_PERIOD_US = 10000;

    TeensyImu() : sensor_(55, BNO055_ADDRESS_A, &Wire), ready_(false) {}

    bool begin() {
        Wire.begin();
        Wire.setClock(400000);
        ready_ = sensor_.begin();
        if (!ready_) {
            return false;
        }
        // Prime the first read so bring-up does not wait a tick for it.
        s_sample_due = true;
        if (!timer_.begin(on_sample_due, SAMPLE_PERIOD_US)) {
            // No hardware timer available. Report the failure rather than
            // running with a flag nothing will ever set again: without this,
            // the first read() clears s_sample_due and data_ready() reports
            // false for the life of the node, with no error anywhere.
            ready_ = false;
            return false;
        }
        return true;
    }

    bool data_ready() const override { return ready_ && s_sample_due; }

    bool read(ImuSample *out) override {
        if (!ready_ || out == nullptr) {
            return false;
        }

        // Cleared before the transaction. If the timer fires during the I2C
        // read, that sample is real and the flag should survive.
        s_sample_due = false;

        sensors_event_t event;
        sensor_.getEvent(&event, Adafruit_BNO055::VECTOR_EULER);

        out->time_ms = millis();
        out->yaw_cd = static_cast<int16_t>(event.orientation.x * 100.0f);
        out->pitch_cd = static_cast<int16_t>(event.orientation.y * 100.0f);
        out->roll_cd = static_cast<int16_t>(event.orientation.z * 100.0f);
        out->valid = true;
        return true;
    }

  private:
    /// IntervalTimer takes a plain function pointer, so the flag is static.
    /// That limits this driver to one IMU per firmware image, which is what
    /// the hardware has.
    static void on_sample_due() { s_sample_due = true; }
    static inline volatile bool s_sample_due = false;

    Adafruit_BNO055 sensor_;
    IntervalTimer timer_;
    bool ready_;
};

}  // namespace floodnet

#endif  // FLOODNET_TEENSY_IMU_HPP
