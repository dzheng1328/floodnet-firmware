#ifndef FLOODNET_TEENSY_IMU_HPP
#define FLOODNET_TEENSY_IMU_HPP

#include <Adafruit_BNO055.h>
#include <Arduino.h>
#include <Wire.h>

#include <floodnet/hal/imu.hpp>

namespace floodnet {

class TeensyImu : public IImuSource {
  public:
    TeensyImu() : sensor_(55, BNO055_ADDRESS_A, &Wire), ready_(false) {}

    bool begin() {
        Wire.begin();
        Wire.setClock(400000);
        ready_ = sensor_.begin();
        return ready_;
    }

    bool read(ImuSample *out) override {
        if (!ready_ || out == nullptr) {
            return false;
        }

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
    Adafruit_BNO055 sensor_;
    bool ready_;
};

}  // namespace floodnet

#endif  // FLOODNET_TEENSY_IMU_HPP
