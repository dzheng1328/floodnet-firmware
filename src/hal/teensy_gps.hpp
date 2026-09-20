#ifndef FLOODNET_TEENSY_GPS_HPP
#define FLOODNET_TEENSY_GPS_HPP

#include <Arduino.h>

#include <floodnet/hal/gps.hpp>

namespace floodnet {

/// Reads NMEA from Serial1. Counts bytes the driver's buffer overran, which is
/// the polling loop's loss showing up as a number rather than as silence.
class TeensyGps : public IGpsSource {
  public:
    explicit TeensyGps(HardwareSerial &port) : port_(port), dropped_(0) {}

    void begin(uint32_t baud) { port_.begin(baud); }

    int read_byte() override {
        if (port_.available() <= 0) {
            return -1;
        }
        return port_.read();
    }

    uint16_t bytes_dropped() const override { return dropped_; }

    /// Call once per loop pass: a full receive buffer means bytes were lost.
    void note_buffer_state() {
        if (port_.available() >= RX_BUFFER_BYTES && dropped_ < 0xFFFF) {
            ++dropped_;
        }
    }

  private:
    /// Teensy 4.x HardwareSerial keeps a 64-byte software receive buffer.
    static const int RX_BUFFER_BYTES = 64;

    HardwareSerial &port_;
    uint16_t dropped_;
};

}  // namespace floodnet

#endif  // FLOODNET_TEENSY_GPS_HPP
