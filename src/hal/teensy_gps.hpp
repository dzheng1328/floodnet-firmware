#ifndef FLOODNET_TEENSY_GPS_HPP
#define FLOODNET_TEENSY_GPS_HPP

#include <Arduino.h>

#include <floodnet/hal/gps.hpp>

namespace floodnet {

/// Reads NMEA from Serial1. Counts bytes the driver's buffer overran, which is
/// the polling loop's loss showing up as a number rather than as silence.
class TeensyGps : public IGpsSource {
  public:
    /// HardwareSerialIMXRT rather than HardwareSerial: addMemoryForRead() is
    /// declared on the concrete Teensy 4 subclass, not on the abstract base
    /// (cores/teensy4/HardwareSerial.h, where the base closes at line 170 and
    /// the subclass runs 172-356). Narrowing is honest here, since this driver
    /// already targets one platform, and it beats a static_cast that would be
    /// undefined behaviour for any other subclass. The seam is unaffected:
    /// IGpsSource in lib/ stays platform-independent, and Serial1 is declared
    /// `extern HardwareSerialIMXRT Serial1`, so the sole call site in
    /// src/main.cpp needs no change.
    explicit TeensyGps(HardwareSerialIMXRT &port) : port_(port), dropped_(0) {}

    void begin(uint32_t baud) {
        // HardwareSerial owns the UART receive interrupt and fills its own
        // buffer, which defaults to 64 bytes: roughly 66 ms of traffic at 9600
        // baud, against the 3023 ms SF12 transmit this firmware can program.
        //
        // This firmware cannot insert a buffer ahead of that interrupt, so it
        // hands the existing handler a deeper array instead. This is the
        // mechanism milestone 2 uses in place of a ring buffer of its own; see
        // the milestone 2 design doc, "The GPS path is not a ring buffer".
        //
        // Called before begin() deliberately: addMemoryForRead() resets the
        // buffer head and tail, so calling it after bytes have arrived would
        // discard them.
        port_.addMemoryForRead(rx_storage_, sizeof(rx_storage_));
        port_.begin(baud);
    }

    int read_byte() override {
        if (port_.available() <= 0) {
            return -1;
        }
        return port_.read();
    }

    uint16_t rx_overflows() const override { return dropped_; }

    /// Call once per loop pass: a full receive buffer means bytes were lost.
    void note_buffer_state() {
        if (port_.available() >= RX_BUFFER_BYTES && dropped_ < 0xFFFF) {
            ++dropped_;
        }
    }

  private:
    /// Handed to HardwareSerial on top of the 64-byte buffer it allocates
    /// itself, for a total of 4096, matching the depth the simulation models.
    /// addMemoryForRead() adds to the built-in size rather than replacing it.
    static const size_t RX_EXTRA_BYTES = 4032;

    /// Total receive depth, and the level at which available() means bytes
    /// were lost.
    static const int RX_BUFFER_BYTES = 4096;

    HardwareSerialIMXRT &port_;
    uint8_t rx_storage_[RX_EXTRA_BYTES];
    uint16_t dropped_;
};

}  // namespace floodnet

#endif  // FLOODNET_TEENSY_GPS_HPP
