#ifndef FLOODNET_TEST_FAKE_GPS_HPP
#define FLOODNET_TEST_FAKE_GPS_HPP

#include <string.h>

#include <floodnet/hal/gps.hpp>

#include "sim_clock.hpp"

namespace floodnet {

/// Emits a sentence repeatedly at a fixed byte rate into a small FIFO,
/// modelling a UART peripheral that discards bytes nobody collected.
class FakeGps : public IGpsSource, public ISimTick {
  public:
    FakeGps(const char *sentence, double bytes_per_ms)
        : sentence_(sentence),
          sentence_len_(strlen(sentence)),
          source_index_(0),
          bytes_per_ms_(bytes_per_ms),
          pending_(0.0),
          head_(0),
          tail_(0),
          count_(0),
          dropped_(0) {}

    void on_tick(uint32_t elapsed_ms) override {
        pending_ += static_cast<double>(elapsed_ms) * bytes_per_ms_;
        while (pending_ >= 1.0) {
            push(sentence_[source_index_]);
            source_index_ = (source_index_ + 1) % sentence_len_;
            pending_ -= 1.0;
        }
    }

    int read_byte() override {
        if (count_ == 0) {
            return -1;
        }
        const uint8_t value = fifo_[tail_];
        tail_ = (tail_ + 1) % FIFO_DEPTH;
        --count_;
        return static_cast<int>(value);
    }

    uint16_t rx_overflows() const override { return dropped_; }

  private:
    /// Matches the 64-byte software receive buffer Teensy 4.x HardwareSerial
    /// keeps, which is the buffer `TeensyGps` watches for overflow. At 9600
    /// baud it holds roughly 66 ms of traffic, so a loop that stalls longer
    /// than that loses bytes.
    static const size_t FIFO_DEPTH = 64;

    void push(char value) {
        if (count_ == FIFO_DEPTH) {
            if (dropped_ < 0xFFFF) {
                ++dropped_;
            }
            return;
        }
        fifo_[head_] = static_cast<uint8_t>(value);
        head_ = (head_ + 1) % FIFO_DEPTH;
        ++count_;
    }

    const char *sentence_;
    size_t sentence_len_;
    size_t source_index_;
    double bytes_per_ms_;
    double pending_;

    uint8_t fifo_[FIFO_DEPTH];
    size_t head_;
    size_t tail_;
    size_t count_;
    uint16_t dropped_;
};

}  // namespace floodnet

#endif  // FLOODNET_TEST_FAKE_GPS_HPP
