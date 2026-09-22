#ifndef FLOODNET_TEST_FAKE_GPS_HPP
#define FLOODNET_TEST_FAKE_GPS_HPP

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <floodnet/hal/gps.hpp>

#include "sim_clock.hpp"

namespace floodnet {

/// Emits a sentence repeatedly at a fixed byte rate into a small FIFO,
/// modelling a UART peripheral that discards bytes nobody collected.
class FakeGps : public IGpsSource, public ISimTick {
  public:
    /// Teensy 4.x HardwareSerial allocates a 64-byte software receive buffer.
    /// At 9600 baud that holds roughly 66 ms of traffic, so a loop stalling
    /// longer than that loses bytes. This is the milestone 1 configuration.
    static const size_t DEFAULT_FIFO_DEPTH = 64;

    /// The milestone 2 configuration, supplied to HardwareSerial through
    /// addMemoryForRead(). Sized to cover a full SF12 transmit: 960 bytes per
    /// second against 3023 ms is about 2902 bytes, rounded up to a power of two.
    static const size_t MAX_FIFO_DEPTH = 4096;

    FakeGps(const char *sentence, double bytes_per_ms,
            size_t fifo_depth = DEFAULT_FIFO_DEPTH)
        : sentence_(sentence),
          sentence_len_(strlen(sentence)),
          source_index_(0),
          bytes_per_ms_(bytes_per_ms),
          pending_(0.0),
          fifo_depth_(fifo_depth),
          head_(0),
          tail_(0),
          count_(0),
          dropped_(0) {
        // Not assert(): this guards the fifo_ indexing below, and a guard
        // that NDEBUG can strip would turn a bad depth into memory corruption.
        if (fifo_depth == 0 || fifo_depth > MAX_FIFO_DEPTH) {
            fprintf(stderr, "FakeGps depth must be between 1 and MAX_FIFO_DEPTH\n");
            abort();
        }
    }

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
        tail_ = (tail_ + 1) % fifo_depth_;
        --count_;
        return static_cast<int>(value);
    }

    uint16_t rx_overflows() const override { return dropped_; }

    bool pending() const override { return count_ > 0; }

  private:
    void push(char value) {
        if (count_ == fifo_depth_) {
            if (dropped_ < 0xFFFF) {
                ++dropped_;
            }
            return;
        }
        fifo_[head_] = static_cast<uint8_t>(value);
        head_ = (head_ + 1) % fifo_depth_;
        ++count_;
    }

    const char *sentence_;
    size_t sentence_len_;
    size_t source_index_;
    double bytes_per_ms_;
    double pending_;

    size_t fifo_depth_;
    uint8_t fifo_[MAX_FIFO_DEPTH];
    size_t head_;
    size_t tail_;
    size_t count_;
    uint16_t dropped_;
};

}  // namespace floodnet

#endif  // FLOODNET_TEST_FAKE_GPS_HPP
