#ifndef FLOODNET_TEST_FAKE_RADIO_HPP
#define FLOODNET_TEST_FAKE_RADIO_HPP

#include <string.h>

#include <floodnet/hal/radio.hpp>

#include "sim_clock.hpp"

namespace floodnet {

/// Records what was transmitted and charges the simulated airtime.
class FakeRadio : public IRadio {
  public:
    FakeRadio(SimClock &clock, uint32_t tx_cost_ms)
        : clock_(clock), tx_cost_ms_(tx_cost_ms), sent_count_(0), last_length_(0) {
        memset(last_payload_, 0, sizeof(last_payload_));
    }

    bool transmit(const uint8_t *data, size_t len) override {
        if (len > sizeof(last_payload_)) {
            return false;
        }
        clock_.delay_ms(tx_cost_ms_);
        memcpy(last_payload_, data, len);
        last_length_ = len;
        ++sent_count_;
        return true;
    }

    int receive(uint8_t *, size_t) override { return -1; }

    size_t sent_count() const { return sent_count_; }
    size_t last_length() const { return last_length_; }
    const uint8_t *last_payload() const { return last_payload_; }

  private:
    static const size_t MAX_PAYLOAD = 64;

    SimClock &clock_;
    uint32_t tx_cost_ms_;
    size_t sent_count_;
    size_t last_length_;
    uint8_t last_payload_[MAX_PAYLOAD];
};

}  // namespace floodnet

#endif  // FLOODNET_TEST_FAKE_RADIO_HPP
