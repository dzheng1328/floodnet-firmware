#ifndef FLOODNET_TEST_FAKE_RADIO_HPP
#define FLOODNET_TEST_FAKE_RADIO_HPP

#include <string.h>

#include <floodnet/hal/radio.hpp>

#include "sim_clock.hpp"

namespace floodnet {

/// Records what was transmitted and charges the simulated airtime.
class FakeRadio : public IAsyncRadio, public ISimTick {
  public:
    FakeRadio(SimClock &clock, uint32_t tx_cost_ms)
        : clock_(clock),
          tx_cost_ms_(tx_cost_ms),
          sent_count_(0),
          last_length_(0),
          busy_(false),
          remaining_ms_(0),
          lose_next_completion_(false),
          powered_(true),
          wedged_(false),
          refusing_(false),
          power_cycles_(0) {
        memset(last_payload_, 0, sizeof(last_payload_));
    }

    bool transmit(const uint8_t *data, size_t len) override {
        // The blocking and async paths must not be mixed on one instance.
        // This call's delay_ms ticks every observer, including this radio
        // when it is registered as one, so an in-flight begin_transmit would
        // complete as a side effect and then have its payload overwritten
        // below.
        //
        // Refusing rather than asserting, and rather than proceeding: every
        // caller already branches on this return (see
        // PollingSampler::step() in src/sampler_polling.cpp), so a refusal shows up as an
        // uncounted packet rather than a corrupted measurement. It also
        // matches begin_transmit(), which refuses-when-busy the same way.
        if (!powered_) {
            return false;
        }
        if (busy_) {
            return false;
        }
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

    bool begin_transmit(const uint8_t *data, size_t len) override {
        if (!powered_ || refusing_ || busy_ || len > sizeof(last_payload_)) {
            return false;
        }
        memcpy(last_payload_, data, len);
        last_length_ = len;
        busy_ = true;
        // Wedged: accepted, sent to the modem, never reported finished.
        remaining_ms_ = wedged_ ? UINT32_MAX : tx_cost_ms_;
        return true;
    }

    bool tx_busy() override { return busy_; }

    void abort_transmit() override {
        busy_ = false;
        remaining_ms_ = 0;
    }

    /// Simulates a lost DIO0 edge: the next transmission goes out, but the
    /// radio never reports it finished, so tx_busy() stays true until
    /// abort_transmit(). One-shot, so a sampler that recovers can be seen to.
    void lose_next_completion() { lose_next_completion_ = true; }

    /// Off models RH_RF95::sleep(): a transmission in progress is abandoned.
    void set_powered(bool on) {
        powered_ = on;
        if (!on) {
            busy_ = false;
            remaining_ms_ = 0;
        }
    }

    bool powered() const { return powered_; }

    /// Every transmission from now on hangs, until power_cycle().
    void set_wedged(bool wedged) { wedged_ = wedged; }

    /// begin_transmit() refuses outright while set.
    void set_refusing(bool refusing) { refusing_ = refusing; }

    /// Models a reset-pin pulse and re-init: clears a wedge, leaves it on.
    void power_cycle() {
        busy_ = false;
        remaining_ms_ = 0;
        wedged_ = false;
        powered_ = true;
        ++power_cycles_;
    }

    size_t power_cycles() const { return power_cycles_; }

    /// Const view of tx_busy() for energy accounting.
    bool transmitting() const { return busy_; }

    void on_tick(uint32_t elapsed_ms) override {
        if (!busy_ || remaining_ms_ == UINT32_MAX) {
            return;
        }
        if (elapsed_ms >= remaining_ms_) {
            if (lose_next_completion_) {
                lose_next_completion_ = false;
                remaining_ms_ = UINT32_MAX;  // wedged: busy until aborted
                return;
            }
            remaining_ms_ = 0;
            busy_ = false;
            ++sent_count_;  // counted on completion, as the blocking path does
        } else {
            remaining_ms_ -= elapsed_ms;
        }
    }

    /// Always false, and deliberately.
    ///
    /// "Pending" means a source has something the loop can act on now. A
    /// completed transmit is not that: the loop notices it by checking
    /// tx_busy() on a later pass, exactly as __WFI plus the 1 kHz systick
    /// behaves on hardware. Returning true whenever the radio is idle would
    /// make wait_for_event return immediately on every pass, and simulated
    /// time would stop advancing.
    bool pending() const override { return false; }

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
    bool busy_;
    uint32_t remaining_ms_;
    bool lose_next_completion_;
    bool powered_;
    bool wedged_;
    bool refusing_;
    size_t power_cycles_;
};

}  // namespace floodnet

#endif  // FLOODNET_TEST_FAKE_RADIO_HPP
