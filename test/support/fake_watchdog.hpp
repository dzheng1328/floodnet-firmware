#ifndef FLOODNET_TEST_FAKE_WATCHDOG_HPP
#define FLOODNET_TEST_FAKE_WATCHDOG_HPP

#include <stddef.h>
#include <stdint.h>

#include <floodnet/hal/watchdog.hpp>

#include "hang_breaker.hpp"
#include "sim_clock.hpp"

namespace floodnet {

/// Counts time since the last kick. Expiry is reported, not acted on: the
/// harness that owns the node sees expired() and rebuilds it, which is what a
/// reset does to RAM on hardware.
///
/// Expiry latches until on_mcu_reset(). On hardware the reset is immediate;
/// in simulation the node's code keeps running until the harness looks, and
/// a kick from that code must not cancel a reset that has already happened.
class FakeWatchdog : public IWatchdog, public ISimTick, public IHangBreaker {
  public:
    FakeWatchdog()
        : armed_(false),
          fired_(false),
          timeout_ms_(0),
          since_kick_ms_(0),
          max_since_kick_ms_(0),
          kicks_(0) {}

    void begin(uint32_t timeout_ms) override {
        armed_ = true;
        fired_ = false;
        timeout_ms_ = timeout_ms;
        since_kick_ms_ = 0;
    }

    void kick() override {
        since_kick_ms_ = 0;
        ++kicks_;
    }

    void on_tick(uint32_t elapsed_ms) override {
        if (!armed_) {
            return;
        }
        since_kick_ms_ = since_kick_ms_ > UINT32_MAX - elapsed_ms ? UINT32_MAX
                                                                   : since_kick_ms_ + elapsed_ms;
        if (since_kick_ms_ > max_since_kick_ms_) {
            max_since_kick_ms_ = since_kick_ms_;
        }
        if (since_kick_ms_ > timeout_ms_) {
            fired_ = true;
        }
    }

    bool pending() const override { return false; }

    bool expired() const { return fired_; }

    bool should_break() const override { return expired(); }

    /// WDOG1 is disabled after a reset until begin() arms it again.
    void on_mcu_reset() {
        armed_ = false;
        fired_ = false;
        since_kick_ms_ = 0;
    }

    uint32_t max_since_kick_ms() const { return max_since_kick_ms_; }
    size_t kicks() const { return kicks_; }

  private:
    bool armed_;
    bool fired_;
    uint32_t timeout_ms_;
    uint32_t since_kick_ms_;
    uint32_t max_since_kick_ms_;
    size_t kicks_;
};

}  // namespace floodnet

#endif  // FLOODNET_TEST_FAKE_WATCHDOG_HPP
