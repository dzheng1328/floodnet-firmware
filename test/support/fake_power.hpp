#ifndef FLOODNET_TEST_FAKE_POWER_HPP
#define FLOODNET_TEST_FAKE_POWER_HPP

#include <stdint.h>

#include <floodnet/hal/power.hpp>

#include "current_model.hpp"
#include "fake_gps.hpp"
#include "fake_imu.hpp"
#include "fake_radio.hpp"
#include "sim_clock.hpp"

namespace floodnet {

/// Drives the fakes' power states and integrates the current they draw into a
/// battery. Register it as a clock observer before the fakes, so each tick is
/// charged at the state that held when the tick began.
class FakePower : public IPower, public ISimTick {
  public:
    /// Telemetry-only placeholder: a straight line, not a discharge model.
    /// No reported result depends on it.
    static const uint16_t FULL_MV = 4200;
    static const uint16_t EMPTY_MV = 3000;

    /// `capacity_nA_ms` of 0 means unlimited.
    FakePower(SimClock &clock, FakeGps &gps, FakeImu &imu, FakeRadio &radio,
              const CurrentModel &model, uint64_t capacity_nA_ms)
        : clock_(clock),
          gps_(gps),
          imu_(imu),
          radio_(radio),
          model_(model),
          capacity_(capacity_nA_ms),
          consumed_(0),
          depleted_(false),
          depleted_at_ms_(0),
          sleeping_(false),
          sleeps_(0) {}

    void set_power(Peripheral p, PowerState s) override {
        const bool on = s == PowerState::On;
        switch (p) {
        case Peripheral::Gps:
            gps_.set_powered(on);
            break;
        case Peripheral::Imu:
            imu_.set_powered(on);
            break;
        case Peripheral::Radio:
            radio_.set_powered(on);
            break;
        }
    }

    bool power_cycle(Peripheral p) override {
        switch (p) {
        case Peripheral::Gps:
            gps_.set_powered(false);
            gps_.set_powered(true);
            break;
        case Peripheral::Imu:
            imu_.power_cycle();
            break;
        case Peripheral::Radio:
            radio_.power_cycle();
            break;
        }
        return true;
    }

    void sleep_until(uint32_t wake_ms) override {
        if (static_cast<int32_t>(wake_ms - clock_.now_ms()) <= 0) {
            return;
        }
        sleeping_ = true;
        clock_.advance_to(wake_ms);
        sleeping_ = false;
        ++sleeps_;
    }

    uint16_t battery_mv() override {
        if (capacity_ == 0) {
            return FULL_MV;
        }
        const uint64_t left = consumed_ >= capacity_ ? 0 : capacity_ - consumed_;
        return static_cast<uint16_t>(EMPTY_MV + (FULL_MV - EMPTY_MV) * left / capacity_);
    }

    void on_tick(uint32_t elapsed_ms) override {
        const uint64_t current = current_nA();
        const uint64_t before = consumed_;
        consumed_ += current * elapsed_ms;
        if (capacity_ != 0 && !depleted_ && consumed_ >= capacity_) {
            // Exact, not rounded to the end of the tick: a 60 s sleep jump
            // must not move the death time by up to a minute.
            depleted_ = true;
            const uint32_t tick_start = clock_.now_ms() - elapsed_ms;
            depleted_at_ms_ =
                tick_start + static_cast<uint32_t>((capacity_ - before + current - 1) / current);
        }
    }

    bool pending() const override { return false; }

    uint64_t current_nA() const {
        uint64_t total = sleeping_ ? model_.teensy_sleep_nA : model_.teensy_awake_nA;
        if (!gps_.powered()) {
            total += model_.gps_backup_nA;
        } else {
            total += gps_.acquiring() ? model_.gps_acquisition_nA : model_.gps_tracking_nA;
        }
        total += imu_.powered() ? model_.imu_normal_nA : model_.imu_suspend_nA;
        if (!radio_.powered()) {
            total += model_.radio_sleep_nA;
        } else {
            total += radio_.transmitting() ? model_.radio_tx_nA : model_.radio_standby_nA;
        }
        return total;
    }

    bool depleted() const { return depleted_; }
    uint32_t depleted_at_ms() const { return depleted_at_ms_; }
    uint64_t consumed_nA_ms() const { return consumed_; }
    bool sleeping() const { return sleeping_; }
    size_t sleeps() const { return sleeps_; }

  private:
    SimClock &clock_;
    FakeGps &gps_;
    FakeImu &imu_;
    FakeRadio &radio_;
    CurrentModel model_;
    uint64_t capacity_;
    uint64_t consumed_;
    bool depleted_;
    uint32_t depleted_at_ms_;
    bool sleeping_;
    size_t sleeps_;
};

}  // namespace floodnet

#endif  // FLOODNET_TEST_FAKE_POWER_HPP
