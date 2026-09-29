#ifndef FLOODNET_TEST_SIM_CLOCK_HPP
#define FLOODNET_TEST_SIM_CLOCK_HPP

#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>

#include <floodnet/hal/clock.hpp>

namespace floodnet {

/// Anything that needs to advance when simulated time passes.
class ISimTick {
  public:
    virtual ~ISimTick() {}
    virtual void on_tick(uint32_t elapsed_ms) = 0;

    /// True when this source has something the consumer could act on right
    /// now. `SimClock::wait_for_event` stops advancing as soon as any source
    /// says yes.
    virtual bool pending() const = 0;
};

/// A clock that only moves when something asks it to, notifying observers.
/// This is what lets a blocking HAL call have a visible cost elsewhere.
class SimClock : public IClock {
  public:
    SimClock() : now_ms_(0), observer_count_(0) {}

    void add_observer(ISimTick *observer) {
        // Silently dropping an observer would make a future test fail with no
        // indication why its fake stopped receiving ticks. Fail loudly instead,
        // and not with assert(): NDEBUG would strip it and leave the write
        // below running off the end of observers_.
        if (observer_count_ >= MAX_OBSERVERS) {
            fprintf(stderr, "SimClock observer capacity exceeded\n");
            abort();
        }
        observers_[observer_count_++] = observer;
    }

    uint32_t now_ms() override { return now_ms_; }

    void delay_ms(uint32_t ms) override {
        now_ms_ += ms;
        for (size_t i = 0; i < observer_count_; ++i) {
            observers_[i]->on_tick(ms);
        }
    }

    void wait_for_event(uint32_t max_ms) override {
        // Advance a millisecond at a time rather than jumping to the next
        // scheduled event, so observers keep receiving the same on_tick
        // cadence they get from delay_ms. That is what keeps the polling
        // sampler's measured behaviour identical to milestone 1.
        for (uint32_t elapsed = 0; elapsed < max_ms; ++elapsed) {
            if (any_pending()) {
                return;
            }
            delay_ms(1);
        }
    }

    /// Moves straight to `t_ms` in one on_tick. Used for sleep, where 1 ms
    /// steps would make a 30-day run take billions of iterations. An observer
    /// that is powered down ignores the elapsed time; one that is not sees a
    /// single large tick. Does nothing if `t_ms` is not in the future.
    void advance_to(uint32_t t_ms) {
        const int32_t ahead = static_cast<int32_t>(t_ms - now_ms_);
        if (ahead > 0) {
            delay_ms(static_cast<uint32_t>(ahead));
        }
    }

    bool any_pending() const {
        for (size_t i = 0; i < observer_count_; ++i) {
            if (observers_[i]->pending()) {
                return true;
            }
        }
        return false;
    }

  private:
    static const size_t MAX_OBSERVERS = 8;
    uint32_t now_ms_;
    ISimTick *observers_[MAX_OBSERVERS];
    size_t observer_count_;
};

}  // namespace floodnet

#endif  // FLOODNET_TEST_SIM_CLOCK_HPP
