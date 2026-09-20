#ifndef FLOODNET_TEST_SIM_CLOCK_HPP
#define FLOODNET_TEST_SIM_CLOCK_HPP

#include <assert.h>
#include <stddef.h>
#include <stdint.h>

#include <floodnet/hal/clock.hpp>

namespace floodnet {

/// Anything that needs to advance when simulated time passes.
class ISimTick {
  public:
    virtual ~ISimTick() {}
    virtual void on_tick(uint32_t elapsed_ms) = 0;
};

/// A clock that only moves when something asks it to, notifying observers.
/// This is what lets a blocking HAL call have a visible cost elsewhere.
class SimClock : public IClock {
  public:
    SimClock() : now_ms_(0), observer_count_(0) {}

    void add_observer(ISimTick *observer) {
        // Silently dropping an observer would make a future test fail with no
        // indication why its fake stopped receiving ticks. Fail loudly instead.
        assert(observer_count_ < MAX_OBSERVERS && "SimClock observer capacity exceeded");
        observers_[observer_count_++] = observer;
    }

    uint32_t now_ms() override { return now_ms_; }

    void delay_ms(uint32_t ms) override {
        now_ms_ += ms;
        for (size_t i = 0; i < observer_count_; ++i) {
            observers_[i]->on_tick(ms);
        }
    }

  private:
    static const size_t MAX_OBSERVERS = 4;
    uint32_t now_ms_;
    ISimTick *observers_[MAX_OBSERVERS];
    size_t observer_count_;
};

}  // namespace floodnet

#endif  // FLOODNET_TEST_SIM_CLOCK_HPP
