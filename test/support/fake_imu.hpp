#ifndef FLOODNET_TEST_FAKE_IMU_HPP
#define FLOODNET_TEST_FAKE_IMU_HPP

#include <floodnet/hal/imu.hpp>

#include "hang_breaker.hpp"
#include "sim_clock.hpp"

namespace floodnet {

/// Returns a fixed orientation, charging `read_cost_ms` of simulated time the
/// way a real blocking I2C transaction would, and signalling a fresh sample at
/// the rate the BNO055 actually produces one.
class FakeImu : public IImuSource, public ISimTick {
  public:
    /// BNO055 NDOF fusion output is fixed at 100 Hz.
    static const uint32_t SAMPLE_PERIOD_MS = 10;

    /// How far a hung read advances the clock between breaker checks.
    static const uint32_t HANG_STEP_MS = 100;

    FakeImu(SimClock &clock, uint32_t read_cost_ms)
        : clock_(clock),
          read_cost_ms_(read_cost_ms),
          reads_(0),
          since_sample_ms_(0),
          ready_(true),
          powered_(true),
          failing_(false),
          hang_(nullptr),
          power_cycles_(0) {}

    void on_tick(uint32_t elapsed_ms) override {
        if (!powered_) {
            return;  // suspended: no fusion output
        }
        since_sample_ms_ += elapsed_ms;
        if (since_sample_ms_ >= SAMPLE_PERIOD_MS) {
            since_sample_ms_ = 0;
            ready_ = true;
        }
    }

    bool pending() const override { return ready_; }

    bool data_ready() const override { return ready_; }

    bool read(ImuSample *out) override {
        // Cleared before the transaction rather than after. A sample becoming
        // available during the read is a real event on hardware, and clearing
        // afterwards would swallow it.
        ready_ = false;

        if (hang_ != nullptr) {
            // A read that never returns, until something outside it (the
            // watchdog, or the end of the run) says stop.
            const IHangBreaker *breaker = hang_;
            hang_ = nullptr;
            while (!breaker->should_break()) {
                clock_.delay_ms(HANG_STEP_MS);
            }
            return false;
        }
        if (!powered_) {
            return false;
        }

        clock_.delay_ms(read_cost_ms_);
        ++reads_;
        if (failing_) {
            return false;
        }

        out->time_ms = clock_.now_ms();
        out->yaw_cd = 1500;
        out->pitch_cd = -200;
        out->roll_cd = 50;
        out->valid = true;
        return true;
    }

    size_t reads() const { return reads_; }

    /// Off models BNO055 suspend. On primes a sample, as TeensyImu::resume()
    /// does. Only changes of state do anything.
    void set_powered(bool on) {
        if (on == powered_) {
            return;
        }
        powered_ = on;
        ready_ = on;
        since_sample_ms_ = 0;
    }

    bool powered() const { return powered_; }

    /// Reads cost their time and then fail, as a NACKing bus would. A power
    /// cycle does not clear this: the fault is outside the sensor.
    void set_failing(bool failing) { failing_ = failing; }

    /// The next read hangs until `breaker` says stop. One-shot.
    void hang_next_read(const IHangBreaker *breaker) { hang_ = breaker; }

    void power_cycle() {
        set_powered(false);
        set_powered(true);
        ++power_cycles_;
    }

    size_t power_cycles() const { return power_cycles_; }

  private:
    SimClock &clock_;
    uint32_t read_cost_ms_;
    size_t reads_;
    uint32_t since_sample_ms_;
    bool ready_;
    bool powered_;
    bool failing_;
    const IHangBreaker *hang_;
    size_t power_cycles_;
};

}  // namespace floodnet

#endif  // FLOODNET_TEST_FAKE_IMU_HPP
