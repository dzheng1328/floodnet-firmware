#ifndef FLOODNET_TEST_FAKE_IMU_HPP
#define FLOODNET_TEST_FAKE_IMU_HPP

#include <floodnet/hal/imu.hpp>

#include "sim_clock.hpp"

namespace floodnet {

/// Returns a fixed orientation, charging `read_cost_ms` of simulated time the
/// way a real blocking I2C transaction would, and signalling a fresh sample at
/// the rate the BNO055 actually produces one.
class FakeImu : public IImuSource, public ISimTick {
  public:
    /// BNO055 NDOF fusion output is fixed at 100 Hz.
    static const uint32_t SAMPLE_PERIOD_MS = 10;

    FakeImu(SimClock &clock, uint32_t read_cost_ms)
        : clock_(clock),
          read_cost_ms_(read_cost_ms),
          reads_(0),
          since_sample_ms_(0),
          ready_(true) {}

    void on_tick(uint32_t elapsed_ms) override {
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
        clock_.delay_ms(read_cost_ms_);
        ++reads_;

        out->time_ms = clock_.now_ms();
        out->yaw_cd = 1500;
        out->pitch_cd = -200;
        out->roll_cd = 50;
        out->valid = true;
        return true;
    }

    size_t reads() const { return reads_; }

  private:
    SimClock &clock_;
    uint32_t read_cost_ms_;
    size_t reads_;
    uint32_t since_sample_ms_;
    bool ready_;
};

}  // namespace floodnet

#endif  // FLOODNET_TEST_FAKE_IMU_HPP
