#ifndef FLOODNET_TEST_FAKE_IMU_HPP
#define FLOODNET_TEST_FAKE_IMU_HPP

#include <floodnet/hal/imu.hpp>

#include "sim_clock.hpp"

namespace floodnet {

/// Returns a fixed orientation, charging `read_cost_ms` of simulated time
/// the way a real blocking I2C transaction would.
class FakeImu : public IImuSource {
  public:
    FakeImu(SimClock &clock, uint32_t read_cost_ms)
        : clock_(clock), read_cost_ms_(read_cost_ms), reads_(0) {}

    bool read(ImuSample *out) override {
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
};

}  // namespace floodnet

#endif  // FLOODNET_TEST_FAKE_IMU_HPP
