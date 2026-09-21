#ifndef FLOODNET_HAL_IMU_HPP
#define FLOODNET_HAL_IMU_HPP

#include <floodnet/sample.hpp>

namespace floodnet {

class IImuSource {
  public:
    virtual ~IImuSource() {}

    /// Blocking read of the current orientation. False if the sensor did not answer.
    virtual bool read(ImuSample *out) = 0;
};

}  // namespace floodnet

#endif  // FLOODNET_HAL_IMU_HPP
