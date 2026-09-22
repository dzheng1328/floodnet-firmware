#ifndef FLOODNET_HAL_IMU_HPP
#define FLOODNET_HAL_IMU_HPP

#include <floodnet/sample.hpp>

namespace floodnet {

class IImuSource {
  public:
    virtual ~IImuSource() {}

    /// Blocking read of the current orientation. False if the sensor did not answer.
    virtual bool read(ImuSample *out) = 0;

    /// True when a fresh sample is waiting to be collected.
    ///
    /// The Teensy implementation reflects a flag set by an interrupt handler
    /// that does nothing else. Retrieving the sample needs a blocking I2C
    /// transaction, which must not run in interrupt context, so the read stays
    /// in main context and clears the flag there.
    ///
    /// Note that the signal is a timer at the sensor's fusion output rate, not
    /// a pin on the sensor. The BNO055 has no data-ready interrupt to offer;
    /// see the comment on TeensyImu and the milestone 2 design doc.
    virtual bool data_ready() const = 0;
};

}  // namespace floodnet

#endif  // FLOODNET_HAL_IMU_HPP
