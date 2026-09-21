#ifndef FLOODNET_PAIRING_HPP
#define FLOODNET_PAIRING_HPP

#include <stdint.h>

#include <floodnet/sample.hpp>

namespace floodnet {

/// Holds the most recent IMU sample and attaches it to an arriving GPS fix,
/// provided the two are close enough in time to describe the same instant.
class SamplePairer {
  public:
    explicit SamplePairer(uint32_t max_skew_ms);

    void submit_imu(const ImuSample &sample);

    /// Builds a record from `fix`, attaching the held IMU sample when the gap
    /// between their timestamps is at most `max_skew_ms`. Otherwise the record
    /// carries an invalid IMU sample, which a consumer can detect and ignore.
    SensorRecord pair(const GpsFix &fix, const DiagCounters &diag) const;

  private:
    ImuSample latest_;
    uint32_t max_skew_ms_;
};

}  // namespace floodnet

#endif  // FLOODNET_PAIRING_HPP
