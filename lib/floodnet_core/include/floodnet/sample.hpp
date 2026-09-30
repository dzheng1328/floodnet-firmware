#ifndef FLOODNET_SAMPLE_HPP
#define FLOODNET_SAMPLE_HPP

#include <stdint.h>

namespace floodnet {

/// A single GPS position fix, timestamped with the node's local clock.
struct GpsFix {
    uint32_t time_ms = 0;
    int32_t lat_1e7 = 0;   ///< degrees x 1e7
    int32_t lon_1e7 = 0;   ///< degrees x 1e7
    int32_t alt_mm = 0;    ///< millimetres above mean sea level
    uint8_t satellites = 0;
    bool valid = false;
};

/// A single IMU orientation sample, timestamped with the node's local clock.
struct ImuSample {
    uint32_t time_ms = 0;
    int16_t yaw_cd = 0;    ///< centidegrees, [-18000, 18000); see bno055_euler_to_sample()
    int16_t pitch_cd = 0;  ///< centidegrees, [-18000, 18000)
    int16_t roll_cd = 0;   ///< centidegrees, [-18000, 18000)
    bool valid = false;
};

/// Health counters carried in-band so a receiver can see loss at the source.
struct DiagCounters {
    uint16_t drops = 0;        ///< the node's IGpsSource::rx_overflows() value; see that
                                ///< contract for the unit, which differs by implementation
    uint16_t crc_errors = 0;   ///< Reserved. Always zero: a node only transmits and never
                                ///< receives a radio packet, so it has nothing to fail a CRC
                                ///< check on. Carried by v0x01 only; v0x02 drops it.
};

/// One paired observation, the unit this network transports.
struct SensorRecord {
    GpsFix gps;
    ImuSample imu;
    DiagCounters diag;
};

}  // namespace floodnet

#endif  // FLOODNET_SAMPLE_HPP
