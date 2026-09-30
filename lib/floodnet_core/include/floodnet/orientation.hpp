#ifndef FLOODNET_ORIENTATION_HPP
#define FLOODNET_ORIENTATION_HPP

#include <floodnet/sample.hpp>

namespace floodnet {

/// Converts one BNO055 Euler reading, in degrees, into the centidegree fields
/// of `out`. Arguments are in the sensor's register order: heading (0x1A),
/// roll (0x1C), pitch (0x1E). Adafruit_BNO055::getEvent copies those into
/// orientation.x, .y and .z, so .y is roll and .z is pitch.
///
/// Each angle is rounded to the nearest centidegree and wrapped into
/// [-18000, 18000). The BNO055's heading runs 0 to 360 degrees, and 36000 cd
/// does not fit the int16 wire field, so a heading of 180 degrees or more
/// comes out negative: 350 degrees is -1000 cd. Roll and pitch already lie
/// within +/-180 degrees and pass through unchanged, except that exactly
/// +180 becomes -180, the same direction.
///
/// Returns false, leaving `out` untouched, if `out` is null or any angle is
/// not finite. Only the three angle fields are written.
bool bno055_euler_to_sample(float heading_deg, float roll_deg, float pitch_deg, ImuSample *out);

}  // namespace floodnet

#endif  // FLOODNET_ORIENTATION_HPP
