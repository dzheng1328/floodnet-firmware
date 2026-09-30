#include <floodnet/orientation.hpp>

#include <math.h>

namespace floodnet {

namespace {

const long FULL_TURN_CD = 36000;
const long HALF_TURN_CD = 18000;

int16_t wrap_to_cd(float deg) {
    long cd = lround(static_cast<double>(deg) * 100.0) % FULL_TURN_CD;
    if (cd >= HALF_TURN_CD) {
        cd -= FULL_TURN_CD;
    } else if (cd < -HALF_TURN_CD) {
        cd += FULL_TURN_CD;
    }
    return static_cast<int16_t>(cd);
}

}  // namespace

bool bno055_euler_to_sample(float heading_deg, float roll_deg, float pitch_deg, ImuSample *out) {
    if (out == nullptr || !isfinite(heading_deg) || !isfinite(roll_deg) || !isfinite(pitch_deg)) {
        return false;
    }
    out->yaw_cd = wrap_to_cd(heading_deg);
    out->roll_cd = wrap_to_cd(roll_deg);
    out->pitch_cd = wrap_to_cd(pitch_deg);
    return true;
}

}  // namespace floodnet
