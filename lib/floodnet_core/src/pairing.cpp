#include <floodnet/pairing.hpp>

namespace floodnet {

SamplePairer::SamplePairer(uint32_t max_skew_ms) : latest_(), max_skew_ms_(max_skew_ms) {}

void SamplePairer::submit_imu(const ImuSample &sample) { latest_ = sample; }

SensorRecord SamplePairer::pair(const GpsFix &fix, const DiagCounters &diag) const {
    SensorRecord record;
    record.gps = fix;
    record.diag = diag;

    if (latest_.valid) {
        // Unsigned subtraction, so order the operands rather than using abs().
        const uint32_t skew = (fix.time_ms > latest_.time_ms) ? (fix.time_ms - latest_.time_ms)
                                                              : (latest_.time_ms - fix.time_ms);
        if (skew <= max_skew_ms_) {
            record.imu = latest_;
        }
    }

    return record;
}

}  // namespace floodnet
