#include <floodnet/pairing.hpp>

namespace floodnet {

SamplePairer::SamplePairer(uint32_t max_skew_ms) : latest_(), max_skew_ms_(max_skew_ms) {}

void SamplePairer::submit_imu(const ImuSample &sample) { latest_ = sample; }

SensorRecord SamplePairer::pair(const GpsFix &fix, const DiagCounters &diag) const {
    SensorRecord record;
    record.gps = fix;
    record.diag = diag;

    if (latest_.valid) {
        // Wrap-safe difference. Unsigned subtraction wraps modulo 2^32, so casting the
        // result to int32_t recovers the true signed gap for any interval shorter than
        // 2^31 ms (about 24.8 days), which is far longer than any skew budget we use.
        // This is what keeps pairing correct across the millis() rollover at 49.7 days.
        const int32_t delta = static_cast<int32_t>(fix.time_ms - latest_.time_ms);
        const uint32_t skew = static_cast<uint32_t>(delta < 0 ? -delta : delta);
        if (skew <= max_skew_ms_) {
            record.imu = latest_;
        }
    }

    return record;
}

}  // namespace floodnet
