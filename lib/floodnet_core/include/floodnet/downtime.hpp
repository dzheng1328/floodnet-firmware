#ifndef FLOODNET_DOWNTIME_HPP
#define FLOODNET_DOWNTIME_HPP

#include <stddef.h>
#include <stdint.h>

namespace floodnet {

/// Why a node was down. See the milestone 3 design doc, "Cause attribution".
enum class DownCause : uint8_t {
    Startup = 0,  ///< before the first valid record of the run
    Battery,      ///< the node had died; known to a simulation, not to a gateway
    NoGps,        ///< heartbeats (gps_valid = 0) arrived during the interval
    Radio,        ///< the record ending the interval shows more tx_timeouts
    Reboot,       ///< the record ending the interval shows a new boot_count
    Silent,       ///< down, and nothing in-band explains why
};
const size_t DOWN_CAUSE_COUNT = 6;

/// What a gateway knows about one received packet.
struct GatewayRecord {
    uint32_t arrival_ms = 0;
    bool gps_valid = false;
    uint16_t boot_count = 0;
    uint16_t tx_timeouts = 0;
};

struct DowntimeInput {
    const GatewayRecord *records = nullptr;  ///< in arrival order
    size_t count = 0;
    /// Run length, starting at 0. Times are plain uint32 milliseconds, so a
    /// run must stay under 2^32 ms (49.7 days); runs here are at most 45 days.
    uint32_t scenario_ms = 0;
    /// Down once the newest valid record is older than this: two intervals.
    uint32_t stale_after_ms = 600000;
    bool died = false;
    uint32_t died_at_ms = 0;
};

struct DowntimeReport {
    uint32_t down_ms[DOWN_CAUSE_COUNT] = {};
    uint32_t total_down_ms = 0;
    bool down_at_end = false;  ///< still down when the run ended
};

/// Total down time and its split by cause. A node is down at time t when its
/// newest gps_valid record arrived more than stale_after_ms before t, or when
/// it has none yet. A record exactly stale_after_ms old is not down.
DowntimeReport compute_downtime(const DowntimeInput &in);

const char *down_cause_name(DownCause cause);

}  // namespace floodnet

#endif  // FLOODNET_DOWNTIME_HPP
