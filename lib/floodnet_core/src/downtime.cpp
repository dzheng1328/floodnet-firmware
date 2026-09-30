#include <floodnet/downtime.hpp>

namespace floodnet {

namespace {

void add(DowntimeReport *report, DownCause cause, uint32_t ms) {
    report->down_ms[static_cast<size_t>(cause)] += ms;
    report->total_down_ms += ms;
}

/// Attributes [start, end). `prev` is the last valid record before the
/// interval (null before the first); `next` the valid record that ends it
/// (null when the run ends first). `heartbeat` is whether a gps_valid = 0
/// record arrived inside the interval.
void attribute(DowntimeReport *report, const DowntimeInput &in, uint32_t start, uint32_t end,
               const GatewayRecord *prev, const GatewayRecord *next, bool heartbeat) {
    if (end <= start) {
        return;
    }

    uint32_t alive_end = end;
    if (in.died && in.died_at_ms < end) {
        const uint32_t dead_from = in.died_at_ms > start ? in.died_at_ms : start;
        add(report, DownCause::Battery, end - dead_from);
        alive_end = dead_from;
    }
    if (alive_end <= start) {
        return;
    }

    DownCause cause = DownCause::Silent;
    if (prev == nullptr) {
        cause = DownCause::Startup;
    } else if (heartbeat) {
        cause = DownCause::NoGps;
    } else if (next != nullptr && next->boot_count != prev->boot_count) {
        cause = DownCause::Reboot;
    } else if (next != nullptr && next->tx_timeouts > prev->tx_timeouts) {
        cause = DownCause::Radio;
    }
    add(report, cause, alive_end - start);
}

}  // namespace

DowntimeReport compute_downtime(const DowntimeInput &in) {
    DowntimeReport report;
    const GatewayRecord *prev = nullptr;
    uint32_t fresh_until = 0;
    bool have_heartbeat = false;
    uint32_t last_heartbeat_ms = 0;

    for (size_t i = 0; i < in.count; ++i) {
        const GatewayRecord &record = in.records[i];
        if (record.arrival_ms >= in.scenario_ms) {
            break;
        }
        if (!record.gps_valid) {
            have_heartbeat = true;
            last_heartbeat_ms = record.arrival_ms;
            continue;
        }
        // Heartbeats arrive in order, so "any heartbeat inside the interval"
        // is "the latest one is at or after its start".
        const bool heartbeat = have_heartbeat && last_heartbeat_ms >= fresh_until;
        attribute(&report, in, fresh_until, record.arrival_ms, prev, &record, heartbeat);
        prev = &record;
        have_heartbeat = false;
        fresh_until = record.arrival_ms + in.stale_after_ms;
    }

    if (fresh_until < in.scenario_ms) {
        const bool heartbeat = have_heartbeat && last_heartbeat_ms >= fresh_until;
        attribute(&report, in, fresh_until, in.scenario_ms, prev, nullptr, heartbeat);
        report.down_at_end = true;
    }
    return report;
}

const char *down_cause_name(DownCause cause) {
    switch (cause) {
    case DownCause::Startup:
        return "startup";
    case DownCause::Battery:
        return "battery";
    case DownCause::NoGps:
        return "no_gps";
    case DownCause::Radio:
        return "radio";
    case DownCause::Reboot:
        return "reboot";
    case DownCause::Silent:
        return "silent";
    }
    return "unknown";
}

}  // namespace floodnet
