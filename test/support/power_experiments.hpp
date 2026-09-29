#ifndef FLOODNET_TEST_POWER_EXPERIMENTS_HPP
#define FLOODNET_TEST_POWER_EXPERIMENTS_HPP

#include <stddef.h>
#include <stdint.h>

#include "current_model.hpp"
#include "power_rig.hpp"

namespace floodnet {

// Frozen in the milestone 3 design doc, "Experiments", before any of this ran.
// Changing any of these after seeing output must be recorded in the README.
// Shared by test_power_bench, which prints the results, and test_fault_evidence,
// which pins what the README says about the same runs, so the two cannot drift.
const uint32_t kHourMs = 3600000UL;
const uint32_t kDayMs = 24UL * kHourMs;
const uint32_t kLifeCapMs = 45UL * kDayMs;  // uint32 ms tops out at 49.7 days
const uint32_t kFaultRunMs = 24UL * kHourMs;
const uint32_t kFaultAtMs = 2UL * kHourMs;
const uint32_t kCombinedMs = 30UL * kDayMs;

struct FaultCase {
    const char *name;
    FaultKind kind;
    uint32_t duration_ms;  // 0: does not end by itself
};

const FaultCase kFaults[] = {
    {"fault_lost_completion", FaultKind::LostCompletion, 0},
    {"fault_sky_blockage", FaultKind::SkyBlockage, 2UL * 3600000UL},
    {"fault_imu_failing", FaultKind::ImuFailing, 6UL * 3600000UL},
    {"fault_radio_wedge", FaultKind::RadioWedge, 0},
    {"fault_hang", FaultKind::Hang, 0},
};
const size_t kFaultCount = sizeof(kFaults) / sizeof(kFaults[0]);

inline const FaultCase &fault_case(FaultKind kind) {
    for (size_t i = 0; i < kFaultCount; ++i) {
        if (kFaults[i].kind == kind) {
            return kFaults[i];
        }
    }
    return kFaults[0];  // unreachable: every FaultKind has a row
}

/// Experiment 2's rig configuration: the low sleep current, no battery limit.
const uint32_t kFaultRunSleepNA = TEENSY_SLEEP_LOW_NA;
const uint64_t kFaultRunCapacityNaMs = 0;  // unlimited

/// Experiment 2's run of one fault, on a rig built as
/// PowerRig(build, kFaultRunSleepNA, kFaultRunCapacityNaMs): the fault starts
/// at 2 h and the run lasts 24 h.
inline void run_experiment_2_fault(PowerRig &rig, const FaultCase &fault) {
    const uint32_t end = fault.duration_ms == 0 ? 0 : kFaultAtMs + fault.duration_ms;
    rig.add_fault(fault.kind, kFaultAtMs, end);
    rig.run_until(kFaultRunMs);
}

}  // namespace floodnet

#endif  // FLOODNET_TEST_POWER_EXPERIMENTS_HPP
