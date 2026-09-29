#include <stdio.h>

#include <unity.h>

#include <floodnet/downtime.hpp>

#include "../support/current_model.hpp"
#include "../support/power_experiments.hpp"
#include "../support/power_rig.hpp"

using namespace floodnet;

void setUp(void) {}
void tearDown(void) {}

// The experiment constants live in power_experiments.hpp, shared with
// test_fault_evidence so the pinned facts and the printed runs cannot drift.

static const char *build_name(Build build) {
    return build == Build::Interrupt ? "interrupt" : "duty_cycled";
}

/// 0 for the build that never sleeps, where the figure does not apply.
static uint32_t sleep_ua(Build build, uint32_t sleep_nA) {
    return build == Build::Interrupt ? 0 : sleep_nA / 1000;
}

/// A harness bound, not a result: a run that stopped before its end time
/// without an empty battery did not measure what its line claims.
static void assert_run_completed(PowerRig &rig, uint32_t end_ms) {
    TEST_ASSERT_TRUE_MESSAGE(rig.clock().now_ms() >= end_ms || rig.power().depleted(),
                             "run stopped before its end time with charge left");
}

static void print_downtime(const char *experiment, Build build, uint32_t sleep_nA,
                           const DowntimeReport &report, uint32_t scenario_ms,
                           const char *recovered) {
    for (size_t i = 0; i < DOWN_CAUSE_COUNT; ++i) {
        printf("DOWNTIME,%s,%s,%u,%s,%u,%u,%s\n", experiment, build_name(build),
               sleep_ua(build, sleep_nA), down_cause_name(static_cast<DownCause>(i)),
               report.down_ms[i], scenario_ms, recovered);
    }
}

static void run_life(Build build, uint32_t sleep_nA) {
    PowerRig rig(build, sleep_nA, NCR18650B_CAPACITY_NA_MS);
    rig.run_until(kLifeCapMs);
    // A harness bound, not a result: if a build outlives 45 days, the run
    // needs a longer timeline, not a quieter assertion.
    TEST_ASSERT_TRUE_MESSAGE(rig.power().depleted(), "battery outlived the 45-day cap");
    const uint32_t died = rig.power().depleted_at_ms();
    printf("LIFE,%s,%u,%u,%.2f\n", build_name(build), sleep_ua(build, sleep_nA), died,
           static_cast<double>(died) / kDayMs);
}

void test_experiment_1_battery_life(void) {
    run_life(Build::Interrupt, TEENSY_SLEEP_LOW_NA);
    run_life(Build::DutyCycled, TEENSY_SLEEP_LOW_NA);
    run_life(Build::DutyCycled, TEENSY_SLEEP_HIGH_NA);
}

void test_experiment_2_fault_recovery(void) {
    const Build builds[] = {Build::Interrupt, Build::DutyCycled};
    for (size_t b = 0; b < 2; ++b) {
        for (size_t f = 0; f < kFaultCount; ++f) {
            PowerRig rig(builds[b], kFaultRunSleepNA, kFaultRunCapacityNaMs);
            run_experiment_2_fault(rig, kFaults[f]);
            assert_run_completed(rig, kFaultRunMs);
            const DowntimeReport report = rig.downtime(kFaultRunMs);
            print_downtime(kFaults[f].name, builds[b], kFaultRunSleepNA, report, kFaultRunMs,
                           report.down_at_end ? "0" : "1");
        }
    }
}

void test_null_control_shows_no_downtime_after_the_first_report(void) {
    const Build builds[] = {Build::Interrupt, Build::DutyCycled};
    for (size_t b = 0; b < 2; ++b) {
        PowerRig rig(builds[b], TEENSY_SLEEP_LOW_NA, 0);
        rig.run_until(kFaultRunMs);
        const DowntimeReport report = rig.downtime(kFaultRunMs);
        print_downtime("null_control", builds[b], TEENSY_SLEEP_LOW_NA, report, kFaultRunMs, "-");
        // With no faults and no battery limit, any downtime past startup is
        // the harness producing the number, not the firmware.
        TEST_ASSERT_EQUAL_UINT32(report.down_ms[static_cast<size_t>(DownCause::Startup)],
                                 report.total_down_ms);
        // Startup absorbs everything before the first valid record, so the
        // check above alone passes a harness that delivers nothing. Ending
        // fresh needs at least one valid record to have arrived.
        TEST_ASSERT_FALSE_MESSAGE(report.down_at_end, "null control ended down");
    }
}

void test_experiment_3_combined_30_days(void) {
    const Build builds[] = {Build::Interrupt, Build::DutyCycled};
    for (size_t b = 0; b < 2; ++b) {
        PowerRig rig(builds[b], TEENSY_SLEEP_LOW_NA, NCR18650B_CAPACITY_NA_MS);
        rig.add_fault(FaultKind::LostCompletion, 2 * kDayMs, 0);
        rig.add_fault(FaultKind::SkyBlockage, 5 * kDayMs, 5 * kDayMs + 2 * kHourMs);
        rig.add_fault(FaultKind::ImuFailing, 8 * kDayMs, 8 * kDayMs + 6 * kHourMs);
        rig.add_fault(FaultKind::RadioWedge, 11 * kDayMs, 0);
        rig.add_fault(FaultKind::Hang, 14 * kDayMs, 0);
        rig.run_until(kCombinedMs);
        assert_run_completed(rig, kCombinedMs);
        print_downtime("combined_30d", builds[b], TEENSY_SLEEP_LOW_NA,
                       rig.downtime(kCombinedMs), kCombinedMs, "-");
    }
}

int main(int, char **) {
    UNITY_BEGIN();
    RUN_TEST(test_null_control_shows_no_downtime_after_the_first_report);
    RUN_TEST(test_experiment_1_battery_life);
    RUN_TEST(test_experiment_2_fault_recovery);
    RUN_TEST(test_experiment_3_combined_30_days);
    return UNITY_END();
}
