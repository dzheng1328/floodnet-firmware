#include <stddef.h>
#include <stdint.h>

#include <unity.h>

#include <floodnet/downtime.hpp>
#include <floodnet/packet.hpp>

#include "../support/power_experiments.hpp"
#include "../support/power_rig.hpp"

using namespace floodnet;

void setUp(void) {}
void tearDown(void) {}

// Facts the README states about experiment 2's runs that its DOWNTIME lines
// do not show. Each run is experiment 2's own configuration, through the
// helper test_power_bench uses, so a fact here is a fact about that run.

static void run_fault(PowerRig &rig, FaultKind kind) {
    run_experiment_2_fault(rig, fault_case(kind));
    TEST_ASSERT_EQUAL_UINT32(1, rig.fault_count());
    TEST_ASSERT_TRUE(rig.fault_started(0));
}

/// Largest arrival gap between consecutive gps_valid records.
static uint32_t largest_valid_gap(const PowerRig &rig) {
    const std::vector<GatewayRecord> &records = rig.records();
    uint32_t largest = 0;
    bool have_prev = false;
    uint32_t prev_ms = 0;
    for (size_t i = 0; i < records.size(); ++i) {
        if (!records[i].gps_valid) {
            continue;
        }
        if (have_prev && records[i].arrival_ms - prev_ms > largest) {
            largest = records[i].arrival_ms - prev_ms;
        }
        have_prev = true;
        prev_ms = records[i].arrival_ms;
    }
    return largest;
}

/// Sequence numbers skipped between consecutive deliveries of the same boot.
static uint32_t missing_seqs(const PowerRig &rig) {
    const std::vector<Packet> &packets = rig.deliveries();
    uint32_t missing = 0;
    for (size_t i = 1; i < packets.size(); ++i) {
        if (packets[i].boot_count != packets[i - 1].boot_count) {
            continue;
        }
        TEST_ASSERT_TRUE(packets[i].seq > packets[i - 1].seq);
        missing += packets[i].seq - packets[i - 1].seq - 1;
    }
    return missing;
}

static uint16_t largest_tx_timeouts(const PowerRig &rig) {
    uint16_t largest = 0;
    for (size_t i = 0; i < rig.records().size(); ++i) {
        if (rig.records()[i].tx_timeouts > largest) {
            largest = rig.records()[i].tx_timeouts;
        }
    }
    return largest;
}

static size_t heartbeats(const PowerRig &rig) {
    size_t count = 0;
    for (size_t i = 0; i < rig.records().size(); ++i) {
        if (!rig.records()[i].gps_valid) {
            ++count;
        }
    }
    return count;
}

void test_dc_lost_completion_costs_exactly_one_report(void) {
    PowerRig rig(Build::DutyCycled, kFaultRunSleepNA, kFaultRunCapacityNaMs);
    run_fault(rig, FaultKind::LostCompletion);
    TEST_ASSERT_EQUAL_UINT32(600000, largest_valid_gap(rig));
    TEST_ASSERT_EQUAL_UINT32(1, missing_seqs(rig));
    TEST_ASSERT_EQUAL_UINT16(1, largest_tx_timeouts(rig));
    TEST_ASSERT_EQUAL_size_t(0, rig.reboots());
}

void test_dc_hang_reboots_once_without_going_stale(void) {
    PowerRig rig(Build::DutyCycled, kFaultRunSleepNA, kFaultRunCapacityNaMs);
    run_fault(rig, FaultKind::Hang);
    TEST_ASSERT_EQUAL_size_t(1, rig.reboots());
    TEST_ASSERT_FALSE(rig.deliveries().empty());
    TEST_ASSERT_EQUAL_UINT16(1, rig.deliveries().front().boot_count);
    TEST_ASSERT_EQUAL_UINT16(2, rig.deliveries().back().boot_count);
    TEST_ASSERT_EQUAL_UINT32(390100, largest_valid_gap(rig));
    const DowntimeReport report = rig.downtime(kFaultRunMs);
    TEST_ASSERT_EQUAL_UINT32(0, report.down_ms[static_cast<size_t>(DownCause::Reboot)]);
}

void test_dc_radio_wedge_recovers_after_three_timeouts(void) {
    PowerRig rig(Build::DutyCycled, kFaultRunSleepNA, kFaultRunCapacityNaMs);
    run_fault(rig, FaultKind::RadioWedge);
    TEST_ASSERT_EQUAL_UINT16(3, largest_tx_timeouts(rig));
    TEST_ASSERT_EQUAL_size_t(1, rig.radio().power_cycles());
    TEST_ASSERT_EQUAL_UINT32(1200000, largest_valid_gap(rig));
    TEST_ASSERT_EQUAL_UINT32(3, missing_seqs(rig));
}

void test_interrupt_lost_completion_loses_seconds(void) {
    PowerRig rig(Build::Interrupt, kFaultRunSleepNA, kFaultRunCapacityNaMs);
    run_fault(rig, FaultKind::LostCompletion);
    TEST_ASSERT_EQUAL_UINT32(8050, largest_valid_gap(rig));
}

void test_dc_imu_fault_is_applied(void) {
    PowerRig rig(Build::DutyCycled, kFaultRunSleepNA, kFaultRunCapacityNaMs);
    run_fault(rig, FaultKind::ImuFailing);
    size_t imu_invalid = 0;
    for (size_t i = 0; i < rig.deliveries().size(); ++i) {
        if (!rig.deliveries()[i].record.imu.valid) {
            ++imu_invalid;
        }
    }
    TEST_ASSERT_EQUAL_size_t(72, imu_invalid);
    TEST_ASSERT_EQUAL_size_t(24, rig.imu().power_cycles());
}

void test_dc_sky_blockage_sends_heartbeats(void) {
    PowerRig rig(Build::DutyCycled, kFaultRunSleepNA, kFaultRunCapacityNaMs);
    run_fault(rig, FaultKind::SkyBlockage);
    TEST_ASSERT_EQUAL_size_t(24, heartbeats(rig));
}

void test_interrupt_sky_blockage_sends_no_heartbeats(void) {
    PowerRig rig(Build::Interrupt, kFaultRunSleepNA, kFaultRunCapacityNaMs);
    run_fault(rig, FaultKind::SkyBlockage);
    TEST_ASSERT_EQUAL_size_t(0, heartbeats(rig));
}

/// Why the interrupt build's silent downtime is below the 6600000 ms a
/// report-at-blockage-start reference predicts: its last valid record before
/// the blockage arrived 24.8 s after the blockage began.
void test_interrupt_sky_blockage_last_record_after_start(void) {
    PowerRig rig(Build::Interrupt, kFaultRunSleepNA, kFaultRunCapacityNaMs);
    run_fault(rig, FaultKind::SkyBlockage);
    const uint32_t blockage_end = kFaultAtMs + fault_case(FaultKind::SkyBlockage).duration_ms;
    uint32_t last_before_end = 0;
    for (size_t i = 0; i < rig.records().size(); ++i) {
        const GatewayRecord &r = rig.records()[i];
        if (r.gps_valid && r.arrival_ms < blockage_end) {
            last_before_end = r.arrival_ms;
        }
    }
    TEST_ASSERT_EQUAL_UINT32(7224810, last_before_end);
}

int main(int, char **) {
    UNITY_BEGIN();
    RUN_TEST(test_dc_lost_completion_costs_exactly_one_report);
    RUN_TEST(test_dc_hang_reboots_once_without_going_stale);
    RUN_TEST(test_dc_radio_wedge_recovers_after_three_timeouts);
    RUN_TEST(test_interrupt_lost_completion_loses_seconds);
    RUN_TEST(test_dc_imu_fault_is_applied);
    RUN_TEST(test_dc_sky_blockage_sends_heartbeats);
    RUN_TEST(test_interrupt_sky_blockage_sends_no_heartbeats);
    RUN_TEST(test_interrupt_sky_blockage_last_record_after_start);
    return UNITY_END();
}
