#include <stddef.h>
#include <stdint.h>

#include <unity.h>

#include "../support/noisy_channel.hpp"
#include "../support/power_experiments.hpp"
#include "../support/power_rig.hpp"

using namespace floodnet;

void setUp(void) {}
void tearDown(void) {}

static const uint32_t kSixHoursMs = 6UL * 3600000UL;

static bool same_records(const PowerRig &a, const PowerRig &b) {
    const std::vector<GatewayRecord> &ra = a.records();
    const std::vector<GatewayRecord> &rb = b.records();
    if (ra.size() != rb.size()) {
        return false;
    }
    for (size_t i = 0; i < ra.size(); ++i) {
        if (ra[i].arrival_ms != rb[i].arrival_ms || ra[i].gps_valid != rb[i].gps_valid ||
            ra[i].boot_count != rb[i].boot_count || ra[i].tx_timeouts != rb[i].tx_timeouts) {
            return false;
        }
    }
    return true;
}

static void assert_zero_p_channel_is_transparent(Build build) {
    PowerRig plain(build, kFaultRunSleepNA, kFaultRunCapacityNaMs);
    plain.run_until(kSixHoursMs);
    NoisyChannel channel(0.0, kChannelSeed);
    PowerRig noisy(build, kFaultRunSleepNA, kFaultRunCapacityNaMs);
    noisy.set_channel(&channel);
    noisy.run_until(kSixHoursMs);
    TEST_ASSERT_TRUE(plain.records().size() > 0);
    TEST_ASSERT_TRUE(same_records(plain, noisy));
}

void test_zero_p_channel_is_transparent_interrupt(void) {
    assert_zero_p_channel_is_transparent(Build::Interrupt);
}

void test_zero_p_channel_is_transparent_duty_cycled(void) {
    assert_zero_p_channel_is_transparent(Build::DutyCycled);
}

void test_p_one_channel_delivers_nothing(void) {
    NoisyChannel channel(1.0, kChannelSeed);
    PowerRig rig(Build::DutyCycled, kFaultRunSleepNA, kFaultRunCapacityNaMs);
    rig.set_channel(&channel);
    rig.run_until(2UL * 3600000UL);
    TEST_ASSERT_TRUE(rig.radio().sent_count() > 0);
    TEST_ASSERT_EQUAL_UINT32(0, rig.deliveries().size());
}

void test_interrupt_build_frames_are_corrupted_by_the_channel(void) {
    NoisyChannel channel(1e-2, kChannelSeed);
    PowerRig rig(Build::Interrupt, kFaultRunSleepNA, kFaultRunCapacityNaMs);
    rig.set_channel(&channel);
    rig.run_until(2UL * 3600000UL);
    TEST_ASSERT_TRUE(rig.radio().sent_count() > 0);
    TEST_ASSERT_TRUE(rig.deliveries().size() < rig.radio().sent_count());
}

void test_records_queued_counts_every_queued_record(void) {
    PowerRig rig(Build::Interrupt, kFaultRunSleepNA, kFaultRunCapacityNaMs);
    rig.run_until(3600000UL);
    TEST_ASSERT_TRUE(rig.deliveries().size() > 0);
    TEST_ASSERT_TRUE(rig.records_queued() >= rig.deliveries().size());
}

void test_records_queued_survives_a_reboot(void) {
    PowerRig rig(Build::DutyCycled, kFaultRunSleepNA, kFaultRunCapacityNaMs);
    run_experiment_2_fault(rig, fault_case(FaultKind::Hang));
    TEST_ASSERT_EQUAL_UINT32(1, rig.reboots());
    TEST_ASSERT_TRUE(rig.records_queued() > rig.node().sampler().next_seq());
    TEST_ASSERT_TRUE(rig.records_queued() >= rig.deliveries().size());
}

int main(int, char **) {
    UNITY_BEGIN();
    RUN_TEST(test_zero_p_channel_is_transparent_interrupt);
    RUN_TEST(test_zero_p_channel_is_transparent_duty_cycled);
    RUN_TEST(test_p_one_channel_delivers_nothing);
    RUN_TEST(test_interrupt_build_frames_are_corrupted_by_the_channel);
    RUN_TEST(test_records_queued_counts_every_queued_record);
    RUN_TEST(test_records_queued_survives_a_reboot);
    return UNITY_END();
}
