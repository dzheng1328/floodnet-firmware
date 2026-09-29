#include <unity.h>

#include <floodnet/packet.hpp>

#include "../support/power_rig.hpp"

using namespace floodnet;

void setUp(void) {}
void tearDown(void) {}

static const uint32_t kSlotMs = 300000;

static PowerRig *make_rig() {
    return new PowerRig(Build::DutyCycled, TEENSY_SLEEP_LOW_NA, 0);
}

void test_first_report_follows_a_cold_start(void) {
    std::unique_ptr<PowerRig> rig(make_rig());
    rig->run_until(60000);

    TEST_ASSERT_EQUAL_UINT(1, rig->deliveries().size());
    const Packet &p = rig->deliveries()[0];
    TEST_ASSERT_TRUE(p.record.gps.valid);
    TEST_ASSERT_EQUAL_UINT16(1, p.boot_count);
    TEST_ASSERT_EQUAL_UINT32(0, p.seq);
    // 26 s cold start, one no-fix and one fix sentence, then 3023 ms airtime.
    TEST_ASSERT_GREATER_OR_EQUAL_UINT32(29000, rig->records()[0].arrival_ms);
    TEST_ASSERT_LESS_OR_EQUAL_UINT32(29500, rig->records()[0].arrival_ms);
}

void test_one_report_per_slot_on_an_anchored_schedule(void) {
    std::unique_ptr<PowerRig> rig(make_rig());
    rig->run_until(12 * kSlotMs);

    TEST_ASSERT_EQUAL_UINT(12, rig->deliveries().size());
    for (uint32_t k = 0; k < 12; ++k) {
        TEST_ASSERT_EQUAL_UINT32(k, rig->deliveries()[k].seq);
    }
    for (uint32_t k = 1; k < 12; ++k) {
        // Hot start (1 s) plus airtime (3023 ms), measured from the slot.
        const uint32_t late = rig->records()[k].arrival_ms - k * kSlotMs;
        TEST_ASSERT_GREATER_OR_EQUAL_UINT32(4000, late);
        TEST_ASSERT_LESS_OR_EQUAL_UINT32(4500, late);
    }
}

void test_watchdog_is_kicked_well_inside_its_timeout(void) {
    std::unique_ptr<PowerRig> rig(make_rig());
    rig->run_until(12 * kSlotMs);
    TEST_ASSERT_EQUAL_UINT(0, rig->reboots());
    TEST_ASSERT_LESS_OR_EQUAL_UINT32(61000, rig->watchdog().max_since_kick_ms());
}

void test_sky_blockage_sends_a_heartbeat_at_the_acquire_timeout(void) {
    std::unique_ptr<PowerRig> rig(make_rig());
    rig->gps().set_sky_blocked(true);
    rig->run_until(90000);

    TEST_ASSERT_EQUAL_UINT(1, rig->deliveries().size());
    TEST_ASSERT_FALSE(rig->deliveries()[0].record.gps.valid);
    TEST_ASSERT_GREATER_OR_EQUAL_UINT32(63000, rig->records()[0].arrival_ms);
    TEST_ASSERT_LESS_OR_EQUAL_UINT32(63500, rig->records()[0].arrival_ms);
}

void test_stale_sentence_from_before_sleep_is_not_reported(void) {
    std::unique_ptr<PowerRig> rig(make_rig());
    rig->run_until(100000);  // first report sent; asleep
    rig->gps().inject(kRigFixSentence);  // a whole fix left in the buffer
    rig->gps().set_sky_blocked(true);
    rig->run_until(kSlotMs + 90000);

    TEST_ASSERT_EQUAL_UINT(2, rig->deliveries().size());
    TEST_ASSERT_FALSE(rig->deliveries()[1].record.gps.valid);
}

void test_wedged_radio_is_power_cycled_and_recovers(void) {
    std::unique_ptr<PowerRig> rig(make_rig());
    rig->run_until(100000);
    rig->radio().set_wedged(true);
    rig->run_until(4 * kSlotMs + 60000);  // slots 1-3 fail; slot 4 succeeds

    TEST_ASSERT_EQUAL_UINT(1, rig->radio().power_cycles());
    TEST_ASSERT_EQUAL_UINT(2, rig->deliveries().size());
    TEST_ASSERT_EQUAL_UINT16(3, rig->deliveries()[1].tx_timeouts);
    TEST_ASSERT_GREATER_OR_EQUAL_UINT32(4 * kSlotMs, rig->records()[1].arrival_ms);
}

void test_refused_transmit_leaves_transmit_after_its_timeout(void) {
    std::unique_ptr<PowerRig> rig(make_rig());
    rig->run_until(100000);
    rig->radio().set_refusing(true);
    rig->run_until(kSlotMs + 1500);
    TEST_ASSERT_TRUE(rig->node().state() == NodeState::Transmit);
    rig->run_until(kSlotMs + 12000);
    TEST_ASSERT_TRUE(rig->node().state() == NodeState::Sleep);
    TEST_ASSERT_EQUAL_UINT(1, rig->deliveries().size());
}

void test_failing_imu_is_power_cycled_after_three_wakes(void) {
    std::unique_ptr<PowerRig> rig(make_rig());
    rig->run_until(100000);
    rig->imu().set_failing(true);
    rig->run_until(3 * kSlotMs + 60000);

    TEST_ASSERT_EQUAL_UINT(1, rig->imu().power_cycles());
    TEST_ASSERT_EQUAL_UINT(4, rig->deliveries().size());
    for (size_t k = 1; k < 4; ++k) {
        TEST_ASSERT_TRUE(rig->deliveries()[k].record.gps.valid);  // still not down
        TEST_ASSERT_FALSE(rig->deliveries()[k].record.imu.valid);
    }
}

void test_hang_is_reset_by_the_watchdog_and_reports_resume(void) {
    std::unique_ptr<PowerRig> rig(make_rig());
    rig->run_until(100000);
    rig->imu().hang_next_read(&rig->watchdog());
    rig->run_until(kSlotMs + 120000);

    TEST_ASSERT_EQUAL_UINT(1, rig->reboots());
    TEST_ASSERT_EQUAL_UINT16(2, rig->node().boot_count());
    TEST_ASSERT_EQUAL_UINT(2, rig->deliveries().size());
    // seq restarted at 0 after the reset; the gateway kept it anyway.
    TEST_ASSERT_EQUAL_UINT32(0, rig->deliveries()[1].seq);
    TEST_ASSERT_EQUAL_UINT16(2, rig->deliveries()[1].boot_count);
}

void test_interrupt_build_streams_and_never_sleeps(void) {
    PowerRig rig(Build::Interrupt, TEENSY_SLEEP_LOW_NA, 0);
    rig.run_until(60000);
    TEST_ASSERT_EQUAL_UINT(0, rig.power().sleeps());
    TEST_ASSERT_GREATER_THAN_UINT(5, rig.deliveries().size());
    TEST_ASSERT_EQUAL_UINT16(0, rig.deliveries()[0].boot_count);  // v0x01
}

int main(int, char **) {
    UNITY_BEGIN();
    RUN_TEST(test_first_report_follows_a_cold_start);
    RUN_TEST(test_one_report_per_slot_on_an_anchored_schedule);
    RUN_TEST(test_watchdog_is_kicked_well_inside_its_timeout);
    RUN_TEST(test_sky_blockage_sends_a_heartbeat_at_the_acquire_timeout);
    RUN_TEST(test_stale_sentence_from_before_sleep_is_not_reported);
    RUN_TEST(test_wedged_radio_is_power_cycled_and_recovers);
    RUN_TEST(test_refused_transmit_leaves_transmit_after_its_timeout);
    RUN_TEST(test_failing_imu_is_power_cycled_after_three_wakes);
    RUN_TEST(test_hang_is_reset_by_the_watchdog_and_reports_resume);
    RUN_TEST(test_interrupt_build_streams_and_never_sleeps);
    return UNITY_END();
}
