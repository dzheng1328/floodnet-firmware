#include <unity.h>

#include <floodnet/downtime.hpp>

using namespace floodnet;

void setUp(void) {}
void tearDown(void) {}

static GatewayRecord rec(uint32_t arrival, bool valid, uint16_t boot = 1, uint16_t tx = 0) {
    GatewayRecord r;
    r.arrival_ms = arrival;
    r.gps_valid = valid;
    r.boot_count = boot;
    r.tx_timeouts = tx;
    return r;
}

static DowntimeReport run(const GatewayRecord *records, size_t count, uint32_t scenario_ms) {
    DowntimeInput in;
    in.records = records;
    in.count = count;
    in.scenario_ms = scenario_ms;
    return compute_downtime(in);
}

static uint32_t cause(const DowntimeReport &r, DownCause c) {
    return r.down_ms[static_cast<size_t>(c)];
}

void test_no_records_is_all_startup(void) {
    DowntimeReport r = run(nullptr, 0, 3600000);
    TEST_ASSERT_EQUAL_UINT32(3600000, cause(r, DownCause::Startup));
    TEST_ASSERT_EQUAL_UINT32(3600000, r.total_down_ms);
    TEST_ASSERT_TRUE(r.down_at_end);
}

void test_steady_reports_leave_only_startup(void) {
    GatewayRecord records[12];
    for (uint32_t k = 0; k < 12; ++k) {
        records[k] = rec(29000 + k * 300000, true);
    }
    DowntimeReport r = run(records, 12, 3600000);
    TEST_ASSERT_EQUAL_UINT32(29000, cause(r, DownCause::Startup));
    TEST_ASSERT_EQUAL_UINT32(29000, r.total_down_ms);
    TEST_ASSERT_FALSE(r.down_at_end);
}

void test_gap_longer_than_two_intervals_is_silent(void) {
    const GatewayRecord records[] = {rec(1000, true), rec(301000, true), rec(1201000, true)};
    DowntimeReport r = run(records, 3, 1300000);
    TEST_ASSERT_EQUAL_UINT32(300000, cause(r, DownCause::Silent));  // 901000 to 1201000
    TEST_ASSERT_EQUAL_UINT32(301000, r.total_down_ms);
}

void test_exactly_stale_after_is_not_down(void) {
    const GatewayRecord records[] = {rec(1000, true), rec(601000, true)};
    DowntimeReport r = run(records, 2, 700000);
    TEST_ASSERT_EQUAL_UINT32(1000, r.total_down_ms);  // startup only
}

void test_heartbeats_during_a_gap_attribute_to_no_gps(void) {
    const GatewayRecord records[] = {rec(1000, true), rec(301000, false), rec(601000, false),
                                     rec(901000, false), rec(1201000, true)};
    DowntimeReport r = run(records, 5, 1300000);
    TEST_ASSERT_EQUAL_UINT32(600000, cause(r, DownCause::NoGps));
}

void test_heartbeat_before_the_gap_does_not_claim_it(void) {
    // A heartbeat while still fresh, then silence: that later silence was not
    // explained by the heartbeat.
    const GatewayRecord records[] = {rec(1000, true), rec(301000, false), rec(1201000, true)};
    DowntimeReport r = run(records, 3, 1300000);
    TEST_ASSERT_EQUAL_UINT32(0, cause(r, DownCause::NoGps));
    TEST_ASSERT_EQUAL_UINT32(600000, cause(r, DownCause::Silent));
}

void test_boot_count_change_attributes_to_reboot(void) {
    const GatewayRecord records[] = {rec(1000, true, 1), rec(1001000, true, 2)};
    DowntimeReport r = run(records, 2, 1100000);
    TEST_ASSERT_EQUAL_UINT32(400000, cause(r, DownCause::Reboot));
}

void test_tx_timeouts_increase_attributes_to_radio(void) {
    const GatewayRecord records[] = {rec(1000, true, 1, 0), rec(1001000, true, 1, 3)};
    DowntimeReport r = run(records, 2, 1100000);
    TEST_ASSERT_EQUAL_UINT32(400000, cause(r, DownCause::Radio));
}

void test_reboot_takes_precedence_over_radio(void) {
    const GatewayRecord records[] = {rec(1000, true, 1, 0), rec(1001000, true, 2, 5)};
    DowntimeReport r = run(records, 2, 1100000);
    TEST_ASSERT_EQUAL_UINT32(400000, cause(r, DownCause::Reboot));
    TEST_ASSERT_EQUAL_UINT32(0, cause(r, DownCause::Radio));
}

void test_battery_death_splits_the_interval(void) {
    const GatewayRecord records[] = {rec(1000, true)};
    DowntimeInput in;
    in.records = records;
    in.count = 1;
    in.scenario_ms = 1000000;
    in.died = true;
    in.died_at_ms = 700000;
    DowntimeReport r = compute_downtime(in);
    TEST_ASSERT_EQUAL_UINT32(99000, cause(r, DownCause::Silent));   // 601000 to 700000
    TEST_ASSERT_EQUAL_UINT32(300000, cause(r, DownCause::Battery));
    TEST_ASSERT_TRUE(r.down_at_end);
}

void test_death_before_any_record_is_battery_after_startup(void) {
    DowntimeInput in;
    in.scenario_ms = 10000;
    in.died = true;
    in.died_at_ms = 5000;
    DowntimeReport r = compute_downtime(in);
    TEST_ASSERT_EQUAL_UINT32(5000, cause(r, DownCause::Startup));
    TEST_ASSERT_EQUAL_UINT32(5000, cause(r, DownCause::Battery));
}

void test_records_after_the_scenario_are_ignored(void) {
    const GatewayRecord records[] = {rec(1000, true), rec(2000000, true)};
    DowntimeReport r = run(records, 2, 1000000);
    TEST_ASSERT_EQUAL_UINT32(399000, cause(r, DownCause::Silent));
}

void test_cause_names_are_stable(void) {
    TEST_ASSERT_EQUAL_STRING("startup", down_cause_name(DownCause::Startup));
    TEST_ASSERT_EQUAL_STRING("battery", down_cause_name(DownCause::Battery));
    TEST_ASSERT_EQUAL_STRING("no_gps", down_cause_name(DownCause::NoGps));
    TEST_ASSERT_EQUAL_STRING("radio", down_cause_name(DownCause::Radio));
    TEST_ASSERT_EQUAL_STRING("reboot", down_cause_name(DownCause::Reboot));
    TEST_ASSERT_EQUAL_STRING("silent", down_cause_name(DownCause::Silent));
}

int main(int, char **) {
    UNITY_BEGIN();
    RUN_TEST(test_no_records_is_all_startup);
    RUN_TEST(test_steady_reports_leave_only_startup);
    RUN_TEST(test_gap_longer_than_two_intervals_is_silent);
    RUN_TEST(test_exactly_stale_after_is_not_down);
    RUN_TEST(test_heartbeats_during_a_gap_attribute_to_no_gps);
    RUN_TEST(test_heartbeat_before_the_gap_does_not_claim_it);
    RUN_TEST(test_boot_count_change_attributes_to_reboot);
    RUN_TEST(test_tx_timeouts_increase_attributes_to_radio);
    RUN_TEST(test_reboot_takes_precedence_over_radio);
    RUN_TEST(test_battery_death_splits_the_interval);
    RUN_TEST(test_death_before_any_record_is_battery_after_startup);
    RUN_TEST(test_records_after_the_scenario_are_ignored);
    RUN_TEST(test_cause_names_are_stable);
    return UNITY_END();
}
