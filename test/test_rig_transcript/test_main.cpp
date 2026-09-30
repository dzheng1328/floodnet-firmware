#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include <set>
#include <string>
#include <vector>

#include <unity.h>

#include <floodnet/hal/tx_listener.hpp>

#include "../support/noisy_channel.hpp"
#include "../support/power_experiments.hpp"
#include "../support/power_rig.hpp"

using namespace floodnet;

void setUp(void) {}
void tearDown(void) {}

struct Recorder : public ITxListener {
    size_t completed = 0;
    size_t abandoned = 0;
    std::set<uint16_t> boots;
    void on_tx_end(uint16_t, uint16_t boot_count, uint32_t, uint32_t, bool done) override {
        (done ? completed : abandoned) += 1;
        boots.insert(boot_count);
    }
};

void test_run_until_quiet_ends_with_nothing_in_flight(void) {
    PowerRig rig(Build::Interrupt, kFaultRunSleepNA, kFaultRunCapacityNaMs);
    rig.run_until_quiet(600000UL);
    TEST_ASSERT_FALSE(rig.tx_in_flight());
    TEST_ASSERT_TRUE(rig.clock().now_ms() >= 600000UL);
}

void test_completed_listener_events_equal_radio_completions(void) {
    PowerRig rig(Build::Interrupt, kFaultRunSleepNA, kFaultRunCapacityNaMs);
    Recorder rec;
    rig.set_tx_listener(&rec);
    rig.run_until_quiet(600000UL);
    TEST_ASSERT_TRUE(rec.completed > 0);
    TEST_ASSERT_EQUAL_UINT32(rig.radio().sent_count(), rec.completed);
    TEST_ASSERT_EQUAL_UINT32(rig.tx_timeouts_total(), rec.abandoned);
}

void test_listener_survives_a_reboot(void) {
    PowerRig rig(Build::DutyCycled, kFaultRunSleepNA, kFaultRunCapacityNaMs);
    Recorder rec;
    rig.set_tx_listener(&rec);
    rig.add_fault(FaultKind::Hang, kFaultAtMs, 0);
    rig.run_until_quiet(kFaultRunMs);
    TEST_ASSERT_EQUAL_UINT32(1, rig.reboots());
    TEST_ASSERT_EQUAL_UINT32(2, rec.boots.size());
    // The IMU and the radio are never powered together, so the hang cannot
    // strike with a frame in flight, and no completion goes unreported.
    TEST_ASSERT_EQUAL_UINT32(0, rig.resets_mid_transmit());
    TEST_ASSERT_EQUAL_UINT32(rig.radio().sent_count(), rec.completed);
}

void test_lost_completion_is_a_timeout(void) {
    PowerRig rig(Build::DutyCycled, kFaultRunSleepNA, kFaultRunCapacityNaMs);
    Recorder rec;
    rig.set_tx_listener(&rec);
    rig.add_fault(FaultKind::LostCompletion, kFaultAtMs, 0);
    rig.run_until_quiet(kFaultRunMs);
    TEST_ASSERT_EQUAL_UINT32(1, rig.tx_timeouts_total());
    TEST_ASSERT_EQUAL_UINT32(1, rec.abandoned);
}

void test_gateway_log_has_a_rec_line_per_delivery(void) {
    PowerRig rig(Build::DutyCycled, kFaultRunSleepNA, kFaultRunCapacityNaMs);
    std::vector<std::string> lines;
    rig.set_gateway_log([&lines](uint32_t, const char *line) { lines.push_back(line); });
    rig.run_until_quiet(6UL * 3600000UL);
    TEST_ASSERT_TRUE(rig.deliveries().size() > 0);
    TEST_ASSERT_EQUAL_UINT32(rig.deliveries().size(), lines.size());
    for (size_t i = 0; i < lines.size(); ++i) {
        TEST_ASSERT_EQUAL_INT(0, strncmp(lines[i].c_str(), "REC,", 4));
    }
}

void test_every_corrupted_frame_is_a_decode_error_line(void) {
    NoisyChannel channel(1.0, kChannelSeed);
    PowerRig rig(Build::DutyCycled, kFaultRunSleepNA, kFaultRunCapacityNaMs);
    rig.set_channel(&channel);
    std::vector<std::string> lines;
    rig.set_gateway_log([&lines](uint32_t, const char *line) { lines.push_back(line); });
    rig.run_until_quiet(2UL * 3600000UL);
    TEST_ASSERT_TRUE(rig.radio().sent_count() > 0);
    TEST_ASSERT_EQUAL_UINT32(rig.radio().sent_count(), rig.decode_failures());
    TEST_ASSERT_EQUAL_UINT32(rig.decode_failures(), lines.size());
    TEST_ASSERT_EQUAL_STRING("ERR,decode,1", lines[0].c_str());
}

int main(int, char **) {
    UNITY_BEGIN();
    RUN_TEST(test_run_until_quiet_ends_with_nothing_in_flight);
    RUN_TEST(test_completed_listener_events_equal_radio_completions);
    RUN_TEST(test_listener_survives_a_reboot);
    RUN_TEST(test_lost_completion_is_a_timeout);
    RUN_TEST(test_gateway_log_has_a_rec_line_per_delivery);
    RUN_TEST(test_every_corrupted_frame_is_a_decode_error_line);
    return UNITY_END();
}
