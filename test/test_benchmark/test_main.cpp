#include <stdio.h>

#include <unity.h>

#include <floodnet/packet.hpp>

#include "../support/fake_gps.hpp"
#include "../support/fake_imu.hpp"
#include "../support/fake_radio.hpp"
#include "../support/sim_clock.hpp"
#include "sampler_polling.hpp"

using namespace floodnet;

void setUp(void) {}
void tearDown(void) {}

// Mirrors the profile constants in test/test_polling/test_main.cpp. Kept as a
// separate copy rather than a shared header: this file exists to produce the
// numbers published in the README, and duplicating three constants is a far
// smaller risk than coupling the benchmark's build to the correctness suite's.
static const double kGpsByteRate = 0.96;
static const char kSentence[] =
    "$GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,*47\r\n";

static const uint32_t kImuReadMs = 10;
static const uint32_t kRadioSf12Ms = 3023;
static const uint32_t kRadioSf7Ms = 92;
static const uint32_t kRadioControlMs = 5;
static const uint32_t kImuControlMs = 2;

// Run each profile for a fixed simulated duration, not a fixed step count, so
// the three are directly comparable on a per-second basis.
static const uint32_t kBenchDurationMs = 60000;

struct BenchResult {
    uint32_t sim_ms;
    uint32_t packets;
    uint16_t overflows;
    double packets_per_sec;
    double overflows_per_sec;
};

static BenchResult run_profile(const char *name, uint32_t imu_ms, uint32_t radio_ms) {
    SimClock clock;
    FakeGps gps(kSentence, kGpsByteRate);
    clock.add_observer(&gps);
    FakeImu imu(clock, imu_ms);
    FakeRadio radio(clock, radio_ms);

    PollingSampler sampler(gps, imu, radio, clock, 1, 3);
    while (clock.now_ms() < kBenchDurationMs) {
        sampler.step();
    }

    BenchResult result;
    result.sim_ms = clock.now_ms();
    result.packets = sampler.packets_sent();
    result.overflows = sampler.diag().drops;
    const double sim_sec = static_cast<double>(result.sim_ms) / 1000.0;
    result.packets_per_sec = static_cast<double>(result.packets) / sim_sec;
    result.overflows_per_sec = static_cast<double>(result.overflows) / sim_sec;

    printf("BENCH,%s,%u,%u,%u,%.2f,%.2f\n", name, result.sim_ms, result.packets,
           result.overflows, result.packets_per_sec, result.overflows_per_sec);

    return result;
}

void test_benchmark_profiles_are_ordered_correctly(void) {
    BenchResult control = run_profile("CONTROL", kImuControlMs, kRadioControlMs);
    BenchResult sf7 = run_profile("SF7", kImuReadMs, kRadioSf7Ms);
    BenchResult sf12 = run_profile("SF12", kImuReadMs, kRadioSf12Ms);

    // Ordering assertions are the real invariants. Exact packet/overflow
    // counts on a timing model would be brittle and break on any legitimate
    // tuning, so they are deliberately not asserted here.
    TEST_ASSERT_EQUAL_UINT16(0, control.overflows);
    TEST_ASSERT_GREATER_THAN_UINT16(0, sf7.overflows);
    TEST_ASSERT_GREATER_THAN_UINT16(0, sf12.overflows);

    // FakeGps saturates its counter at 0xFFFF. If this ever trips, the
    // benchmark duration has outgrown the counter and the published
    // overflow figures are clipped rather than measured.
    TEST_ASSERT_LESS_THAN_UINT16(0xFFFF, control.overflows);
    TEST_ASSERT_LESS_THAN_UINT16(0xFFFF, sf7.overflows);
    TEST_ASSERT_LESS_THAN_UINT16(0xFFFF, sf12.overflows);

    // Not TEST_ASSERT_GREATER_THAN: Unity expands that macro to an int
    // comparison (UNITY_TEST_ASSERT_GREATER_THAN_INT), which truncates these
    // double rates before comparing. That happens to pass today only because
    // the compared magnitudes straddle integer boundaries; a retune that put
    // two rates in the same integer bucket would pass here while failing to
    // hold the real, sub-1 precision ordering. Compare the doubles directly.
    TEST_ASSERT_TRUE(sf12.overflows_per_sec > sf7.overflows_per_sec);
    TEST_ASSERT_TRUE(control.packets_per_sec > sf12.packets_per_sec);
}

int main(int, char **) {
    UNITY_BEGIN();
    RUN_TEST(test_benchmark_profiles_are_ordered_correctly);
    return UNITY_END();
}
