#include <stdio.h>

#include <unity.h>

#include <floodnet/packet.hpp>

#include "../support/fake_gps.hpp"
#include "../support/fake_imu.hpp"
#include "../support/fake_radio.hpp"
#include "../support/sim_clock.hpp"
#include "sampler_interrupt.hpp"
#include "sampler_polling.hpp"

using namespace floodnet;

void setUp(void) {}
void tearDown(void) {}

// Mirrors the profile constants in test/test_polling/ and test/test_interrupt/.
// Kept as a separate copy rather than a shared header: this file exists to
// produce the numbers published in the README, and duplicating a few constants
// is a far smaller risk than coupling the benchmark's build to a test suite's.
static const double kGpsByteRate = 0.96;
static const char kSentence[] =
    "$GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,*47\r\n";

static const uint32_t kImuReadMs = 10;
static const uint32_t kRadioSf12Ms = 3023;
static const uint32_t kRadioSf7Ms = 92;
static const uint32_t kRadioControlMs = 5;
static const uint32_t kImuControlMs = 2;

static const size_t kShallowBuffer = 64;    // milestone 1
static const size_t kDeepBuffer = 4096;     // milestone 2

// Fixed simulated duration rather than a fixed step count, so every row is
// directly comparable on a per-second basis.
static const uint32_t kBenchDurationMs = 60000;

struct BenchResult {
    uint32_t packets;
    uint16_t overflows;
    uint16_t tx_drops;
    double packets_per_sec;
    double overflows_per_sec;
};

static void report(const char *strategy, size_t buffer_bytes, const char *profile,
                   uint32_t sim_ms, BenchResult *r, size_t txq_high_water) {
    const double sim_sec = static_cast<double>(sim_ms) / 1000.0;
    r->packets_per_sec = static_cast<double>(r->packets) / sim_sec;
    r->overflows_per_sec = static_cast<double>(r->overflows) / sim_sec;

    printf("BENCH,%s,%zu,%s,%u,%u,%u,%u,%.2f,%.2f,%zu\n", strategy, buffer_bytes, profile,
           sim_ms, r->packets, r->overflows, r->tx_drops, r->packets_per_sec,
           r->overflows_per_sec, txq_high_water);
}

static BenchResult run_polling(const char *profile, size_t buffer_bytes, uint32_t imu_ms,
                               uint32_t radio_ms) {
    SimClock clock;
    FakeGps gps(kSentence, kGpsByteRate, buffer_bytes);
    // ONLY the GPS is registered, exactly as milestone 1 registered it. The
    // polling sampler charges its own time through blocking HAL calls, and
    // adding observers it never had would change what this row measures.
    clock.add_observer(&gps);
    FakeImu imu(clock, imu_ms);
    FakeRadio radio(clock, radio_ms);

    PollingSampler sampler(gps, imu, radio, clock, 1, 3);
    while (clock.now_ms() < kBenchDurationMs) {
        sampler.step();
    }

    BenchResult result;
    result.packets = sampler.packets_sent();
    result.overflows = sampler.diag().drops;
    // Not measurable on this path, rather than measured as zero: the polling
    // sampler has no outbound queue, and it discards all but the last fix of
    // each drain without counting them. See README.md, "The 2x2".
    result.tx_drops = 0;
    report("polling", buffer_bytes, profile, clock.now_ms(), &result, 0);
    return result;
}

static BenchResult run_interrupt(const char *profile, size_t buffer_bytes, uint32_t imu_ms,
                                 uint32_t radio_ms) {
    SimClock clock;
    FakeGps gps(kSentence, kGpsByteRate, buffer_bytes);
    FakeImu imu(clock, imu_ms);
    FakeRadio radio(clock, radio_ms);
    // All three registered: a loop that does not block depends on the clock
    // to advance its sources.
    clock.add_observer(&gps);
    clock.add_observer(&imu);
    clock.add_observer(&radio);

    InterruptSampler sampler(gps, imu, radio, clock, 1, 3);
    while (clock.now_ms() < kBenchDurationMs) {
        sampler.step();
    }

    BenchResult result;
    result.packets = sampler.packets_sent();
    result.overflows = sampler.diag().drops;
    result.tx_drops = sampler.tx_queue_drops();
    report("interrupt", buffer_bytes, profile, clock.now_ms(), &result,
           sampler.tx_queue_high_water());
    return result;
}

void test_baseline_row_still_reproduces_milestone_one(void) {
    BenchResult control = run_polling("CONTROL", kShallowBuffer, kImuControlMs, kRadioControlMs);
    BenchResult sf7 = run_polling("SF7", kShallowBuffer, kImuReadMs, kRadioSf7Ms);
    BenchResult sf12 = run_polling("SF12", kShallowBuffer, kImuReadMs, kRadioSf12Ms);

    // The milestone 1 invariants, unchanged. If these move, milestone 2 has
    // disturbed the thing it is measured against and the comparison is void.
    TEST_ASSERT_EQUAL_UINT16(0, control.overflows);
    TEST_ASSERT_GREATER_THAN_UINT16(0, sf7.overflows);
    TEST_ASSERT_GREATER_THAN_UINT16(0, sf12.overflows);
    TEST_ASSERT_LESS_THAN_UINT16(0xFFFF, control.overflows);
    TEST_ASSERT_LESS_THAN_UINT16(0xFFFF, sf7.overflows);
    TEST_ASSERT_LESS_THAN_UINT16(0xFFFF, sf12.overflows);
    TEST_ASSERT_TRUE(sf12.overflows_per_sec > sf7.overflows_per_sec);
    TEST_ASSERT_TRUE(control.packets_per_sec > sf12.packets_per_sec);
}

void test_deep_buffer_alone_eliminates_gps_byte_loss(void) {
    // This test originally asserted the opposite, and the plan was wrong by
    // its own arithmetic. The buffer was sized at 4096 bytes specifically to
    // cover a full SF12 transmit -- 3023 ms at 0.96 B/ms is 2902 bytes -- and
    // then this test claimed that same buffer would overflow.
    //
    // It does not. For GPS byte loss alone, a deeper receive buffer is
    // sufficient and the non-blocking loop is not required. That is the
    // milestone's least comfortable result, and it is asserted here rather
    // than left out.
    //
    // All three profiles run so the matrix is complete: without the SF7 row in
    // particular, the move from the 5.99 packets/sec baseline could not be
    // attributed between the deeper buffer and the non-blocking loop, which is
    // the entire reason this benchmark is a 2x2. That row measures 9.79, above
    // the interrupt build's 9.07: at SF7 the deeper buffer alone is faster.
    BenchResult control = run_polling("CONTROL", kDeepBuffer, kImuControlMs, kRadioControlMs);
    BenchResult sf7 = run_polling("SF7", kDeepBuffer, kImuReadMs, kRadioSf7Ms);
    BenchResult sf12 = run_polling("SF12", kDeepBuffer, kImuReadMs, kRadioSf12Ms);

    TEST_ASSERT_EQUAL_UINT16(0, control.overflows);
    TEST_ASSERT_EQUAL_UINT16(0, sf7.overflows);
    TEST_ASSERT_EQUAL_UINT16(0, sf12.overflows);
}

void test_non_blocking_loop_alone_also_eliminates_byte_loss(void) {
    BenchResult control = run_interrupt("CONTROL", kShallowBuffer, kImuControlMs, kRadioControlMs);
    BenchResult sf7 = run_interrupt("SF7", kShallowBuffer, kImuReadMs, kRadioSf7Ms);
    BenchResult sf12 = run_interrupt("SF12", kShallowBuffer, kImuReadMs, kRadioSf12Ms);

    // This row was expected to overflow the 64-byte buffer and does not: a
    // loop that drains on every pass keeps up even at the shallow depth. The
    // test is named for that result, so it asserts it.
    TEST_ASSERT_EQUAL_UINT16(0, control.overflows);
    TEST_ASSERT_EQUAL_UINT16(0, sf7.overflows);
    TEST_ASSERT_EQUAL_UINT16(0, sf12.overflows);
}

void test_milestone_two_configuration_loses_no_gps_bytes(void) {
    BenchResult control = run_interrupt("CONTROL", kDeepBuffer, kImuControlMs, kRadioControlMs);
    BenchResult sf7 = run_interrupt("SF7", kDeepBuffer, kImuReadMs, kRadioSf7Ms);
    BenchResult sf12 = run_interrupt("SF12", kDeepBuffer, kImuReadMs, kRadioSf12Ms);

    // The milestone's core result.
    TEST_ASSERT_EQUAL_UINT16(0, control.overflows);
    TEST_ASSERT_EQUAL_UINT16(0, sf7.overflows);
    TEST_ASSERT_EQUAL_UINT16(0, sf12.overflows);

    // And its honest cost: at SF12 the radio still cannot keep up, so whole
    // packets are dropped. Zero here would mean the node had somehow sent
    // everything, which the airtime makes impossible.
    TEST_ASSERT_GREATER_THAN_UINT16(0, sf12.tx_drops);
}

void test_control_profile_is_a_null_control(void) {
    // CONTROL is already limited by the GPS sentence rate, not the radio, so
    // a non-blocking loop has nothing to win there. A large gain would mean
    // the simulation is flattering the new sampler, which is a measurement
    // bug and not a result. Allow 10% for timing granularity.
    BenchResult polled = run_polling("CONTROL", kShallowBuffer, kImuControlMs, kRadioControlMs);
    BenchResult interrupted =
        run_interrupt("CONTROL", kDeepBuffer, kImuControlMs, kRadioControlMs);

    TEST_ASSERT_TRUE(interrupted.packets_per_sec < polled.packets_per_sec * 1.10);
}

void test_non_blocking_loop_beats_the_milestone_one_baseline_at_sf7(void) {
    // A regression guard, not a throughput claim. It compares against the
    // milestone 1 baseline (polling, 64-byte buffer, 5.99 packets/sec), so it
    // changes two variables at once and cannot attribute the gain. The
    // single-variable comparison goes the other way: polling with the 4096-byte
    // buffer measures 9.79 at SF7, above this build's 9.07. See README.md,
    // "Why the interrupt build is slower at SF7".
    BenchResult polled = run_polling("SF7", kShallowBuffer, kImuReadMs, kRadioSf7Ms);
    BenchResult interrupted = run_interrupt("SF7", kDeepBuffer, kImuReadMs, kRadioSf7Ms);

    TEST_ASSERT_TRUE(interrupted.packets_per_sec > polled.packets_per_sec);
}

int main(int, char **) {
    UNITY_BEGIN();
    RUN_TEST(test_baseline_row_still_reproduces_milestone_one);
    RUN_TEST(test_deep_buffer_alone_eliminates_gps_byte_loss);
    RUN_TEST(test_non_blocking_loop_alone_also_eliminates_byte_loss);
    RUN_TEST(test_milestone_two_configuration_loses_no_gps_bytes);
    RUN_TEST(test_control_profile_is_a_null_control);
    RUN_TEST(test_non_blocking_loop_beats_the_milestone_one_baseline_at_sf7);
    return UNITY_END();
}
