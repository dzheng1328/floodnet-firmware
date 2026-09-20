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

// 9600 baud, 8N1, is about 0.96 bytes per millisecond.
static const double kGpsByteRate = 0.96;
static const char kSentence[] =
    "$GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,*47\r\n";

// Two timing profiles, because they demonstrate two different things.
//
// FAST keeps every blocking call short enough that the 64-byte receive buffer
// never overruns, so sentences arrive intact. It proves the loop works.
//
// REALISTIC uses a 10 ms IMU read and 60 ms of LoRa airtime. Together those
// exceed the ~66 ms the buffer can hold at 9600 baud, so bytes are lost. It
// proves the loop's defect. Mixing the two into one profile would mean either
// no packets or no drops, and the milestone needs to show both.
static const uint32_t kFastImuMs = 2;
static const uint32_t kFastRadioMs = 5;
static const uint32_t kRealisticImuMs = 10;
static const uint32_t kRealisticRadioMs = 60;

void test_emits_a_decodable_packet(void) {
    SimClock clock;
    FakeGps gps(kSentence, kGpsByteRate);
    clock.add_observer(&gps);
    FakeImu imu(clock, kFastImuMs);
    FakeRadio radio(clock, kFastRadioMs);

    PollingSampler sampler(gps, imu, radio, clock, 0x0042, 3);
    for (int i = 0; i < 400; ++i) {
        sampler.step();
    }

    TEST_ASSERT_GREATER_THAN_UINT32(0, sampler.packets_sent());

    Packet decoded;
    TEST_ASSERT_TRUE(decode_packet(radio.last_payload(), radio.last_length(), &decoded));
    TEST_ASSERT_EQUAL_UINT16(0x0042, decoded.node_id);
    TEST_ASSERT_EQUAL_UINT8(3, decoded.ttl);
    TEST_ASSERT_TRUE(decoded.record.gps.valid);
    TEST_ASSERT_EQUAL_INT32(481173000, decoded.record.gps.lat_1e7);
}

void test_sequence_numbers_increment(void) {
    SimClock clock;
    FakeGps gps(kSentence, kGpsByteRate);
    clock.add_observer(&gps);
    FakeImu imu(clock, kFastImuMs);
    FakeRadio radio(clock, kFastRadioMs);

    PollingSampler sampler(gps, imu, radio, clock, 1, 3);
    for (int i = 0; i < 400; ++i) {
        sampler.step();
    }

    Packet decoded;
    TEST_ASSERT_TRUE(decode_packet(radio.last_payload(), radio.last_length(), &decoded));
    TEST_ASSERT_EQUAL_UINT32(sampler.packets_sent() - 1, decoded.seq);
}

void test_blocking_calls_lose_gps_bytes(void) {
    // This is the defect the interrupt-driven rewrite exists to remove.
    // Asserting it here means the later fix has something concrete to beat.
    SimClock clock;
    FakeGps gps(kSentence, kGpsByteRate);
    clock.add_observer(&gps);
    FakeImu imu(clock, kRealisticImuMs);
    FakeRadio radio(clock, kRealisticRadioMs);

    PollingSampler sampler(gps, imu, radio, clock, 1, 3);
    for (int i = 0; i < 400; ++i) {
        sampler.step();
    }

    TEST_ASSERT_GREATER_THAN_UINT16(0, sampler.diag().drops);
}

void test_fast_loop_loses_nothing(void) {
    // The mirror of the test above: the loss is a consequence of the stall,
    // not something inherent to polling, and this pins that down.
    SimClock clock;
    FakeGps gps(kSentence, kGpsByteRate);
    clock.add_observer(&gps);
    FakeImu imu(clock, kFastImuMs);
    FakeRadio radio(clock, kFastRadioMs);

    PollingSampler sampler(gps, imu, radio, clock, 1, 3);
    for (int i = 0; i < 400; ++i) {
        sampler.step();
    }

    TEST_ASSERT_EQUAL_UINT16(0, sampler.diag().drops);
}

void test_reports_drops_in_the_packet(void) {
    SimClock clock;
    FakeGps gps(kSentence, kGpsByteRate);
    clock.add_observer(&gps);
    FakeImu imu(clock, kRealisticImuMs);
    FakeRadio radio(clock, kRealisticRadioMs);

    PollingSampler sampler(gps, imu, radio, clock, 1, 3);
    for (int i = 0; i < 400; ++i) {
        sampler.step();
    }

    Packet decoded;
    TEST_ASSERT_TRUE(decode_packet(radio.last_payload(), radio.last_length(), &decoded));
    TEST_ASSERT_GREATER_THAN_UINT16(0, decoded.record.diag.drops);
}

void test_no_transmission_without_a_complete_sentence(void) {
    SimClock clock;
    FakeGps gps("garbage without a dollar sign", kGpsByteRate);
    clock.add_observer(&gps);
    FakeImu imu(clock, kFastImuMs);
    FakeRadio radio(clock, kFastRadioMs);

    PollingSampler sampler(gps, imu, radio, clock, 1, 3);
    for (int i = 0; i < 100; ++i) {
        sampler.step();
    }

    TEST_ASSERT_EQUAL_UINT32(0, sampler.packets_sent());
}

int main(int, char **) {
    UNITY_BEGIN();
    RUN_TEST(test_emits_a_decodable_packet);
    RUN_TEST(test_sequence_numbers_increment);
    RUN_TEST(test_blocking_calls_lose_gps_bytes);
    RUN_TEST(test_fast_loop_loses_nothing);
    RUN_TEST(test_reports_drops_in_the_packet);
    RUN_TEST(test_no_transmission_without_a_complete_sentence);
    return UNITY_END();
}
