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

// Three radio timing profiles. Airtime figures are computed from the LoRa
// formula, not guessed: Tsym = 2^SF / BW, preamble = (8 + 4.25) * Tsym, and
// payload symbols = 8 + ceil((8*PL - 4*SF + 28 + 16) / (4*(SF - 2*DE))) * (CR + 4)
// for a 45-byte explicit-header packet.
//
// SF12 is what src/hal/teensy_radio.hpp actually programs (Bw125Cr48Sf4096).
// SF7 is the fastest practical LoRa setting, included to show that the
// superloop cannot keep up at ANY realizable configuration.
// CONTROL is synthetic. No radio is this fast. It exists only to demonstrate
// that the loss tracks stall duration rather than being inherent to parsing,
// and it is the one profile that must report zero overflows.
static const uint32_t kImuReadMs = 10;
static const uint32_t kRadioSf12Ms = 3023;   // SF12/BW125/CR4-8, ~3.0 s
static const uint32_t kRadioSf7Ms = 92;      // SF7/BW125/CR4-5, ~92 ms
static const uint32_t kRadioControlMs = 5;   // synthetic, not realizable
static const uint32_t kImuControlMs = 2;

void test_emits_a_decodable_packet(void) {
    SimClock clock;
    FakeGps gps(kSentence, kGpsByteRate);
    clock.add_observer(&gps);
    FakeImu imu(clock, kImuControlMs);
    FakeRadio radio(clock, kRadioControlMs);

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
    FakeImu imu(clock, kImuControlMs);
    FakeRadio radio(clock, kRadioControlMs);

    PollingSampler sampler(gps, imu, radio, clock, 1, 3);
    for (int i = 0; i < 400; ++i) {
        sampler.step();
    }

    Packet decoded;
    TEST_ASSERT_TRUE(decode_packet(radio.last_payload(), radio.last_length(), &decoded));
    TEST_ASSERT_EQUAL_UINT32(sampler.packets_sent() - 1, decoded.seq);
}

void test_sf12_profile_loses_gps_bytes(void) {
    // This is the defect the interrupt-driven rewrite exists to remove, timed
    // against the modem configuration src/hal/teensy_radio.hpp actually
    // programs (Bw125Cr48Sf4096, i.e. SF12/BW125/CR4-8).
    SimClock clock;
    FakeGps gps(kSentence, kGpsByteRate);
    clock.add_observer(&gps);
    FakeImu imu(clock, kImuReadMs);
    FakeRadio radio(clock, kRadioSf12Ms);

    PollingSampler sampler(gps, imu, radio, clock, 1, 3);
    for (int i = 0; i < 400; ++i) {
        sampler.step();
    }

    TEST_ASSERT_GREATER_THAN_UINT16(0, sampler.diag().drops);
}

void test_control_profile_loses_nothing(void) {
    // The mirror of the test above: the loss is a consequence of the stall,
    // not something inherent to polling, and this pins that down. CONTROL is
    // synthetic and faster than any real radio; no realistic profile can use
    // this baseline for comparison.
    SimClock clock;
    FakeGps gps(kSentence, kGpsByteRate);
    clock.add_observer(&gps);
    FakeImu imu(clock, kImuControlMs);
    FakeRadio radio(clock, kRadioControlMs);

    PollingSampler sampler(gps, imu, radio, clock, 1, 3);
    for (int i = 0; i < 400; ++i) {
        sampler.step();
    }

    TEST_ASSERT_EQUAL_UINT16(0, sampler.diag().drops);
}

void test_no_practical_lora_setting_keeps_up(void) {
    // Even the fastest practical LoRa configuration stalls the loop longer
    // than the 64-byte receive buffer can cover at 9600 baud (~66 ms), so
    // the superloop loses data at SF7 as well as at SF12. This is the real
    // conclusion of milestone 1: the defect is the blocking strategy, not
    // the choice of spreading factor.
    SimClock clock;
    FakeGps gps(kSentence, kGpsByteRate);
    clock.add_observer(&gps);
    FakeImu imu(clock, kImuReadMs);
    FakeRadio radio(clock, kRadioSf7Ms);

    PollingSampler sampler(gps, imu, radio, clock, 1, 3);
    for (int i = 0; i < 400; ++i) {
        sampler.step();
    }

    TEST_ASSERT_GREATER_THAN_UINT16(0, sampler.diag().drops);
}

void test_reports_drops_in_the_packet(void) {
    SimClock clock;
    FakeGps gps(kSentence, kGpsByteRate);
    clock.add_observer(&gps);
    FakeImu imu(clock, kImuReadMs);
    FakeRadio radio(clock, kRadioSf12Ms);

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
    FakeImu imu(clock, kImuControlMs);
    FakeRadio radio(clock, kRadioControlMs);

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
    RUN_TEST(test_sf12_profile_loses_gps_bytes);
    RUN_TEST(test_control_profile_loses_nothing);
    RUN_TEST(test_no_practical_lora_setting_keeps_up);
    RUN_TEST(test_reports_drops_in_the_packet);
    RUN_TEST(test_no_transmission_without_a_complete_sentence);
    return UNITY_END();
}
