#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include <vector>

#include <unity.h>

#include <floodnet/hal/tx_listener.hpp>
#include <floodnet/packet.hpp>

#include "../support/fake_gps.hpp"
#include "../support/fake_imu.hpp"
#include "../support/fake_radio.hpp"
#include "../support/sim_clock.hpp"
#include "sampler_interrupt.hpp"

using namespace floodnet;

void setUp(void) {}
void tearDown(void) {}

static const double kGpsByteRate = 0.96;
static const char kSentence[] =
    "$GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,*47\r\n";
static const uint32_t kImuMs = 2;
static const uint32_t kRadioMs = 5;

struct Event {
    uint16_t node_id;
    uint16_t boot_count;
    uint32_t seq;
    uint32_t now_ms;
    bool completed;
};

struct Recorder : public ITxListener {
    std::vector<Event> events;
    void on_tx_end(uint16_t node_id, uint16_t boot_count, uint32_t seq, uint32_t now_ms,
                   bool completed) override {
        Event e = {node_id, boot_count, seq, now_ms, completed};
        events.push_back(e);
    }
};

struct Rig {
    SimClock clock;
    FakeGps gps;
    FakeImu imu;
    FakeRadio radio;
    InterruptSampler sampler;

    explicit Rig(WireVersion version)
        : clock(),
          gps(kSentence, kGpsByteRate, FakeGps::MAX_FIFO_DEPTH),
          imu(clock, kImuMs),
          radio(clock, kRadioMs),
          sampler(gps, imu, radio, clock, 0x0042, 3, version) {
        clock.add_observer(&gps);
        clock.add_observer(&imu);
        clock.add_observer(&radio);
    }

    void run_for(uint32_t duration_ms) {
        const uint32_t end = clock.now_ms() + duration_ms;
        while (clock.now_ms() < end) {
            sampler.step();
        }
    }
};

void test_completed_transmits_are_reported(void) {
    Rig rig(WireVersion::V1);
    Recorder rec;
    rig.sampler.set_tx_listener(&rec);
    rig.run_for(2000);
    TEST_ASSERT_TRUE(rec.events.size() > 0);
    TEST_ASSERT_EQUAL_UINT32(rig.sampler.packets_sent(), rec.events.size());
    for (size_t i = 0; i < rec.events.size(); ++i) {
        TEST_ASSERT_TRUE(rec.events[i].completed);
        TEST_ASSERT_EQUAL_UINT16(0x0042, rec.events[i].node_id);
        TEST_ASSERT_EQUAL_UINT16(0, rec.events[i].boot_count);
    }
}

void test_reported_seq_is_the_transmitted_frame(void) {
    Rig rig(WireVersion::V1);
    Recorder rec;
    rig.sampler.set_tx_listener(&rec);
    rig.run_for(2000);
    // Stop at a pass boundary where the last frame's end has been observed.
    while (rig.sampler.tx_in_flight()) {
        rig.sampler.step();
    }
    Packet last;
    TEST_ASSERT_TRUE(decode_packet(rig.radio.last_payload(), rig.radio.last_length(), &last));
    TEST_ASSERT_EQUAL_UINT32(last.seq, rec.events.back().seq);
}

void test_v2_reports_the_boot_count(void) {
    Rig rig(WireVersion::V2);
    Recorder rec;
    rig.sampler.set_tx_listener(&rec);
    rig.sampler.set_node_status(7, 3700);
    rig.run_for(2000);
    TEST_ASSERT_TRUE(rec.events.size() > 0);
    TEST_ASSERT_EQUAL_UINT16(7, rec.events.back().boot_count);
}

void test_timeout_is_reported_as_not_completed(void) {
    Rig rig(WireVersion::V1);
    Recorder rec;
    rig.sampler.set_tx_listener(&rec);
    rig.radio.lose_next_completion();
    rig.run_for(InterruptSampler::TX_TIMEOUT_MS + 1000);
    TEST_ASSERT_EQUAL_UINT16(1, rig.sampler.tx_timeouts());
    TEST_ASSERT_TRUE(rec.events.size() > 0);
    TEST_ASSERT_FALSE(rec.events[0].completed);
    TEST_ASSERT_TRUE(rec.events[0].now_ms >= InterruptSampler::TX_TIMEOUT_MS);
}

void test_now_ms_is_the_clock_at_the_end(void) {
    Rig rig(WireVersion::V1);
    Recorder rec;
    rig.sampler.set_tx_listener(&rec);
    rig.run_for(2000);
    TEST_ASSERT_TRUE(rec.events.size() > 1);
    TEST_ASSERT_TRUE(rec.events[0].now_ms >= kRadioMs);
    TEST_ASSERT_TRUE(rec.events.back().now_ms <= rig.clock.now_ms());
}

void test_no_listener_changes_nothing(void) {
    Rig plain(WireVersion::V1);
    Rig heard(WireVersion::V1);
    Recorder rec;
    heard.sampler.set_tx_listener(&rec);
    plain.run_for(5000);
    heard.run_for(5000);
    TEST_ASSERT_EQUAL_UINT32(plain.sampler.packets_sent(), heard.sampler.packets_sent());
    TEST_ASSERT_EQUAL_UINT32(plain.sampler.next_seq(), heard.sampler.next_seq());
    TEST_ASSERT_EQUAL_UINT32(plain.radio.last_length(), heard.radio.last_length());
    TEST_ASSERT_EQUAL_MEMORY(plain.radio.last_payload(), heard.radio.last_payload(),
                             plain.radio.last_length());
}

int main(int, char **) {
    UNITY_BEGIN();
    RUN_TEST(test_completed_transmits_are_reported);
    RUN_TEST(test_reported_seq_is_the_transmitted_frame);
    RUN_TEST(test_v2_reports_the_boot_count);
    RUN_TEST(test_timeout_is_reported_as_not_completed);
    RUN_TEST(test_now_ms_is_the_clock_at_the_end);
    RUN_TEST(test_no_listener_changes_nothing);
    return UNITY_END();
}
