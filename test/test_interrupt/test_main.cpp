#include <unity.h>

#include <floodnet/packet.hpp>

#include "../support/fake_gps.hpp"
#include "../support/fake_imu.hpp"
#include "../support/fake_radio.hpp"
#include "../support/sim_clock.hpp"
#include "sampler_interrupt.hpp"

using namespace floodnet;

void setUp(void) {}
void tearDown(void) {}

// Radio and IMU timings match test/test_polling/'s profiles, so any timing
// difference between the two suites is the sampler, not the setup. The GPS
// buffer here is FakeGps::MAX_FIFO_DEPTH, the milestone 2 configuration,
// rather than test_polling's default depth - isolating that variable is the
// benchmark's job in a later task.
static const double kGpsByteRate = 0.96;
static const char kSentence[] =
    "$GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,*47\r\n";

static const uint32_t kImuReadMs = 10;
static const uint32_t kRadioSf12Ms = 3023;
static const uint32_t kRadioControlMs = 5;
static const uint32_t kImuControlMs = 2;

/// Everything one run needs, wired together. All three fakes are registered as
/// clock observers here, unlike the polling suite, because a non-blocking loop
/// depends on the clock to advance them.
struct Rig {
    SimClock clock;
    FakeGps gps;
    FakeImu imu;
    FakeRadio radio;
    InterruptSampler sampler;

    Rig(uint32_t imu_ms, uint32_t radio_ms, size_t gps_depth)
        : clock(),
          gps(kSentence, kGpsByteRate, gps_depth),
          imu(clock, imu_ms),
          radio(clock, radio_ms),
          sampler(gps, imu, radio, clock, 0x0042, 3) {
        clock.add_observer(&gps);
        clock.add_observer(&imu);
        clock.add_observer(&radio);
    }

    void run_for(uint32_t duration_ms) {
        while (clock.now_ms() < duration_ms) {
            sampler.step();
        }
    }
};

void test_emits_a_decodable_packet(void) {
    Rig rig(kImuControlMs, kRadioControlMs, FakeGps::MAX_FIFO_DEPTH);
    rig.run_for(2000);

    TEST_ASSERT_GREATER_THAN_UINT32(0, rig.sampler.packets_sent());

    Packet decoded;
    TEST_ASSERT_TRUE(decode_packet(rig.radio.last_payload(), rig.radio.last_length(), &decoded));
    TEST_ASSERT_EQUAL_UINT16(0x0042, decoded.node_id);
    TEST_ASSERT_EQUAL_UINT8(3, decoded.ttl);
    TEST_ASSERT_TRUE(decoded.record.gps.valid);
    TEST_ASSERT_EQUAL_INT32(481173000, decoded.record.gps.lat_1e7);
}

void test_sf12_loses_no_gps_bytes(void) {
    // The assertion milestone 1 could not make. test_polling's
    // test_sf12_profile_loses_gps_bytes asserts the opposite against the same
    // profile, and that contrast is the milestone.
    Rig rig(kImuReadMs, kRadioSf12Ms, FakeGps::MAX_FIFO_DEPTH);
    rig.run_for(60000);

    TEST_ASSERT_EQUAL_UINT16(0, rig.sampler.diag().drops);
}

void test_sf12_drops_whole_packets_instead(void) {
    // Loss does not vanish at SF12; the radio still cannot keep up. It moves
    // from silent byte-level corruption to counted, deliberate packet drops.
    // This test exists so nobody reads the test above as "loss was fixed".
    Rig rig(kImuReadMs, kRadioSf12Ms, FakeGps::MAX_FIFO_DEPTH);
    rig.run_for(60000);

    TEST_ASSERT_GREATER_THAN_UINT16(0, rig.sampler.tx_queue_drops());
}

void test_dropped_packets_leave_visible_sequence_gaps(void) {
    // Sequence numbers are assigned when a fix is queued, not when it is sent,
    // so a receiver can see exactly how many observations the node discarded.
    //
    // Asserting the identity rather than just next_seq() > packets_sent():
    // that weaker form passes even when seq_ is assigned at transmit time,
    // because the single in-flight packet at the end of a run supplies the
    // whole margin. The gap must be at least as large as the number of
    // packets the queue actually threw away.
    Rig rig(kImuReadMs, kRadioSf12Ms, FakeGps::MAX_FIFO_DEPTH);
    rig.run_for(60000);

    TEST_ASSERT_GREATER_OR_EQUAL_UINT32(
        rig.sampler.tx_queue_drops(),
        rig.sampler.next_seq() - rig.sampler.packets_sent());
}

void test_control_profile_loses_nothing_at_all(void) {
    Rig rig(kImuControlMs, kRadioControlMs, FakeGps::MAX_FIFO_DEPTH);
    rig.run_for(60000);

    TEST_ASSERT_EQUAL_UINT16(0, rig.sampler.diag().drops);
    TEST_ASSERT_EQUAL_UINT16(0, rig.sampler.tx_queue_drops());
}

void test_tx_queue_high_water_tracks_radio_pressure(void) {
    // tx_queue_high_water is a published column in every BENCH row, so pin it.
    // At SF12 the radio cannot keep up and the queue fills to capacity; at
    // CONTROL each packet is sent before the next fix arrives.
    Rig sf12(kImuReadMs, kRadioSf12Ms, FakeGps::MAX_FIFO_DEPTH);
    sf12.run_for(60000);
    TEST_ASSERT_EQUAL_size_t(8, sf12.sampler.tx_queue_high_water());

    Rig control(kImuControlMs, kRadioControlMs, FakeGps::MAX_FIFO_DEPTH);
    control.run_for(60000);
    TEST_ASSERT_EQUAL_size_t(1, control.sampler.tx_queue_high_water());
}

void test_collects_gps_while_the_radio_is_transmitting(void) {
    // The defect milestone 1 had, stated directly: during a 3-second SF12
    // transmit the loop must still be draining the receive buffer.
    Rig rig(kImuReadMs, kRadioSf12Ms, FakeGps::MAX_FIFO_DEPTH);

    // Get one transmit started, then confirm fixes keep accumulating while it
    // is in flight.
    while (!rig.radio.tx_busy() && rig.clock.now_ms() < 10000) {
        rig.sampler.step();
    }
    TEST_ASSERT_TRUE(rig.radio.tx_busy());

    const uint32_t seq_at_tx_start = rig.sampler.next_seq();
    const uint32_t started_at = rig.clock.now_ms();
    while (rig.clock.now_ms() < started_at + 1000) {
        rig.sampler.step();
    }

    TEST_ASSERT_TRUE(rig.radio.tx_busy());  // still on the air
    TEST_ASSERT_GREATER_THAN_UINT32(seq_at_tx_start, rig.sampler.next_seq());
    TEST_ASSERT_EQUAL_UINT16(0, rig.sampler.diag().drops);
}

void test_a_lost_completion_edge_does_not_mute_the_node(void) {
    // On hardware, tx_busy() clears only when RadioHead's DIO0 handler runs.
    // Lose that one edge and, without a deadline, the node never transmits
    // again while tx_queue_drops climbs forever. The parent spec requires
    // radio operations to carry timeouts; this is the async path's.
    Rig rig(kImuControlMs, kRadioControlMs, FakeGps::MAX_FIFO_DEPTH);
    rig.radio.lose_next_completion();
    rig.run_for(60000);

    // Exactly one transmission was abandoned, and it was counted.
    TEST_ASSERT_EQUAL_UINT16(1, rig.sampler.tx_timeouts());

    // And the node kept sending afterwards. CONTROL sends ~859 packets in 60 s
    // when nothing goes wrong; one 5 s stall costs at most a twelfth of that.
    TEST_ASSERT_GREATER_THAN_UINT32(700, rig.sampler.packets_sent());
}

void test_accounting_stays_closed_across_a_timeout(void) {
    // Every sequence number assigned is either sent, dropped from the queue,
    // abandoned on timeout, or still waiting. A timeout that counted nothing
    // would leave a gap no counter explains.
    Rig rig(kImuReadMs, kRadioSf12Ms, FakeGps::MAX_FIFO_DEPTH);
    rig.radio.lose_next_completion();
    rig.run_for(60000);

    const uint32_t accounted = rig.sampler.packets_sent() + rig.sampler.tx_queue_drops() +
                               rig.sampler.tx_timeouts() + rig.sampler.tx_queue_size() +
                               (rig.sampler.tx_in_flight() ? 1u : 0u);
    TEST_ASSERT_EQUAL_UINT16(1, rig.sampler.tx_timeouts());
    TEST_ASSERT_EQUAL_UINT32(rig.sampler.next_seq(), accounted);
}

void test_normal_sf12_airtime_is_not_mistaken_for_a_wedge(void) {
    // 3023 ms of airtime must stay well inside the deadline.
    Rig rig(kImuReadMs, kRadioSf12Ms, FakeGps::MAX_FIFO_DEPTH);
    rig.run_for(60000);

    TEST_ASSERT_EQUAL_UINT16(0, rig.sampler.tx_timeouts());
}

void test_no_transmission_without_a_complete_sentence(void) {
    SimClock clock;
    FakeGps gps("garbage without a dollar sign", kGpsByteRate,
                FakeGps::MAX_FIFO_DEPTH);
    FakeImu imu(clock, kImuControlMs);
    FakeRadio radio(clock, kRadioControlMs);
    clock.add_observer(&gps);
    clock.add_observer(&imu);
    clock.add_observer(&radio);

    InterruptSampler sampler(gps, imu, radio, clock, 1, 3);
    while (clock.now_ms() < 2000) {
        sampler.step();
    }

    TEST_ASSERT_EQUAL_UINT32(0, sampler.packets_sent());
    TEST_ASSERT_EQUAL_UINT32(0, sampler.next_seq());
}

/// A v0x02 sampler on the CONTROL profile, built the way the Rig builds one.
struct V2Rig {
    SimClock clock;
    FakeGps gps;
    FakeImu imu;
    FakeRadio radio;
    InterruptSampler sampler;

    V2Rig()
        : clock(),
          gps(kSentence, kGpsByteRate, FakeGps::MAX_FIFO_DEPTH),
          imu(clock, kImuControlMs),
          radio(clock, kRadioControlMs),
          sampler(gps, imu, radio, clock, 0x0042, 3, WireVersion::V2) {
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

void test_default_wire_version_is_still_v1(void) {
    Rig rig(kImuControlMs, kRadioControlMs, FakeGps::MAX_FIFO_DEPTH);
    rig.run_for(2000);
    TEST_ASSERT_EQUAL_HEX8(0x01, rig.radio.last_payload()[1]);
}

void test_v2_sampler_stamps_node_status_at_send_time(void) {
    V2Rig rig;
    rig.sampler.set_node_status(7, 3700);
    rig.run_for(2000);

    TEST_ASSERT_EQUAL_HEX8(0x02, rig.radio.last_payload()[1]);
    Packet decoded;
    TEST_ASSERT_TRUE(decode_packet(rig.radio.last_payload(), rig.radio.last_length(), &decoded));
    TEST_ASSERT_EQUAL_UINT16(7, decoded.boot_count);
    TEST_ASSERT_EQUAL_UINT16(3700, decoded.battery_mv);
    TEST_ASSERT_EQUAL_UINT16(0, decoded.tx_timeouts);
}

void test_heartbeat_takes_a_sequence_number_and_reports_no_fix(void) {
    V2Rig rig;
    rig.gps.set_powered(false);
    rig.sampler.enqueue_heartbeat();
    TEST_ASSERT_EQUAL_UINT32(1, rig.sampler.next_seq());

    rig.run_for(100);
    TEST_ASSERT_EQUAL_UINT32(1, rig.sampler.packets_sent());
    Packet decoded;
    TEST_ASSERT_TRUE(decode_packet(rig.radio.last_payload(), rig.radio.last_length(), &decoded));
    TEST_ASSERT_EQUAL_UINT32(0, decoded.seq);
    TEST_ASSERT_FALSE(decoded.record.gps.valid);
}

void test_last_record_imu_valid_tracks_pairing(void) {
    V2Rig with_imu;
    with_imu.run_for(200);
    TEST_ASSERT_GREATER_THAN_UINT32(0, with_imu.sampler.next_seq());
    TEST_ASSERT_TRUE(with_imu.sampler.last_record_imu_valid());

    V2Rig without_imu;
    without_imu.imu.set_powered(false);
    without_imu.run_for(200);
    TEST_ASSERT_GREATER_THAN_UINT32(0, without_imu.sampler.next_seq());
    TEST_ASSERT_FALSE(without_imu.sampler.last_record_imu_valid());
}

int main(int, char **) {
    UNITY_BEGIN();
    RUN_TEST(test_emits_a_decodable_packet);
    RUN_TEST(test_sf12_loses_no_gps_bytes);
    RUN_TEST(test_sf12_drops_whole_packets_instead);
    RUN_TEST(test_dropped_packets_leave_visible_sequence_gaps);
    RUN_TEST(test_control_profile_loses_nothing_at_all);
    RUN_TEST(test_tx_queue_high_water_tracks_radio_pressure);
    RUN_TEST(test_collects_gps_while_the_radio_is_transmitting);
    RUN_TEST(test_a_lost_completion_edge_does_not_mute_the_node);
    RUN_TEST(test_accounting_stays_closed_across_a_timeout);
    RUN_TEST(test_normal_sf12_airtime_is_not_mistaken_for_a_wedge);
    RUN_TEST(test_no_transmission_without_a_complete_sentence);
    RUN_TEST(test_default_wire_version_is_still_v1);
    RUN_TEST(test_v2_sampler_stamps_node_status_at_send_time);
    RUN_TEST(test_heartbeat_takes_a_sequence_number_and_reports_no_fix);
    RUN_TEST(test_last_record_imu_valid_tracks_pairing);
    return UNITY_END();
}
