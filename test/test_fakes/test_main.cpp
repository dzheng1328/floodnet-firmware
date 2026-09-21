#include <string.h>
#include <unity.h>

#include "../support/fake_gps.hpp"
#include "../support/fake_imu.hpp"
#include "../support/fake_radio.hpp"
#include "../support/sim_clock.hpp"

using namespace floodnet;

void setUp(void) {}
void tearDown(void) {}

void test_clock_advances_on_delay(void) {
    SimClock clock;
    TEST_ASSERT_EQUAL_UINT32(0, clock.now_ms());
    clock.delay_ms(25);
    TEST_ASSERT_EQUAL_UINT32(25, clock.now_ms());
}

void test_gps_delivers_bytes_as_time_passes(void) {
    SimClock clock;
    FakeGps gps("$GPGGA\r\n", 1.0);  // one byte per millisecond
    clock.add_observer(&gps);

    clock.delay_ms(3);
    TEST_ASSERT_EQUAL_INT('$', gps.read_byte());
    TEST_ASSERT_EQUAL_INT('G', gps.read_byte());
    TEST_ASSERT_EQUAL_INT('P', gps.read_byte());
    TEST_ASSERT_EQUAL_INT(-1, gps.read_byte());
}

void test_gps_drops_bytes_when_fifo_overflows(void) {
    SimClock clock;
    FakeGps gps("$GPGGA\r\n", 1.0);
    clock.add_observer(&gps);

    // FIFO holds 64 bytes; nobody reads during these 100 ms.
    clock.delay_ms(100);
    TEST_ASSERT_GREATER_THAN_UINT16(0, gps.rx_overflows());
}

void test_imu_read_costs_time(void) {
    SimClock clock;
    FakeImu imu(clock, 10);

    ImuSample sample;
    TEST_ASSERT_TRUE(imu.read(&sample));
    TEST_ASSERT_TRUE(sample.valid);
    TEST_ASSERT_EQUAL_UINT32(10, clock.now_ms());
}

void test_radio_records_transmissions_and_costs_time(void) {
    SimClock clock;
    FakeRadio radio(clock, 60);

    const uint8_t payload[] = {1, 2, 3};
    TEST_ASSERT_TRUE(radio.transmit(payload, sizeof(payload)));
    TEST_ASSERT_EQUAL_UINT(1, radio.sent_count());
    TEST_ASSERT_EQUAL_UINT(3, radio.last_length());
    TEST_ASSERT_EQUAL_UINT8(2, radio.last_payload()[1]);
    TEST_ASSERT_EQUAL_UINT32(60, clock.now_ms());
}

/// An observer that never has anything, used to prove the cap is honoured.
class SilentSource : public ISimTick {
  public:
    void on_tick(uint32_t) override {}
    bool pending() const override { return false; }
};

void test_wait_for_event_returns_at_once_when_data_is_already_waiting(void) {
    SimClock clock;
    FakeGps gps("$GPGGA\r\n", 1.0);
    clock.add_observer(&gps);

    clock.delay_ms(3);  // three bytes are now waiting
    const uint32_t before = clock.now_ms();

    clock.wait_for_event(50);

    TEST_ASSERT_EQUAL_UINT32(before, clock.now_ms());
}

void test_wait_for_event_advances_time_until_data_arrives(void) {
    SimClock clock;
    // 0.125 is 2^-3 and therefore exact in binary floating point. FakeGps
    // accumulates `pending_ += elapsed_ms * bytes_per_ms_` and pushes a byte
    // when that reaches 1.0, so an inexact rate accumulates rounding error:
    // 0.1 summed ten times gives 0.9999999999999999 and the byte lands on
    // tick 11, not 10. Use a rate the accumulator can represent exactly.
    FakeGps gps("$GPGGA\r\n", 0.125);  // one byte every 8 ms, exactly
    clock.add_observer(&gps);

    clock.wait_for_event(50);

    // Woke as soon as the first byte landed, not at the cap.
    TEST_ASSERT_EQUAL_UINT32(8, clock.now_ms());
    TEST_ASSERT_EQUAL_INT('$', gps.read_byte());
}

void test_wait_for_event_gives_up_at_the_cap(void) {
    SimClock clock;
    SilentSource silent;
    clock.add_observer(&silent);

    clock.wait_for_event(25);

    TEST_ASSERT_EQUAL_UINT32(25, clock.now_ms());
}

void test_wait_for_event_advances_even_with_no_observers(void) {
    // A loop with nothing registered must still make progress rather than
    // spin forever with the clock frozen.
    SimClock clock;

    clock.wait_for_event(7);

    TEST_ASSERT_EQUAL_UINT32(7, clock.now_ms());
}

void test_gps_default_depth_matches_milestone_one(void) {
    SimClock clock;
    FakeGps gps("$GPGGA\r\n", 1.0);
    clock.add_observer(&gps);

    // 64 bytes fit; the 65th onward are lost.
    clock.delay_ms(64);
    TEST_ASSERT_EQUAL_UINT16(0, gps.rx_overflows());
    clock.delay_ms(10);
    TEST_ASSERT_EQUAL_UINT16(10, gps.rx_overflows());
}

void test_gps_deeper_buffer_absorbs_what_the_default_loses(void) {
    SimClock clock;
    FakeGps gps("$GPGGA\r\n", 1.0, 4096);
    clock.add_observer(&gps);

    // The same 74 ms that overflowed the 64-byte buffer above.
    clock.delay_ms(74);
    TEST_ASSERT_EQUAL_UINT16(0, gps.rx_overflows());

    // A full SF12 transmit still fits.
    clock.delay_ms(3023);
    TEST_ASSERT_EQUAL_UINT16(0, gps.rx_overflows());
}

int main(int, char **) {
    UNITY_BEGIN();
    RUN_TEST(test_clock_advances_on_delay);
    RUN_TEST(test_gps_delivers_bytes_as_time_passes);
    RUN_TEST(test_gps_drops_bytes_when_fifo_overflows);
    RUN_TEST(test_imu_read_costs_time);
    RUN_TEST(test_radio_records_transmissions_and_costs_time);
    RUN_TEST(test_wait_for_event_returns_at_once_when_data_is_already_waiting);
    RUN_TEST(test_wait_for_event_advances_time_until_data_arrives);
    RUN_TEST(test_wait_for_event_gives_up_at_the_cap);
    RUN_TEST(test_wait_for_event_advances_even_with_no_observers);
    RUN_TEST(test_gps_default_depth_matches_milestone_one);
    RUN_TEST(test_gps_deeper_buffer_absorbs_what_the_default_loses);
    return UNITY_END();
}
