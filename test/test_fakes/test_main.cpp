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
    TEST_ASSERT_GREATER_THAN_UINT16(0, gps.bytes_dropped());
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

int main(int, char **) {
    UNITY_BEGIN();
    RUN_TEST(test_clock_advances_on_delay);
    RUN_TEST(test_gps_delivers_bytes_as_time_passes);
    RUN_TEST(test_gps_drops_bytes_when_fifo_overflows);
    RUN_TEST(test_imu_read_costs_time);
    RUN_TEST(test_radio_records_transmissions_and_costs_time);
    return UNITY_END();
}
