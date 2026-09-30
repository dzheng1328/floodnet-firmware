#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include <unity.h>

#include <floodnet/gateway_format.hpp>
#include <floodnet/packet.hpp>

using namespace floodnet;

void setUp(void) {}
void tearDown(void) {}

static Packet full_packet() {
    Packet p;
    p.node_id = 0x0042;
    p.seq = 123456;
    p.record.gps.time_ms = 987654;
    p.record.gps.lat_1e7 = 481173000;
    p.record.gps.lon_1e7 = -115166667;
    p.record.gps.alt_mm = -1200;
    p.record.gps.satellites = 8;
    p.record.gps.valid = true;
    p.record.imu.yaw_cd = -4500;
    p.record.imu.pitch_cd = 1234;
    p.record.imu.roll_cd = -1;
    p.record.imu.valid = false;
    p.record.diag.drops = 7;
    p.record.diag.crc_errors = 0;
    p.boot_count = 3;
    p.tx_timeouts = 2;
    p.battery_mv = 3700;
    return p;
}

void test_rec_line_matches_the_gateway_field_order(void) {
    char line[REC_LINE_MAX];
    const size_t n = format_rec_line(full_packet(), -97, line, sizeof(line));
    TEST_ASSERT_EQUAL_STRING(
        "REC,66,123456,987654,481173000,-115166667,-1200,8,-4500,1234,-1,7,0,-97,1,0,3,2,3700",
        line);
    TEST_ASSERT_EQUAL_UINT32(strlen(line), n);
}

void test_rec_line_returns_zero_when_the_buffer_is_too_small(void) {
    char line[10];
    TEST_ASSERT_EQUAL_UINT32(0, format_rec_line(full_packet(), -97, line, sizeof(line)));
}

void test_decode_error_line(void) {
    char line[32];
    const size_t n = format_decode_error_line(5, line, sizeof(line));
    TEST_ASSERT_EQUAL_STRING("ERR,decode,5", line);
    TEST_ASSERT_EQUAL_UINT32(12, n);
}

void test_decode_error_line_returns_zero_when_the_buffer_is_too_small(void) {
    char line[4];
    TEST_ASSERT_EQUAL_UINT32(0, format_decode_error_line(5, line, sizeof(line)));
}

int main(int, char **) {
    UNITY_BEGIN();
    RUN_TEST(test_rec_line_matches_the_gateway_field_order);
    RUN_TEST(test_rec_line_returns_zero_when_the_buffer_is_too_small);
    RUN_TEST(test_decode_error_line);
    RUN_TEST(test_decode_error_line_returns_zero_when_the_buffer_is_too_small);
    return UNITY_END();
}
