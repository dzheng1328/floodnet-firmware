#include <string.h>
#include <unity.h>
#include <floodnet/packet.hpp>

using namespace floodnet;

void setUp(void) {}
void tearDown(void) {}

static Packet make_packet(void) {
    Packet p;
    p.node_id = 0x1234;
    p.seq = 0xDEADBEEF;
    p.ttl = 3;
    p.flags = 0;
    p.record.gps.time_ms = 1000;
    p.record.gps.lat_1e7 = 481173000;
    p.record.gps.lon_1e7 = -115166667;
    p.record.gps.alt_mm = 545400;
    p.record.gps.satellites = 8;
    p.record.gps.valid = true;
    p.record.imu.time_ms = 995;
    p.record.imu.yaw_cd = -4500;
    p.record.imu.pitch_cd = 250;
    p.record.imu.roll_cd = 0;
    p.record.imu.valid = true;
    p.record.diag.drops = 7;
    p.record.diag.crc_errors = 2;
    return p;
}

void test_crc16_matches_known_vector(void) {
    // CRC16-CCITT-FALSE of "123456789" is 0x29B1.
    const uint8_t input[] = {'1', '2', '3', '4', '5', '6', '7', '8', '9'};
    TEST_ASSERT_EQUAL_HEX16(0x29B1, crc16_ccitt(input, sizeof(input)));
}

void test_encode_returns_packet_size(void) {
    uint8_t buf[PACKET_SIZE];
    TEST_ASSERT_EQUAL_UINT(PACKET_SIZE, encode_packet(make_packet(), buf, sizeof(buf)));
}

void test_encode_rejects_short_buffer(void) {
    uint8_t buf[PACKET_SIZE - 1];
    TEST_ASSERT_EQUAL_UINT(0, encode_packet(make_packet(), buf, sizeof(buf)));
}

void test_round_trip_preserves_every_field(void) {
    uint8_t buf[PACKET_SIZE];
    Packet original = make_packet();
    TEST_ASSERT_EQUAL_UINT(PACKET_SIZE, encode_packet(original, buf, sizeof(buf)));

    Packet decoded;
    TEST_ASSERT_TRUE(decode_packet(buf, sizeof(buf), &decoded));

    TEST_ASSERT_EQUAL_UINT16(original.node_id, decoded.node_id);
    TEST_ASSERT_EQUAL_UINT32(original.seq, decoded.seq);
    TEST_ASSERT_EQUAL_UINT8(original.ttl, decoded.ttl);
    TEST_ASSERT_EQUAL_UINT32(original.record.gps.time_ms, decoded.record.gps.time_ms);
    TEST_ASSERT_EQUAL_INT32(original.record.gps.lat_1e7, decoded.record.gps.lat_1e7);
    TEST_ASSERT_EQUAL_INT32(original.record.gps.lon_1e7, decoded.record.gps.lon_1e7);
    TEST_ASSERT_EQUAL_INT32(original.record.gps.alt_mm, decoded.record.gps.alt_mm);
    TEST_ASSERT_EQUAL_UINT8(original.record.gps.satellites, decoded.record.gps.satellites);
    TEST_ASSERT_TRUE(decoded.record.gps.valid);
    TEST_ASSERT_EQUAL_UINT32(original.record.imu.time_ms, decoded.record.imu.time_ms);
    TEST_ASSERT_EQUAL_INT16(original.record.imu.yaw_cd, decoded.record.imu.yaw_cd);
    TEST_ASSERT_EQUAL_INT16(original.record.imu.pitch_cd, decoded.record.imu.pitch_cd);
    TEST_ASSERT_TRUE(decoded.record.imu.valid);
    TEST_ASSERT_EQUAL_UINT16(original.record.diag.drops, decoded.record.diag.drops);
    TEST_ASSERT_EQUAL_UINT16(original.record.diag.crc_errors, decoded.record.diag.crc_errors);
}

void test_decode_rejects_corrupted_payload(void) {
    uint8_t buf[PACKET_SIZE];
    encode_packet(make_packet(), buf, sizeof(buf));
    buf[20] ^= 0xFF;  // flip bits inside the longitude field

    Packet decoded;
    TEST_ASSERT_FALSE(decode_packet(buf, sizeof(buf), &decoded));
}

void test_decode_rejects_bad_magic(void) {
    uint8_t buf[PACKET_SIZE];
    encode_packet(make_packet(), buf, sizeof(buf));
    buf[0] = 0x00;

    Packet decoded;
    TEST_ASSERT_FALSE(decode_packet(buf, sizeof(buf), &decoded));
}

void test_decode_rejects_wrong_length(void) {
    uint8_t buf[PACKET_SIZE];
    encode_packet(make_packet(), buf, sizeof(buf));

    Packet decoded;
    TEST_ASSERT_FALSE(decode_packet(buf, PACKET_SIZE - 1, &decoded));
}

int main(int, char **) {
    UNITY_BEGIN();
    RUN_TEST(test_crc16_matches_known_vector);
    RUN_TEST(test_encode_returns_packet_size);
    RUN_TEST(test_encode_rejects_short_buffer);
    RUN_TEST(test_round_trip_preserves_every_field);
    RUN_TEST(test_decode_rejects_corrupted_payload);
    RUN_TEST(test_decode_rejects_bad_magic);
    RUN_TEST(test_decode_rejects_wrong_length);
    return UNITY_END();
}
