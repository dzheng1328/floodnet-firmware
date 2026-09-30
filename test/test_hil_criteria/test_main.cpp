#include <math.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include <unity.h>

#include <floodnet/packet.hpp>

#include "../support/hil.hpp"

using namespace floodnet;

void setUp(void) {}
void tearDown(void) {}

void test_link_frame_decodes(void) {
    uint8_t frame[PACKET_SIZE];
    build_link_frame(7, frame);
    Packet p;
    TEST_ASSERT_TRUE(decode_packet(frame, PACKET_SIZE, &p));
    TEST_ASSERT_EQUAL_UINT32(7, p.seq);
}

void test_crc_matched_corruption_is_undetected(void) {
    uint8_t sent[PACKET_SIZE];
    uint8_t rx[PACKET_SIZE];
    build_link_frame(7, sent);
    memcpy(rx, sent, PACKET_SIZE);
    rx[20] ^= 0x01;
    const uint16_t crc = crc16_ccitt(rx, 43);
    rx[43] = static_cast<uint8_t>(crc & 0xFF);
    rx[44] = static_cast<uint8_t>(crc >> 8);
    Packet p;
    const bool accepted = decode_packet(rx, PACKET_SIZE, &p);
    TEST_ASSERT_TRUE(accepted);
    TEST_ASSERT_TRUE(is_undetected(sent, rx, PACKET_SIZE, accepted));
}

void test_plain_corruption_is_rejected(void) {
    uint8_t sent[PACKET_SIZE];
    uint8_t rx[PACKET_SIZE];
    build_link_frame(7, sent);
    memcpy(rx, sent, PACKET_SIZE);
    rx[20] ^= 0x01;
    Packet p;
    const bool accepted = decode_packet(rx, PACKET_SIZE, &p);
    TEST_ASSERT_FALSE(accepted);
    TEST_ASSERT_FALSE(is_undetected(sent, rx, PACKET_SIZE, accepted));
}

void test_clean_frame_is_not_undetected(void) {
    uint8_t sent[PACKET_SIZE];
    build_link_frame(3, sent);
    TEST_ASSERT_FALSE(is_undetected(sent, sent, PACKET_SIZE, true));
}

void test_run_link_at_zero_p_accepts_everything(void) {
    const LinkResult r = run_link(0.0, 1000, kChannelSeed);
    TEST_ASSERT_EQUAL_UINT32(1000, r.frames);
    TEST_ASSERT_EQUAL_UINT32(1000, r.accepted);
    TEST_ASSERT_EQUAL_UINT32(0, r.corrupted);
    TEST_ASSERT_EQUAL_UINT32(0, r.undetected);
}

void test_run_link_at_p_one_accepts_nothing(void) {
    const LinkResult r = run_link(1.0, 1000, kChannelSeed);
    TEST_ASSERT_EQUAL_UINT32(1000, r.corrupted);
    TEST_ASSERT_EQUAL_UINT32(0, r.accepted);
}

void test_expected_ratio(void) {
    // Unity's double assertions are disabled in this build, so compare directly.
    TEST_ASSERT_TRUE(expected_ratio(0.0) == 1.0);
    TEST_ASSERT_TRUE(fabs(expected_ratio(1e-3) - pow(0.999, 360.0)) < 1e-12);
}

void test_link_pass_is_exact_at_zero_p(void) {
    TEST_ASSERT_TRUE(link_pass(100000, 100000, 1.0));
    TEST_ASSERT_FALSE(link_pass(99999, 100000, 1.0));
}

void test_link_pass_uses_three_sigma(void) {
    // N = 10000, q = 0.5: mean 5000, sd 50, so 5150 passes and 5151 fails.
    TEST_ASSERT_TRUE(link_pass(5150, 10000, 0.5));
    TEST_ASSERT_FALSE(link_pass(5151, 10000, 0.5));
    TEST_ASSERT_TRUE(link_pass(4850, 10000, 0.5));
    TEST_ASSERT_FALSE(link_pass(4849, 10000, 0.5));
}

void test_integrity_bound(void) {
    TEST_ASSERT_TRUE(integrity_bound(0) == 1.0);
    // corrupted = 65536 * 4: lambda 4, bound 4 + 6 + 1 = 11.
    TEST_ASSERT_TRUE(integrity_bound(262144) == 11.0);
    TEST_ASSERT_TRUE(integrity_pass(11, 262144));
    TEST_ASSERT_FALSE(integrity_pass(12, 262144));
}

// Pinned to docs/results/milestone-4-hil-regression.txt: the refactor of
// run_link() must not move milestone 4's numbers.
void test_run_link_reproduces_milestone_4_rows(void) {
    const LinkResult a = run_link(1e-3, kLinkFrames, kChannelSeed);
    TEST_ASSERT_EQUAL_UINT32(69617, a.accepted);
    TEST_ASSERT_EQUAL_UINT32(0, a.undetected);
    const LinkResult b = run_link(1e-2, kLinkFrames, kChannelSeed);
    TEST_ASSERT_EQUAL_UINT32(2741, b.accepted);
    TEST_ASSERT_EQUAL_UINT32(2, b.undetected);
}

// x^16 + x^12 + x^5 + 1 placed in the data bytes is a multiple of the CRC
// polynomial, so the CRC cannot see it.
void test_crc_polynomial_pattern_is_undetected(void) {
    uint8_t sent[PACKET_SIZE];
    build_link_frame(0, sent);
    uint8_t errors[PACKET_SIZE];
    memset(errors, 0, sizeof errors);
    const size_t start = 100;
    const size_t offsets[] = {0, 4, 11, 16};
    for (size_t i = 0; i < 4; ++i) {
        const size_t bit = start + offsets[i];
        errors[bit / 8] |= static_cast<uint8_t>(0x80u >> (bit % 8));
    }
    TEST_ASSERT_TRUE(pattern_undetected(sent, errors));
}

void test_single_bit_pattern_is_detected(void) {
    uint8_t sent[PACKET_SIZE];
    build_link_frame(0, sent);
    uint8_t errors[PACKET_SIZE];
    memset(errors, 0, sizeof errors);
    errors[25] = 0x10;
    TEST_ASSERT_FALSE(pattern_undetected(sent, errors));
}

void test_crc_field_starts_at_byte_43(void) {
    TEST_ASSERT_EQUAL_UINT32(344, kCrcFieldFirstBit);
    TEST_ASSERT_FALSE(in_crc_field(343));
    TEST_ASSERT_TRUE(in_crc_field(344));
    TEST_ASSERT_TRUE(in_crc_field(359));
}

int main(int, char **) {
    UNITY_BEGIN();
    RUN_TEST(test_link_frame_decodes);
    RUN_TEST(test_crc_matched_corruption_is_undetected);
    RUN_TEST(test_plain_corruption_is_rejected);
    RUN_TEST(test_clean_frame_is_not_undetected);
    RUN_TEST(test_run_link_at_zero_p_accepts_everything);
    RUN_TEST(test_run_link_at_p_one_accepts_nothing);
    RUN_TEST(test_expected_ratio);
    RUN_TEST(test_link_pass_is_exact_at_zero_p);
    RUN_TEST(test_link_pass_uses_three_sigma);
    RUN_TEST(test_integrity_bound);
    RUN_TEST(test_run_link_reproduces_milestone_4_rows);
    RUN_TEST(test_crc_polynomial_pattern_is_undetected);
    RUN_TEST(test_single_bit_pattern_is_detected);
    RUN_TEST(test_crc_field_starts_at_byte_43);
    return UNITY_END();
}
