#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include <unity.h>

#include <floodnet/packet.hpp>

#include "../support/hil.hpp"

using namespace floodnet;

void setUp(void) {}
void tearDown(void) {}

// Why the milestone 6 crc_field row fails. crc16_ccitt() runs most
// significant bit first, but put_u16() stores the CRC low byte first, so on
// the air the two CRC bytes are swapped relative to the order the burst
// guarantee assumes. These tests pin that cause: every missed short burst
// straddles the data/CRC boundary, and the same frame with its CRC stored
// high byte first misses none.

static uint8_t g_sent[PACKET_SIZE];

/// The frame as the wire format would carry it with the CRC high byte first.
static void to_big_endian_crc(uint8_t *frame) {
    const uint8_t low = frame[43];
    frame[43] = frame[44];
    frame[44] = low;
}

static bool big_endian_crc_accepts(const uint8_t *frame) {
    const uint16_t stored = static_cast<uint16_t>((frame[43] << 8) | frame[44]);
    return crc16_ccitt(frame, 43) == stored;
}

void test_big_endian_crc_catches_every_short_burst(void) {
    uint8_t sent[PACKET_SIZE];
    build_link_frame(0, sent);
    to_big_endian_crc(sent);
    TEST_ASSERT_TRUE(big_endian_crc_accepts(sent));
    const ExhaustiveResult r = for_each_short_burst(
        [&sent](const uint8_t *errors, size_t, size_t) {
            uint8_t received[PACKET_SIZE];
            for (size_t i = 0; i < PACKET_SIZE; ++i) {
                received[i] = static_cast<uint8_t>(sent[i] ^ errors[i]);
            }
            return big_endian_crc_accepts(received);
        });
    TEST_ASSERT_EQUAL_UINT64(10813439, r.data_patterns);
    TEST_ASSERT_EQUAL_UINT64(524288, r.crc_patterns);
    TEST_ASSERT_EQUAL_UINT64(0, r.data_undetected);
    TEST_ASSERT_EQUAL_UINT64(0, r.crc_undetected);
}

static unsigned g_straddling = 0;

void test_every_little_endian_miss_straddles_the_crc_boundary(void) {
    build_link_frame(0, g_sent);
    g_straddling = 0;
    const ExhaustiveResult r = for_each_short_burst(
        [](const uint8_t *errors, size_t first, size_t last) {
            const bool missed = pattern_undetected(g_sent, errors);
            if (missed && first < kCrcFieldFirstBit && last >= kCrcFieldFirstBit) {
                ++g_straddling;
            }
            return missed;
        });
    TEST_ASSERT_EQUAL_UINT64(0, r.data_undetected);
    TEST_ASSERT_TRUE_MESSAGE(r.crc_undetected > 0, "the little-endian CRC missed nothing");
    TEST_ASSERT_EQUAL_UINT64(r.crc_undetected, g_straddling);
}

int main(int, char **) {
    UNITY_BEGIN();
    RUN_TEST(test_big_endian_crc_catches_every_short_burst);
    RUN_TEST(test_every_little_endian_miss_straddles_the_crc_boundary);
    return UNITY_END();
}
