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
// starts in the data bytes and ends in the CRC bytes, and the same frame
// with its CRC stored high byte first misses none.

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

// The first miss the exhaustive check finds, spanning bits 341 to 353 on
// the air. Read in the CRC's bit order, byte 42 then the high byte (44) then
// the low byte (43), it is x^2 times the CRC polynomial 0x11021.
void test_first_miss_is_the_crc_polynomial_in_crc_order(void) {
    uint8_t sent[PACKET_SIZE];
    build_link_frame(0, sent);
    uint8_t errors[PACKET_SIZE];
    memset(errors, 0, sizeof errors);
    errors[42] = 0x04;
    errors[43] = 0x84;
    errors[44] = 0x40;
    TEST_ASSERT_TRUE(pattern_undetected(sent, errors));
    const uint32_t crc_order =
        (static_cast<uint32_t>(errors[42]) << 16) | (static_cast<uint32_t>(errors[44]) << 8) | errors[43];
    TEST_ASSERT_EQUAL_HEX32(0x11021u << 2, crc_order);
}

// The error pattern in the CRC's own bit order: bytes 0 to 42 as sent, then
// the CRC high byte (44), then the low byte (43). Returns the remainder of
// that polynomial divided by the CRC polynomial 0x11021; a CRC misses an
// error exactly when this is 0.
static uint16_t crc_order_remainder(const uint8_t *errors) {
    uint8_t ordered[PACKET_SIZE];
    memcpy(ordered, errors, 43);
    ordered[43] = errors[44];
    ordered[44] = errors[43];
    uint32_t rem = 0;
    for (size_t i = 0; i < PACKET_SIZE * 8; ++i) {
        rem = (rem << 1) | ((ordered[i / 8] >> (7 - i % 8)) & 1u);
        if (rem & 0x10000u) {
            rem ^= 0x11021u;
        }
    }
    return static_cast<uint16_t>(rem);
}

/// True when the pattern, in CRC order, is exactly x^k times 0x11021.
static bool is_shifted_polynomial(const uint8_t *errors) {
    uint8_t ordered[PACKET_SIZE];
    memcpy(ordered, errors, 43);
    ordered[43] = errors[44];
    ordered[44] = errors[43];
    size_t set[8];
    size_t n = 0;
    for (size_t i = 0; i < PACKET_SIZE * 8; ++i) {
        if ((ordered[i / 8] >> (7 - i % 8)) & 1u) {
            if (n == 8) {
                return false;
            }
            set[n++] = i;
        }
    }
    return n == 4 && set[1] - set[0] == 4 && set[2] - set[0] == 11 && set[3] - set[0] == 16;
}

static unsigned g_multiples = 0;
static unsigned g_shifts = 0;

// Undetected means, for a CRC, that the error is a multiple of its
// polynomial in the CRC's bit order. Only some of the misses are the
// polynomial merely shifted; the rest are other multiples of it.
void test_every_little_endian_miss_is_a_multiple_of_the_polynomial(void) {
    build_link_frame(0, g_sent);
    g_multiples = 0;
    g_shifts = 0;
    const ExhaustiveResult r = for_each_short_burst([](const uint8_t *errors, size_t, size_t) {
        const bool missed = pattern_undetected(g_sent, errors);
        if (missed) {
            g_multiples += crc_order_remainder(errors) == 0 ? 1 : 0;
            g_shifts += is_shifted_polynomial(errors) ? 1 : 0;
        }
        return missed;
    });
    TEST_ASSERT_EQUAL_UINT64(12, r.crc_undetected);
    TEST_ASSERT_EQUAL_UINT32(12, g_multiples);
    TEST_ASSERT_EQUAL_UINT32(3, g_shifts);
}

int main(int, char **) {
    UNITY_BEGIN();
    RUN_TEST(test_big_endian_crc_catches_every_short_burst);
    RUN_TEST(test_every_little_endian_miss_straddles_the_crc_boundary);
    RUN_TEST(test_first_miss_is_the_crc_polynomial_in_crc_order);
    RUN_TEST(test_every_little_endian_miss_is_a_multiple_of_the_polynomial);
    return UNITY_END();
}
