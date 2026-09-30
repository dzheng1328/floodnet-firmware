#include <math.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include <unity.h>

#include "../support/noisy_channel.hpp"

using namespace floodnet;

void setUp(void) {}
void tearDown(void) {}

static const size_t kLen = 45;
static const uint32_t kFrames = 100000;

static void fill(uint8_t *frame) {
    for (size_t i = 0; i < kLen; ++i) {
        frame[i] = static_cast<uint8_t>(i * 37 + 11);
    }
}

void test_zero_p_changes_nothing_and_draws_nothing(void) {
    NoisyChannel ch(0.0, kChannelSeed);
    uint8_t frame[kLen];
    uint8_t original[kLen];
    fill(frame);
    memcpy(original, frame, kLen);
    for (int i = 0; i < 1000; ++i) {
        TEST_ASSERT_EQUAL_UINT32(0, ch.corrupt(frame, kLen));
    }
    TEST_ASSERT_EQUAL_MEMORY(original, frame, kLen);
    TEST_ASSERT_EQUAL_UINT64(0, ch.draws());
}

void test_p_one_flips_every_bit(void) {
    NoisyChannel ch(1.0, kChannelSeed);
    uint8_t frame[kLen];
    fill(frame);
    uint8_t expected[kLen];
    for (size_t i = 0; i < kLen; ++i) {
        expected[i] = static_cast<uint8_t>(~frame[i]);
    }
    TEST_ASSERT_EQUAL_UINT32(kLen * 8, ch.corrupt(frame, kLen));
    TEST_ASSERT_EQUAL_MEMORY(expected, frame, kLen);
}

static void assert_flip_count_within_3_sigma(double p) {
    NoisyChannel ch(p, kChannelSeed);
    uint8_t frame[kLen];
    uint64_t flips = 0;
    for (uint32_t i = 0; i < kFrames; ++i) {
        fill(frame);
        flips += ch.corrupt(frame, kLen);
    }
    const double n = static_cast<double>(kFrames) * kLen * 8;
    const double mean = n * p;
    const double sd = sqrt(n * p * (1.0 - p));
    TEST_ASSERT_TRUE_MESSAGE(fabs(static_cast<double>(flips) - mean) <= 3.0 * sd,
                             "flip count outside 3 sigma of n p");
}

void test_flip_rate_matches_p(void) { assert_flip_count_within_3_sigma(1e-3); }

void test_tiny_p_flip_count_matches_expectation(void) { assert_flip_count_within_3_sigma(1e-6); }

/// At p = 0.5 an off-by-one in the gap or a wrong bit mask moves the rate far
/// outside the band, which the low-p tests above cannot see.
void test_every_bit_position_flips_at_rate_p(void) {
    const double p = 0.5;
    const uint32_t frames = 10000;
    NoisyChannel ch(p, kChannelSeed);
    uint8_t frame[kLen];
    uint8_t original[kLen];
    uint32_t per_position[8] = {0, 0, 0, 0, 0, 0, 0, 0};
    for (uint32_t i = 0; i < frames; ++i) {
        fill(frame);
        memcpy(original, frame, kLen);
        ch.corrupt(frame, kLen);
        for (size_t b = 0; b < kLen; ++b) {
            const uint8_t changed = static_cast<uint8_t>(frame[b] ^ original[b]);
            for (int bit = 0; bit < 8; ++bit) {
                per_position[bit] += (changed >> bit) & 1u;
            }
        }
    }
    const double n = static_cast<double>(frames) * kLen;
    const double mean = n * p;
    const double sd = sqrt(n * p * (1.0 - p));
    for (int bit = 0; bit < 8; ++bit) {
        TEST_ASSERT_TRUE_MESSAGE(fabs(per_position[bit] - mean) <= 3.0 * sd,
                                 "a bit position flips outside 3 sigma of p");
    }
}

void test_frames_counts_every_call_including_zero_p(void) {
    NoisyChannel clean(0.0, kChannelSeed);
    NoisyChannel noisy(1e-2, kChannelSeed);
    uint8_t frame[kLen];
    for (int i = 0; i < 37; ++i) {
        fill(frame);
        clean.corrupt(frame, kLen);
        noisy.corrupt(frame, kLen);
    }
    TEST_ASSERT_EQUAL_UINT64(37, clean.frames());
    TEST_ASSERT_EQUAL_UINT64(37, noisy.frames());
    TEST_ASSERT_EQUAL_UINT64(0, clean.draws());
}

void test_same_seed_reproduces_the_same_corruption(void) {
    NoisyChannel a(1e-2, kChannelSeed);
    NoisyChannel b(1e-2, kChannelSeed);
    uint8_t fa[kLen];
    uint8_t fb[kLen];
    for (int i = 0; i < 1000; ++i) {
        fill(fa);
        fill(fb);
        TEST_ASSERT_EQUAL_UINT32(a.corrupt(fa, kLen), b.corrupt(fb, kLen));
        TEST_ASSERT_EQUAL_MEMORY(fa, fb, kLen);
    }
}

int main(int, char **) {
    UNITY_BEGIN();
    RUN_TEST(test_zero_p_changes_nothing_and_draws_nothing);
    RUN_TEST(test_p_one_flips_every_bit);
    RUN_TEST(test_flip_rate_matches_p);
    RUN_TEST(test_tiny_p_flip_count_matches_expectation);
    RUN_TEST(test_every_bit_position_flips_at_rate_p);
    RUN_TEST(test_frames_counts_every_call_including_zero_p);
    RUN_TEST(test_same_seed_reproduces_the_same_corruption);
    return UNITY_END();
}
