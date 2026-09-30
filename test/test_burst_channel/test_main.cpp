#include <math.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include <unity.h>

#include "../support/burst_channel.hpp"
#include "../support/noisy_channel.hpp"

using namespace floodnet;

void setUp(void) {}
void tearDown(void) {}

static const size_t kLen = 45;
static const size_t kBits = kLen * 8;
static const uint32_t kFrames = 100000;

static void fill(uint8_t *frame) {
    for (size_t i = 0; i < kLen; ++i) {
        frame[i] = static_cast<uint8_t>(i * 37 + 11);
    }
}

// Frozen in the milestone 6 design doc, "Testing": 5% of the target.
static bool within_5_percent(double measured, double target) {
    return fabs(measured - target) <= 0.05 * target;
}

void test_zero_p_changes_nothing_and_draws_nothing(void) {
    BurstChannel ch(0.0, 8.0, kChannelSeed);
    uint8_t frame[kLen];
    uint8_t original[kLen];
    fill(frame);
    memcpy(original, frame, kLen);
    for (int i = 0; i < 1000; ++i) {
        TEST_ASSERT_EQUAL_UINT32(0, ch.corrupt(frame, kLen));
    }
    TEST_ASSERT_EQUAL_MEMORY(original, frame, kLen);
    TEST_ASSERT_EQUAL_UINT64(0, ch.draws());
    TEST_ASSERT_EQUAL_UINT64(1000, ch.frames());
}

void test_average_flip_rate_is_p(void) {
    const double p = 1e-3;
    BurstChannel ch(p, 8.0, kChannelSeed);
    uint8_t frame[kLen];
    uint64_t flips = 0;
    for (uint32_t i = 0; i < kFrames; ++i) {
        fill(frame);
        flips += ch.corrupt(frame, kLen);
    }
    const double rate = static_cast<double>(flips) / (static_cast<double>(kFrames) * kBits);
    TEST_ASSERT_TRUE_MESSAGE(within_5_percent(rate, p), "average flip rate is not p");
}

// Counts flips per bit position by comparing each corrupted frame against
// the original, so the count does not depend on corrupt()'s return value.
static void per_position_rates(double p, double L, double *rates) {
    BurstChannel ch(p, L, kChannelSeed);
    uint8_t original[kLen];
    uint8_t frame[kLen];
    fill(original);
    uint32_t counts[kBits];
    memset(counts, 0, sizeof counts);
    for (uint32_t i = 0; i < kFrames; ++i) {
        memcpy(frame, original, kLen);
        ch.corrupt(frame, kLen);
        for (size_t b = 0; b < kBits; ++b) {
            const uint8_t mask = static_cast<uint8_t>(0x80u >> (b % 8));
            if ((frame[b / 8] ^ original[b / 8]) & mask) {
                ++counts[b];
            }
        }
    }
    for (size_t b = 0; b < kBits; ++b) {
        rates[b] = static_cast<double>(counts[b]) / kFrames;
    }
}

void test_every_bit_position_flips_at_rate_p(void) {
    double rates[kBits];
    per_position_rates(0.25, 4.0, rates);
    for (size_t b = 0; b < kBits; ++b) {
        char msg[64];
        snprintf(msg, sizeof msg, "bit %u flips at %.4f", static_cast<unsigned>(b), rates[b]);
        TEST_ASSERT_TRUE_MESSAGE(within_5_percent(rates[b], 0.25), msg);
    }
}

void test_half_p_makes_every_bit_bad(void) {
    double rates[kBits];
    per_position_rates(0.5, 8.0, rates);
    for (size_t b = 0; b < kBits; ++b) {
        char msg[64];
        snprintf(msg, sizeof msg, "bit %u flips at %.4f", static_cast<unsigned>(b), rates[b]);
        TEST_ASSERT_TRUE_MESSAGE(within_5_percent(rates[b], 0.5), msg);
    }
}

void test_errors_cluster(void) {
    const double p = 1e-2;
    BurstChannel ch(p, 32.0, kChannelSeed);
    uint8_t frame[kLen];
    uint64_t corrupted = 0;
    uint64_t flips = 0;
    for (uint32_t i = 0; i < kFrames; ++i) {
        fill(frame);
        const size_t f = ch.corrupt(frame, kLen);
        if (f > 0) {
            ++corrupted;
            flips += f;
        }
    }
    const double independent_corrupted_share = 1.0 - pow(1.0 - p, static_cast<double>(kBits));
    const double independent_flips_per_corrupted = kBits * p / independent_corrupted_share;
    const double share = static_cast<double>(corrupted) / kFrames;
    const double per_corrupted = static_cast<double>(flips) / static_cast<double>(corrupted);
    TEST_ASSERT_TRUE_MESSAGE(share < independent_corrupted_share,
                             "bursts did not corrupt fewer frames");
    TEST_ASSERT_TRUE_MESSAGE(per_corrupted > independent_flips_per_corrupted,
                             "bursts did not put more flips in each corrupted frame");
}

void test_tiny_p_stays_bounded(void) {
    BurstChannel ch(1e-6, 2.0, kChannelSeed);
    uint8_t frame[kLen];
    uint64_t flips = 0;
    for (uint32_t i = 0; i < kFrames; ++i) {
        fill(frame);
        const size_t f = ch.corrupt(frame, kLen);
        TEST_ASSERT_TRUE_MESSAGE(f <= kBits, "more flips than bits");
        flips += f;
    }
    // Expected 36 flips over 36 000 000 bits; the bound only catches a
    // run length that overflowed into a huge bad run.
    TEST_ASSERT_TRUE_MESSAGE(flips > 0 && flips < 1000, "tiny p flip count out of range");
}

void test_same_seed_gives_same_flips(void) {
    BurstChannel a(1e-2, 16.0, kChannelSeed);
    BurstChannel b(1e-2, 16.0, kChannelSeed);
    uint8_t fa[kLen];
    uint8_t fb[kLen];
    for (int i = 0; i < 10000; ++i) {
        fill(fa);
        fill(fb);
        TEST_ASSERT_EQUAL_UINT32(a.corrupt(fa, kLen), b.corrupt(fb, kLen));
        TEST_ASSERT_EQUAL_MEMORY(fa, fb, kLen);
    }
    TEST_ASSERT_EQUAL_UINT64(a.draws(), b.draws());
}

void test_return_value_counts_flipped_bits(void) {
    BurstChannel ch(1e-2, 8.0, kChannelSeed);
    uint8_t original[kLen];
    uint8_t frame[kLen];
    fill(original);
    for (int i = 0; i < 10000; ++i) {
        memcpy(frame, original, kLen);
        const size_t f = ch.corrupt(frame, kLen);
        size_t diff = 0;
        for (size_t j = 0; j < kLen; ++j) {
            diff += static_cast<size_t>(__builtin_popcount(frame[j] ^ original[j]));
        }
        TEST_ASSERT_EQUAL_UINT32(diff, f);
    }
}

// The good-to-bad probability s = (1/L) * 2p / (1 - 2p) is a probability
// only while p <= L / (2 (L + 1)); beyond that the channel cannot hit p.
void test_parameters_outside_the_domain_are_reported(void) {
    TEST_ASSERT_TRUE(BurstChannel(0.01, 2.0, kChannelSeed).valid());
    TEST_ASSERT_TRUE(BurstChannel(0.25, 4.0, kChannelSeed).valid());
    TEST_ASSERT_TRUE(BurstChannel(0.5, 8.0, kChannelSeed).valid());
    TEST_ASSERT_TRUE(BurstChannel(0.0, 1.0, kChannelSeed).valid());
    TEST_ASSERT_TRUE(BurstChannel(1.0 / 3.0, 2.0, kChannelSeed).valid());
    TEST_ASSERT_FALSE(BurstChannel(0.4, 2.0, kChannelSeed).valid());
    TEST_ASSERT_FALSE(BurstChannel(0.4, 1.0, kChannelSeed).valid());
    TEST_ASSERT_FALSE(BurstChannel(0.6, 8.0, kChannelSeed).valid());
    TEST_ASSERT_FALSE(BurstChannel(-0.1, 8.0, kChannelSeed).valid());
    TEST_ASSERT_FALSE(BurstChannel(0.01, 0.5, kChannelSeed).valid());
}

int main(int, char **) {
    UNITY_BEGIN();
    RUN_TEST(test_zero_p_changes_nothing_and_draws_nothing);
    RUN_TEST(test_average_flip_rate_is_p);
    RUN_TEST(test_every_bit_position_flips_at_rate_p);
    RUN_TEST(test_half_p_makes_every_bit_bad);
    RUN_TEST(test_errors_cluster);
    RUN_TEST(test_tiny_p_stays_bounded);
    RUN_TEST(test_same_seed_gives_same_flips);
    RUN_TEST(test_return_value_counts_flipped_bits);
    RUN_TEST(test_parameters_outside_the_domain_are_reported);
    return UNITY_END();
}
