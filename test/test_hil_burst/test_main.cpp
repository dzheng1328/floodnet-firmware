#include <stddef.h>
#include <stdint.h>
#include <stdio.h>

#include <unity.h>

#include "../support/burst_channel.hpp"
#include "../support/hil.hpp"
#include "../support/noisy_channel.hpp"

using namespace floodnet;

void setUp(void) {}
void tearDown(void) {}

// Frozen in the milestone 6 design doc. The runner asserts only that each
// run did what it was told; whether a row passes is printed, never asserted.

static unsigned g_passed = 0;
static const unsigned kGraded = 3;

void test_burst_link_group(void) {
    for (size_t i = 1; i < kBerPointCount; ++i) {
        const double p = kBerPoints[i];
        for (size_t k = 0; k < kBurstLengthCount; ++k) {
            const double L = kBurstLengths[k];
            BurstChannel channel(p, L, kChannelSeed);
            const LinkResult r = run_link_with(channel, kBurstLinkFrames);
            TEST_ASSERT_EQUAL_UINT32(kBurstLinkFrames, r.frames);
            TEST_ASSERT_EQUAL_UINT64(kBurstLinkFrames, channel.frames());
            printf("BURST_LINK,%g,%g,%u,%u,%.6f,%.6f,%u,%u\n", p, L, r.frames, r.accepted,
                   static_cast<double>(r.accepted) / r.frames, expected_ratio(p), r.corrupted,
                   r.undetected);
        }
    }
}

/// Patterns of span 1 to kMaxBurstSpan lying wholly within the first
/// `n_bits` bits, from the closed form rather than from the enumerator.
static uint64_t bursts_within(uint64_t n_bits) {
    uint64_t count = n_bits;
    for (uint64_t w = 2; w <= kMaxBurstSpan; ++w) {
        count += (n_bits + 1 - w) << (w - 2);
    }
    return count;
}

void test_exhaustive_group(void) {
    uint8_t sent[PACKET_SIZE];
    build_link_frame(0, sent);
    const ExhaustiveResult r = run_exhaustive_bursts(sent);
    const uint64_t data_expected = bursts_within(kCrcFieldFirstBit);
    const uint64_t crc_expected = bursts_within(PACKET_SIZE * 8) - data_expected;
    TEST_ASSERT_EQUAL_UINT64(10813439, data_expected);
    TEST_ASSERT_EQUAL_UINT64(524288, crc_expected);
    TEST_ASSERT_EQUAL_UINT64(data_expected, r.data_patterns);
    TEST_ASSERT_EQUAL_UINT64(crc_expected, r.crc_patterns);
    const bool data_pass = r.data_undetected == 0;
    const bool crc_pass = r.crc_undetected == 0;
    g_passed += (data_pass ? 1 : 0) + (crc_pass ? 1 : 0);
    printf("BURST_EXHAUSTIVE,data,%llu,%llu,%d\n",
           static_cast<unsigned long long>(r.data_patterns),
           static_cast<unsigned long long>(r.data_undetected), data_pass ? 1 : 0);
    printf("BURST_EXHAUSTIVE,crc_field,%llu,%llu,%d\n",
           static_cast<unsigned long long>(r.crc_patterns),
           static_cast<unsigned long long>(r.crc_undetected), crc_pass ? 1 : 0);
}

void test_burst_integrity_group(void) {
    BurstChannel channel(kBurstIntegrityBer, kBurstIntegrityLength, kChannelSeed);
    const LinkResult r = run_link_with(channel, kBurstIntegrityFrames);
    TEST_ASSERT_EQUAL_UINT32(kBurstIntegrityFrames, r.frames);
    TEST_ASSERT_TRUE_MESSAGE(r.corrupted > 0, "integrity run corrupted nothing");
    const bool pass = integrity_pass(r.undetected, r.corrupted);
    g_passed += pass ? 1 : 0;
    printf("BURST_INTEGRITY,%g,%g,%u,%u,%u,%.3f,%d\n", kBurstIntegrityBer,
           kBurstIntegrityLength, r.frames, r.corrupted, r.undetected,
           integrity_bound(r.corrupted), pass ? 1 : 0);
}

void test_summary(void) { printf("BURST_REG,all,-,%u,%u\n", g_passed, kGraded); }

int main(int, char **) {
    UNITY_BEGIN();
    RUN_TEST(test_burst_link_group);
    RUN_TEST(test_exhaustive_group);
    RUN_TEST(test_burst_integrity_group);
    RUN_TEST(test_summary);
    return UNITY_END();
}
