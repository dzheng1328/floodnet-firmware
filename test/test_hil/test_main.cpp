#include <math.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>

#include <unity.h>

#include "../support/hil.hpp"
#include "../support/noisy_channel.hpp"
#include "../support/power_experiments.hpp"
#include "../support/power_rig.hpp"

using namespace floodnet;

void setUp(void) {}
void tearDown(void) {}

// Frozen in the milestone 4 design doc, "Frozen scenario table and pass
// criteria". The runner asserts only that each run did what it was told;
// whether a row passes is printed, never asserted.

static const Build kBuilds[] = {Build::Interrupt, Build::DutyCycled};
static const size_t kBuildCount = 2;

static const char *build_name(Build build) {
    return build == Build::Interrupt ? "interrupt" : "duty_cycled";
}

static unsigned g_link_passed = 0;
static unsigned g_integrity_passed = 0;
static unsigned g_transparency_passed[kBuildCount] = {0, 0};
static unsigned g_fault_passed[kBuildCount] = {0, 0};

void test_link_group(void) {
    for (size_t i = 0; i < kBerPointCount; ++i) {
        const double p = kBerPoints[i];
        const LinkResult r = run_link(p, kLinkFrames, kChannelSeed);
        TEST_ASSERT_EQUAL_UINT32(kLinkFrames, r.frames);
        const double q = expected_ratio(p);
        const bool pass = link_pass(r.accepted, r.frames, q);
        g_link_passed += pass ? 1 : 0;
        printf("LINK,%g,%u,%u,%.1f,%.6f,%.6f,%u,%d\n", p, r.frames, r.accepted,
               r.frames * q, static_cast<double>(r.accepted) / r.frames, q, r.undetected,
               pass ? 1 : 0);
    }
}

void test_integrity_group(void) {
    const LinkResult r = run_link(kIntegrityBer, kIntegrityFrames, kChannelSeed);
    TEST_ASSERT_EQUAL_UINT32(kIntegrityFrames, r.frames);
    TEST_ASSERT_TRUE_MESSAGE(r.corrupted > 0, "integrity run corrupted nothing");
    const bool pass = integrity_pass(r.undetected, r.corrupted);
    g_integrity_passed += pass ? 1 : 0;
    printf("INTEGRITY,%g,%u,%u,%u,%.3f,%d\n", kIntegrityBer, r.frames, r.corrupted,
           r.undetected, integrity_bound(r.corrupted), pass ? 1 : 0);
}

static bool same_records(const PowerRig &a, const PowerRig &b) {
    const std::vector<GatewayRecord> &ra = a.records();
    const std::vector<GatewayRecord> &rb = b.records();
    if (ra.size() != rb.size()) {
        return false;
    }
    for (size_t i = 0; i < ra.size(); ++i) {
        if (ra[i].arrival_ms != rb[i].arrival_ms || ra[i].gps_valid != rb[i].gps_valid ||
            ra[i].boot_count != rb[i].boot_count || ra[i].tx_timeouts != rb[i].tx_timeouts) {
            return false;
        }
    }
    return true;
}

void test_transparency_group(void) {
    for (size_t b = 0; b < kBuildCount; ++b) {
        PowerRig plain(kBuilds[b], kFaultRunSleepNA, kFaultRunCapacityNaMs);
        plain.run_until(kHilRunMs);
        NoisyChannel channel(0.0, kChannelSeed);
        PowerRig noisy(kBuilds[b], kFaultRunSleepNA, kFaultRunCapacityNaMs);
        noisy.set_channel(&channel);
        noisy.run_until(kHilRunMs);
        TEST_ASSERT_TRUE_MESSAGE(plain.clock().now_ms() >= kHilRunMs, "plain run ended early");
        TEST_ASSERT_TRUE_MESSAGE(noisy.clock().now_ms() >= kHilRunMs, "noisy run ended early");
        TEST_ASSERT_TRUE_MESSAGE(plain.records().size() > 0, "plain run delivered nothing");
        const bool identical = same_records(plain, noisy);
        g_transparency_passed[b] += identical ? 1 : 0;
        printf("TRANSPARENT,%s,%u,%u,%d,%d\n", build_name(kBuilds[b]),
               static_cast<unsigned>(noisy.records().size()),
               static_cast<unsigned>(plain.records().size()), identical ? 1 : 0,
               identical ? 1 : 0);
    }
}

void test_fault_group(void) {
    for (size_t b = 0; b < kBuildCount; ++b) {
        for (size_t f = 0; f < kFaultCount; ++f) {
            PowerRig rig(kBuilds[b], kFaultRunSleepNA, kFaultRunCapacityNaMs);
            run_experiment_2_fault(rig, kFaults[f]);
            TEST_ASSERT_TRUE_MESSAGE(rig.fault_started(0), "fault never started");
            const bool recovered = !rig.downtime(kFaultRunMs).down_at_end;
            g_fault_passed[b] += recovered ? 1 : 0;
            printf("FAULT,%s,%s,%d,%d\n", build_name(kBuilds[b]), kFaults[f].name,
                   recovered ? 1 : 0, recovered ? 1 : 0);
        }
    }
}

void test_e2e_group(void) {
    for (size_t b = 0; b < kBuildCount; ++b) {
        for (size_t i = 0; i < kBerPointCount; ++i) {
            NoisyChannel channel(kBerPoints[i], kChannelSeed);
            PowerRig rig(kBuilds[b], kFaultRunSleepNA, kFaultRunCapacityNaMs);
            rig.set_channel(&channel);
            rig.run_until(kHilRunMs);
            TEST_ASSERT_TRUE_MESSAGE(rig.clock().now_ms() >= kHilRunMs, "run ended early");
            const uint32_t queued = rig.records_queued();
            TEST_ASSERT_TRUE_MESSAGE(queued > 0, "node queued nothing");
            const unsigned accepted = static_cast<unsigned>(rig.deliveries().size());
            printf("E2E,%s,%g,%u,%u,%.6f\n", build_name(kBuilds[b]), kBerPoints[i], queued,
                   accepted, static_cast<double>(accepted) / queued);
        }
    }
}

void test_summary(void) {
    printf("REG,link,-,%u,%u\n", g_link_passed, static_cast<unsigned>(kBerPointCount));
    printf("REG,integrity,-,%u,1\n", g_integrity_passed);
    unsigned total = g_link_passed + g_integrity_passed;
    for (size_t b = 0; b < kBuildCount; ++b) {
        printf("REG,transparency,%s,%u,1\n", build_name(kBuilds[b]), g_transparency_passed[b]);
        printf("REG,fault,%s,%u,%u\n", build_name(kBuilds[b]), g_fault_passed[b],
               static_cast<unsigned>(kFaultCount));
        total += g_transparency_passed[b] + g_fault_passed[b];
    }
    printf("REG,all,-,%u,21\n", total);
}

int main(int, char **) {
    UNITY_BEGIN();
    RUN_TEST(test_link_group);
    RUN_TEST(test_integrity_group);
    RUN_TEST(test_transparency_group);
    RUN_TEST(test_fault_group);
    RUN_TEST(test_e2e_group);
    RUN_TEST(test_summary);
    return UNITY_END();
}
