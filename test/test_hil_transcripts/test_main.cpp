#include <errno.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/stat.h>

#include <string>

#include <unity.h>

#include <floodnet/hal/tx_listener.hpp>

#include "../support/noisy_channel.hpp"
#include "../support/power_experiments.hpp"
#include "../support/power_rig.hpp"

using namespace floodnet;

void setUp(void) {}
void tearDown(void) {}

// Frozen in the milestone 5 design doc, "Frozen scenarios".
struct Scenario {
    const char *name;
    Build build;
    double p;
    bool has_fault;
    FaultKind fault;
    uint32_t run_ms;
};

static const Scenario kScenarios[] = {
    {"interrupt_p0", Build::Interrupt, 0.0, false, FaultKind::Hang, kHourMs},
    {"interrupt_p1e-3", Build::Interrupt, 1e-3, false, FaultKind::Hang, kHourMs},
    {"interrupt_p1e-2", Build::Interrupt, 1e-2, false, FaultKind::Hang, kHourMs},
    {"duty_cycled_p0", Build::DutyCycled, 0.0, false, FaultKind::Hang, 24 * kHourMs},
    {"duty_cycled_p1e-3", Build::DutyCycled, 1e-3, false, FaultKind::Hang, 24 * kHourMs},
    {"duty_cycled_p1e-2", Build::DutyCycled, 1e-2, false, FaultKind::Hang, 24 * kHourMs},
    {"duty_cycled_lost_completion", Build::DutyCycled, 0.0, true, FaultKind::LostCompletion,
     24 * kHourMs},
    {"duty_cycled_hang", Build::DutyCycled, 0.0, true, FaultKind::Hang, 24 * kHourMs},
};
static const size_t kScenarioCount = sizeof(kScenarios) / sizeof(kScenarios[0]);

static std::string out_root() {
    const char *env = getenv("FLOODNET_TRANSCRIPT_DIR");
    return env != nullptr ? env : "build/hil-transcripts";
}

/// mkdir -p.
static bool make_dirs(const std::string &path) {
    for (size_t i = 1; i <= path.size(); ++i) {
        if (i == path.size() || path[i] == '/') {
            const std::string part = path.substr(0, i);
            if (mkdir(part.c_str(), 0755) != 0 && errno != EEXIST) {
                return false;
            }
        }
    }
    return true;
}

/// Writes TX lines exactly as the board's SerialTxListener prints them,
/// prefixed with the rig's clock.
struct FileTxLog : public ITxListener {
    FILE *file;
    size_t lines;
    explicit FileTxLog(FILE *f) : file(f), lines(0) {}
    void on_tx_end(uint16_t node_id, uint16_t boot_count, uint32_t seq, uint32_t now_ms,
                   bool completed) override {
        fprintf(file, "%lu TX,%u,%u,%lu,%lu,%s\n", static_cast<unsigned long>(now_ms),
                static_cast<unsigned>(node_id), static_cast<unsigned>(boot_count),
                static_cast<unsigned long>(seq), static_cast<unsigned long>(now_ms),
                completed ? "ok" : "timeout");
        ++lines;
    }
};

static void write_scenario(const Scenario &s) {
    const std::string dir = out_root() + "/" + s.name;
    TEST_ASSERT_TRUE_MESSAGE(make_dirs(dir), "cannot create the transcript directory");
    FILE *node = fopen((dir + "/node.log").c_str(), "w");
    FILE *gateway = fopen((dir + "/gateway.log").c_str(), "w");
    FILE *expected = fopen((dir + "/expected.json").c_str(), "w");
    TEST_ASSERT_NOT_NULL(node);
    TEST_ASSERT_NOT_NULL(gateway);
    TEST_ASSERT_NOT_NULL(expected);

    NoisyChannel channel(s.p, kChannelSeed);
    PowerRig rig(s.build, kFaultRunSleepNA, kFaultRunCapacityNaMs);
    rig.set_channel(&channel);
    FileTxLog tx_log(node);
    rig.set_tx_listener(&tx_log);
    size_t gateway_lines = 0;
    rig.set_gateway_log([gateway, &gateway_lines](uint32_t t_ms, const char *line) {
        fprintf(gateway, "%lu %s\n", static_cast<unsigned long>(t_ms), line);
        ++gateway_lines;
    });
    if (s.has_fault) {
        rig.add_fault(s.fault, kFaultAtMs, 0);
    }
    rig.run_until_quiet(s.run_ms);

    TEST_ASSERT_TRUE_MESSAGE(rig.clock().now_ms() >= s.run_ms, "run ended early");
    TEST_ASSERT_FALSE_MESSAGE(rig.tx_in_flight(), "run ended with a transmit in flight");
    TEST_ASSERT_TRUE_MESSAGE(tx_log.lines > 0, "node transmitted nothing");
    TEST_ASSERT_EQUAL_UINT32(rig.deliveries().size() + rig.decode_failures(), gateway_lines);
    if (s.has_fault) {
        TEST_ASSERT_TRUE_MESSAGE(rig.fault_started(0), "fault never started");
    }

    const size_t tx_ok = rig.radio().sent_count() - rig.tx_orphaned();
    fprintf(expected,
            "{\"scenario\": \"%s\", \"build\": \"%s\", \"p\": %g, \"tx_ok\": %lu, "
            "\"tx_timeout\": %lu, \"tx_orphaned\": %lu, \"rec_lines\": %lu, "
            "\"unmatched_rec\": %lu, \"decode_errors\": %lu}\n",
            s.name, s.build == Build::Interrupt ? "interrupt" : "duty_cycled", s.p,
            static_cast<unsigned long>(tx_ok), static_cast<unsigned long>(rig.tx_timeouts_total()),
            static_cast<unsigned long>(rig.tx_orphaned()),
            static_cast<unsigned long>(rig.deliveries().size()),
            static_cast<unsigned long>(rig.unmatched_accepted()),
            static_cast<unsigned long>(rig.decode_failures()));
    fclose(node);
    fclose(gateway);
    fclose(expected);
    printf("TRANSCRIPT,%s,%lu,%lu\n", s.name, static_cast<unsigned long>(tx_log.lines),
           static_cast<unsigned long>(gateway_lines));
}

void test_write_every_scenario(void) {
    for (size_t i = 0; i < kScenarioCount; ++i) {
        write_scenario(kScenarios[i]);
    }
}

int main(int, char **) {
    UNITY_BEGIN();
    RUN_TEST(test_write_every_scenario);
    return UNITY_END();
}
