#include <unity.h>

#include <floodnet/node_state.hpp>

using namespace floodnet;

void setUp(void) {}
void tearDown(void) {}

static NodeStateMachine booted(uint32_t now) {
    NodeStateMachine m;
    m.boot(now);
    m.handle(NodeEvent::PeripheralsReady, now);
    return m;
}

/// Ticks a sleeping machine at each deadline it asks for until it wakes into
/// ACQUIRE, and returns the wake time. `a` is the action that entered SLEEP.
static uint32_t sleep_until_acquire(NodeStateMachine *m, NodeActions a) {
    uint32_t now = a.sleep_until_ms;
    for (int guard = 0; guard < 100 && m->state() == NodeState::Sleep; ++guard) {
        now = a.sleep_until_ms;
        a = m->handle(NodeEvent::Tick, now);
    }
    return now;
}

/// One full cycle from ACQUIRE: fix at +1000, transmit outcome at +4000.
static NodeActions cycle(NodeStateMachine *m, uint32_t start, NodeEvent fix, NodeEvent tx) {
    m->handle(fix, start + 1000);
    return m->handle(tx, start + 4000);
}

void test_boot_powers_everything_and_kicks(void) {
    NodeStateMachine m;
    NodeActions a = m.boot(0);
    TEST_ASSERT_TRUE(m.state() == NodeState::Boot);
    TEST_ASSERT_TRUE(a.transitioned);
    TEST_ASSERT_TRUE(a.kick_watchdog);
    TEST_ASSERT_TRUE(a.power.gps && a.power.imu && a.power.radio);
}

void test_ready_enters_acquire_with_gps_and_imu(void) {
    NodeStateMachine m;
    m.boot(0);
    NodeActions a = m.handle(NodeEvent::PeripheralsReady, 0);
    TEST_ASSERT_TRUE(m.state() == NodeState::Acquire);
    TEST_ASSERT_TRUE(a.power.gps);
    TEST_ASSERT_TRUE(a.power.imu);
    TEST_ASSERT_FALSE(a.power.radio);
    TEST_ASSERT_EQUAL_UINT32(300000, m.next_slot_ms());
}

void test_fix_moves_to_transmit_and_powers_down_sensors(void) {
    NodeStateMachine m = booted(0);
    NodeActions a = m.handle(NodeEvent::FixQueued, 1200);
    TEST_ASSERT_TRUE(m.state() == NodeState::Transmit);
    TEST_ASSERT_FALSE(a.power.gps);
    TEST_ASSERT_FALSE(a.power.imu);
    TEST_ASSERT_TRUE(a.power.radio);
    TEST_ASSERT_FALSE(a.queue_heartbeat);
}

void test_tick_without_a_transition_does_not_kick(void) {
    NodeStateMachine m = booted(0);
    NodeActions a = m.handle(NodeEvent::Tick, 59999);
    TEST_ASSERT_TRUE(m.state() == NodeState::Acquire);
    TEST_ASSERT_FALSE(a.transitioned);
    TEST_ASSERT_FALSE(a.kick_watchdog);
    TEST_ASSERT_TRUE(a.power.gps);  // a held state still reports its plan
}

void test_acquire_timeout_sends_a_heartbeat(void) {
    NodeStateMachine m = booted(0);
    NodeActions a = m.handle(NodeEvent::Tick, 60000);
    TEST_ASSERT_TRUE(m.state() == NodeState::Transmit);
    TEST_ASSERT_TRUE(a.queue_heartbeat);
    TEST_ASSERT_TRUE(a.kick_watchdog);
}

void test_success_sleeps_until_the_first_chunk(void) {
    NodeStateMachine m = booted(0);
    NodeActions a = cycle(&m, 0, NodeEvent::FixQueued, NodeEvent::TxSucceeded);
    TEST_ASSERT_TRUE(m.state() == NodeState::Sleep);
    TEST_ASSERT_EQUAL_UINT32(64000, a.sleep_until_ms);
    TEST_ASSERT_FALSE(a.power.gps || a.power.imu || a.power.radio);
}

void test_sleep_chunks_renew_and_kick_until_the_slot(void) {
    NodeStateMachine m = booted(0);
    NodeActions a = cycle(&m, 0, NodeEvent::FixQueued, NodeEvent::TxSucceeded);
    const uint32_t expected[] = {124000, 184000, 244000, 300000};
    for (size_t i = 0; i < 4; ++i) {
        a = m.handle(NodeEvent::Tick, a.sleep_until_ms);
        TEST_ASSERT_TRUE(m.state() == NodeState::Sleep);
        TEST_ASSERT_TRUE(a.kick_watchdog);
        TEST_ASSERT_EQUAL_UINT32(expected[i], a.sleep_until_ms);
    }
    a = m.handle(NodeEvent::Tick, 300000);
    TEST_ASSERT_TRUE(m.state() == NodeState::Acquire);
    TEST_ASSERT_EQUAL_UINT32(600000, m.next_slot_ms());
}

void test_early_wake_changes_nothing(void) {
    NodeStateMachine m = booted(0);
    cycle(&m, 0, NodeEvent::FixQueued, NodeEvent::TxSucceeded);
    NodeActions a = m.handle(NodeEvent::Tick, 30000);
    TEST_ASSERT_FALSE(a.transitioned);
    TEST_ASSERT_EQUAL_UINT32(64000, a.sleep_until_ms);
}

void test_schedule_is_anchored_to_the_slot_not_the_work(void) {
    NodeStateMachine m = booted(0);
    m.handle(NodeEvent::Tick, 60000);                                   // slow: timed out
    NodeActions a = m.handle(NodeEvent::TxSucceeded, 63100);
    TEST_ASSERT_EQUAL_UINT32(300000, sleep_until_acquire(&m, a));       // not 363100
    TEST_ASSERT_EQUAL_UINT32(600000, m.next_slot_ms());
}

void test_missed_slots_are_skipped_not_burst(void) {
    NodeStateMachine m = booted(0);
    cycle(&m, 0, NodeEvent::FixQueued, NodeEvent::TxSucceeded);
    m.handle(NodeEvent::Tick, 950000);  // woke far too late
    TEST_ASSERT_TRUE(m.state() == NodeState::Acquire);
    TEST_ASSERT_EQUAL_UINT32(1200000, m.next_slot_ms());
}

void test_three_failed_transmits_power_cycle_the_radio(void) {
    NodeStateMachine m = booted(0);
    uint32_t start = 0;
    for (int i = 0; i < 2; ++i) {
        NodeActions a = cycle(&m, start, NodeEvent::FixQueued, NodeEvent::TxFailed);
        TEST_ASSERT_FALSE(a.power_cycle_radio);
        start = sleep_until_acquire(&m, a);
    }
    NodeActions a = cycle(&m, start, NodeEvent::FixQueued, NodeEvent::TxFailed);
    TEST_ASSERT_TRUE(a.power_cycle_radio);
    TEST_ASSERT_EQUAL_UINT8(0, m.radio_failures());
}

void test_success_resets_the_radio_failure_count(void) {
    NodeStateMachine m = booted(0);
    const NodeEvent outcomes[] = {NodeEvent::TxFailed, NodeEvent::TxFailed,
                                  NodeEvent::TxSucceeded, NodeEvent::TxFailed};
    uint32_t start = 0;
    for (size_t i = 0; i < 4; ++i) {
        NodeActions a = cycle(&m, start, NodeEvent::FixQueued, outcomes[i]);
        TEST_ASSERT_FALSE(a.power_cycle_radio);
        start = sleep_until_acquire(&m, a);
    }
    TEST_ASSERT_EQUAL_UINT8(1, m.radio_failures());
}

void test_transmit_timeout_counts_as_a_failure(void) {
    NodeStateMachine m = booted(0);
    m.handle(NodeEvent::FixQueued, 1000);
    NodeActions a = m.handle(NodeEvent::Tick, 10999);
    TEST_ASSERT_TRUE(m.state() == NodeState::Transmit);
    TEST_ASSERT_FALSE(a.transitioned);
    m.handle(NodeEvent::Tick, 11000);
    TEST_ASSERT_TRUE(m.state() == NodeState::Sleep);
    TEST_ASSERT_EQUAL_UINT8(1, m.radio_failures());
}

void test_three_wakes_without_imu_power_cycle_the_imu(void) {
    NodeStateMachine m = booted(0);
    uint32_t start = 0;
    for (int i = 0; i < 2; ++i) {
        NodeActions a = m.handle(NodeEvent::FixQueuedNoImu, start + 1000);
        TEST_ASSERT_FALSE(a.power_cycle_imu);
        start = sleep_until_acquire(&m, m.handle(NodeEvent::TxSucceeded, start + 4000));
    }
    NodeActions a = m.handle(NodeEvent::FixQueuedNoImu, start + 1000);
    TEST_ASSERT_TRUE(a.power_cycle_imu);
    TEST_ASSERT_EQUAL_UINT8(0, m.imu_failures());
}

void test_heartbeat_does_not_count_against_the_imu(void) {
    NodeStateMachine m = booted(0);
    m.handle(NodeEvent::FixQueuedNoImu, 1000);
    NodeActions a = m.handle(NodeEvent::TxSucceeded, 4000);
    const uint32_t start = sleep_until_acquire(&m, a);
    m.handle(NodeEvent::Tick, start + 60000);  // no fix at all: heartbeat
    TEST_ASSERT_EQUAL_UINT8(1, m.imu_failures());
}

void test_schedule_survives_millis_wrap(void) {
    const uint32_t t0 = 0xFFFF0000u;  // about 65 s before millis() wraps
    NodeStateMachine m = booted(t0);
    NodeActions a = cycle(&m, t0, NodeEvent::FixQueued, NodeEvent::TxSucceeded);
    TEST_ASSERT_EQUAL_UINT32(t0 + 64000u, a.sleep_until_ms);  // the slot below lies past the wrap
    TEST_ASSERT_EQUAL_UINT32(t0 + 300000u, sleep_until_acquire(&m, a));
    TEST_ASSERT_EQUAL_UINT32(t0 + 600000u, m.next_slot_ms());
}

void test_unrelated_events_are_ignored(void) {
    NodeStateMachine m = booted(0);
    NodeActions a = m.handle(NodeEvent::TxSucceeded, 500);
    TEST_ASSERT_TRUE(m.state() == NodeState::Acquire);
    TEST_ASSERT_FALSE(a.transitioned);

    cycle(&m, 0, NodeEvent::FixQueued, NodeEvent::TxSucceeded);
    a = m.handle(NodeEvent::FixQueued, 5000);
    TEST_ASSERT_TRUE(m.state() == NodeState::Sleep);
    TEST_ASSERT_FALSE(a.transitioned);
}

int main(int, char **) {
    UNITY_BEGIN();
    RUN_TEST(test_boot_powers_everything_and_kicks);
    RUN_TEST(test_ready_enters_acquire_with_gps_and_imu);
    RUN_TEST(test_fix_moves_to_transmit_and_powers_down_sensors);
    RUN_TEST(test_tick_without_a_transition_does_not_kick);
    RUN_TEST(test_acquire_timeout_sends_a_heartbeat);
    RUN_TEST(test_success_sleeps_until_the_first_chunk);
    RUN_TEST(test_sleep_chunks_renew_and_kick_until_the_slot);
    RUN_TEST(test_early_wake_changes_nothing);
    RUN_TEST(test_schedule_is_anchored_to_the_slot_not_the_work);
    RUN_TEST(test_missed_slots_are_skipped_not_burst);
    RUN_TEST(test_three_failed_transmits_power_cycle_the_radio);
    RUN_TEST(test_success_resets_the_radio_failure_count);
    RUN_TEST(test_transmit_timeout_counts_as_a_failure);
    RUN_TEST(test_three_wakes_without_imu_power_cycle_the_imu);
    RUN_TEST(test_heartbeat_does_not_count_against_the_imu);
    RUN_TEST(test_schedule_survives_millis_wrap);
    RUN_TEST(test_unrelated_events_are_ignored);
    return UNITY_END();
}
