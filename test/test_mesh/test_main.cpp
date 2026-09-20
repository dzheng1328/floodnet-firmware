#include <unity.h>
#include <floodnet/mesh.hpp>

using namespace floodnet;

void setUp(void) {}
void tearDown(void) {}

static Packet packet(uint16_t node_id, uint32_t seq, uint8_t ttl) {
    Packet p;
    p.node_id = node_id;
    p.seq = seq;
    p.ttl = ttl;
    return p;
}

void test_first_sighting_relays(void) {
    DedupTable table;
    TEST_ASSERT_TRUE(should_relay(table, packet(1, 100, 3)));
}

void test_duplicate_does_not_relay(void) {
    DedupTable table;
    should_relay(table, packet(1, 100, 3));
    TEST_ASSERT_FALSE(should_relay(table, packet(1, 100, 3)));
}

void test_same_sequence_from_another_node_relays(void) {
    DedupTable table;
    should_relay(table, packet(1, 100, 3));
    TEST_ASSERT_TRUE(should_relay(table, packet(2, 100, 3)));
}

void test_expired_packet_does_not_relay(void) {
    DedupTable table;
    TEST_ASSERT_FALSE(should_relay(table, packet(1, 100, 0)));
}

void test_expired_packet_is_not_recorded(void) {
    // A TTL check that consumed a table slot would let an expired sighting
    // suppress the same packet arriving later by a shorter path.
    DedupTable table;
    should_relay(table, packet(1, 100, 0));
    TEST_ASSERT_TRUE(should_relay(table, packet(1, 100, 3)));
}

void test_table_evicts_oldest_entry(void) {
    DedupTable table;
    for (uint32_t seq = 0; seq < DEDUP_CAPACITY; ++seq) {
        should_relay(table, packet(1, seq, 3));
    }
    // One more sighting evicts sequence 0.
    should_relay(table, packet(1, DEDUP_CAPACITY, 3));
    TEST_ASSERT_TRUE(should_relay(table, packet(1, 0, 3)));
}

void test_prepare_relay_decrements_ttl(void) {
    Packet p = packet(1, 100, 3);
    TEST_ASSERT_TRUE(prepare_relay(&p));
    TEST_ASSERT_EQUAL_UINT8(2, p.ttl);
}

void test_prepare_relay_fails_on_last_hop(void) {
    Packet p = packet(1, 100, 1);
    TEST_ASSERT_FALSE(prepare_relay(&p));
    TEST_ASSERT_EQUAL_UINT8(0, p.ttl);
}

int main(int, char **) {
    UNITY_BEGIN();
    RUN_TEST(test_first_sighting_relays);
    RUN_TEST(test_duplicate_does_not_relay);
    RUN_TEST(test_same_sequence_from_another_node_relays);
    RUN_TEST(test_expired_packet_does_not_relay);
    RUN_TEST(test_expired_packet_is_not_recorded);
    RUN_TEST(test_table_evicts_oldest_entry);
    RUN_TEST(test_prepare_relay_decrements_ttl);
    RUN_TEST(test_prepare_relay_fails_on_last_hop);
    return UNITY_END();
}
