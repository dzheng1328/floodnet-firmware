#include <unity.h>

#include <floodnet/packet_queue.hpp>

using namespace floodnet;

void setUp(void) {}
void tearDown(void) {}

static Packet packet_with_seq(uint32_t seq) {
    Packet p;
    p.node_id = 7;
    p.seq = seq;
    p.ttl = 3;
    return p;
}

void test_push_then_pop_returns_the_packet(void) {
    PacketQueue<4> queue;
    Packet out;

    TEST_ASSERT_TRUE(queue.empty());
    queue.push(packet_with_seq(11));
    TEST_ASSERT_FALSE(queue.empty());
    TEST_ASSERT_TRUE(queue.pop(&out));
    TEST_ASSERT_EQUAL_UINT32(11, out.seq);
    TEST_ASSERT_TRUE(queue.empty());
}

void test_pop_on_empty_queue_reports_failure(void) {
    PacketQueue<4> queue;
    Packet out;

    TEST_ASSERT_FALSE(queue.pop(&out));
}

void test_order_is_first_in_first_out(void) {
    PacketQueue<4> queue;
    Packet out;

    queue.push(packet_with_seq(1));
    queue.push(packet_with_seq(2));

    TEST_ASSERT_TRUE(queue.pop(&out));
    TEST_ASSERT_EQUAL_UINT32(1, out.seq);
    TEST_ASSERT_TRUE(queue.pop(&out));
    TEST_ASSERT_EQUAL_UINT32(2, out.seq);
}

void test_full_queue_drops_the_oldest_and_counts_it(void) {
    // The OPPOSITE policy from RingBuffer, and deliberately so. This queue is
    // produced and consumed in main context, so it is free to choose, and a
    // receiver would rather have the node's current position than a position
    // from several seconds ago. See the milestone 2 design doc,
    // "Buffer capacities".
    PacketQueue<4> queue;
    Packet out;

    for (uint32_t i = 0; i < 4; ++i) {
        queue.push(packet_with_seq(i));
    }
    TEST_ASSERT_EQUAL_UINT16(0, queue.drops());

    queue.push(packet_with_seq(99));
    TEST_ASSERT_EQUAL_UINT16(1, queue.drops());
    TEST_ASSERT_EQUAL_size_t(4, queue.size());

    // seq 0 is the one that went. The newest survived.
    TEST_ASSERT_TRUE(queue.pop(&out));
    TEST_ASSERT_EQUAL_UINT32(1, out.seq);
    TEST_ASSERT_TRUE(queue.pop(&out));
    TEST_ASSERT_EQUAL_UINT32(2, out.seq);
    TEST_ASSERT_TRUE(queue.pop(&out));
    TEST_ASSERT_EQUAL_UINT32(3, out.seq);
    TEST_ASSERT_TRUE(queue.pop(&out));
    TEST_ASSERT_EQUAL_UINT32(99, out.seq);
    TEST_ASSERT_FALSE(queue.pop(&out));
}

void test_indices_survive_wrapping_many_times(void) {
    PacketQueue<4> queue;
    Packet out;

    for (uint32_t i = 0; i < 1000; ++i) {
        queue.push(packet_with_seq(i));
        TEST_ASSERT_TRUE(queue.pop(&out));
        TEST_ASSERT_EQUAL_UINT32(i, out.seq);
    }
    TEST_ASSERT_TRUE(queue.empty());
    TEST_ASSERT_EQUAL_UINT16(0, queue.drops());
}

void test_high_water_records_the_deepest_occupancy(void) {
    PacketQueue<8> queue;
    Packet out;

    for (uint32_t i = 0; i < 6; ++i) {
        queue.push(packet_with_seq(i));
    }
    for (uint32_t i = 0; i < 6; ++i) {
        TEST_ASSERT_TRUE(queue.pop(&out));
    }

    TEST_ASSERT_EQUAL_size_t(6, queue.high_water());
}

void test_drop_counter_saturates_instead_of_wrapping(void) {
    PacketQueue<2> queue;

    for (uint32_t i = 0; i < 70000; ++i) {
        queue.push(packet_with_seq(i));
    }
    TEST_ASSERT_EQUAL_UINT16(0xFFFF, queue.drops());
}

int main(int, char **) {
    UNITY_BEGIN();
    RUN_TEST(test_push_then_pop_returns_the_packet);
    RUN_TEST(test_pop_on_empty_queue_reports_failure);
    RUN_TEST(test_order_is_first_in_first_out);
    RUN_TEST(test_full_queue_drops_the_oldest_and_counts_it);
    RUN_TEST(test_indices_survive_wrapping_many_times);
    RUN_TEST(test_high_water_records_the_deepest_occupancy);
    RUN_TEST(test_drop_counter_saturates_instead_of_wrapping);
    return UNITY_END();
}
