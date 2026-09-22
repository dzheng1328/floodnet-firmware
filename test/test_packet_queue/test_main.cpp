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

void test_non_power_of_two_capacity_wraps_correctly(void) {
    // The class documents modulo index arithmetic. A capacity of 3 fails if
    // that is ever "optimised" to a bitmask, which only works for powers of two.
    PacketQueue<3> queue;
    Packet out;

    for (uint32_t i = 0; i < 3; ++i) {
        queue.push(packet_with_seq(i));
    }
    queue.push(packet_with_seq(3));
    TEST_ASSERT_EQUAL_UINT16(1, queue.drops());
    TEST_ASSERT_EQUAL_size_t(3, queue.size());

    // Drop-oldest discarded seq 0; the rest come out in order.
    for (uint32_t expected = 1; expected <= 3; ++expected) {
        TEST_ASSERT_TRUE(queue.pop(&out));
        TEST_ASSERT_EQUAL_UINT32(expected, out.seq);
    }
    TEST_ASSERT_FALSE(queue.pop(&out));

    // With two entries standing, push one and pop one many times, so the head
    // and tail indices cross the capacity boundary at every offset.
    queue.push(packet_with_seq(100));
    queue.push(packet_with_seq(101));
    for (uint32_t i = 102; i < 1102; ++i) {
        queue.push(packet_with_seq(i));
        TEST_ASSERT_TRUE(queue.pop(&out));
        TEST_ASSERT_EQUAL_UINT32(i - 2, out.seq);
    }
    TEST_ASSERT_EQUAL_UINT16(1, queue.drops());

    // Overflow again at an offset that is not zero.
    queue.push(packet_with_seq(2000));
    queue.push(packet_with_seq(2001));
    TEST_ASSERT_EQUAL_UINT16(2, queue.drops());
    const uint32_t expected[] = {1101, 2000, 2001};
    for (size_t i = 0; i < 3; ++i) {
        TEST_ASSERT_TRUE(queue.pop(&out));
        TEST_ASSERT_EQUAL_UINT32(expected[i], out.seq);
    }
    TEST_ASSERT_TRUE(queue.empty());
}

void test_capacity_of_one_keeps_only_the_newest(void) {
    PacketQueue<1> queue;
    Packet out;

    queue.push(packet_with_seq(1));
    TEST_ASSERT_TRUE(queue.pop(&out));
    TEST_ASSERT_EQUAL_UINT32(1, out.seq);
    TEST_ASSERT_TRUE(queue.empty());

    queue.push(packet_with_seq(2));
    queue.push(packet_with_seq(3));
    TEST_ASSERT_EQUAL_UINT16(1, queue.drops());
    TEST_ASSERT_EQUAL_size_t(1, queue.size());
    TEST_ASSERT_TRUE(queue.pop(&out));
    TEST_ASSERT_EQUAL_UINT32(3, out.seq);
    TEST_ASSERT_FALSE(queue.pop(&out));
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
    RUN_TEST(test_non_power_of_two_capacity_wraps_correctly);
    RUN_TEST(test_capacity_of_one_keeps_only_the_newest);
    return UNITY_END();
}
