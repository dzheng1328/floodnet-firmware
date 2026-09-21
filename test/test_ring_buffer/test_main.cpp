#include <stdint.h>
#include <unity.h>

#include <floodnet/ring_buffer.hpp>

using namespace floodnet;

void setUp(void) {}
void tearDown(void) {}

void test_push_then_pop_returns_the_value(void) {
    RingBuffer<int, 4> buffer;
    int value = 0;

    TEST_ASSERT_TRUE(buffer.empty());
    TEST_ASSERT_TRUE(buffer.push(42));
    TEST_ASSERT_FALSE(buffer.empty());
    TEST_ASSERT_TRUE(buffer.pop(&value));
    TEST_ASSERT_EQUAL_INT(42, value);
    TEST_ASSERT_TRUE(buffer.empty());
}

void test_pop_on_empty_buffer_reports_failure(void) {
    RingBuffer<int, 4> buffer;
    int value = 99;

    TEST_ASSERT_FALSE(buffer.pop(&value));
    TEST_ASSERT_EQUAL_INT(99, value);  // untouched
}

void test_order_is_first_in_first_out(void) {
    RingBuffer<int, 4> buffer;
    int value = 0;

    TEST_ASSERT_TRUE(buffer.push(1));
    TEST_ASSERT_TRUE(buffer.push(2));
    TEST_ASSERT_TRUE(buffer.push(3));

    TEST_ASSERT_TRUE(buffer.pop(&value));
    TEST_ASSERT_EQUAL_INT(1, value);
    TEST_ASSERT_TRUE(buffer.pop(&value));
    TEST_ASSERT_EQUAL_INT(2, value);
    TEST_ASSERT_TRUE(buffer.pop(&value));
    TEST_ASSERT_EQUAL_INT(3, value);
}

void test_full_buffer_drops_the_newest_and_counts_it(void) {
    // The policy is drop-NEWEST, not drop-oldest. A producer running in an
    // interrupt handler cannot advance the consumer's index, which is what
    // discarding the oldest element would require. See the milestone 2 design
    // doc, "Corrections to the parent spec".
    RingBuffer<int, 4> buffer;
    int value = 0;

    for (int i = 0; i < 4; ++i) {
        TEST_ASSERT_TRUE(buffer.push(i));
    }
    TEST_ASSERT_EQUAL_size_t(4, buffer.size());
    TEST_ASSERT_EQUAL_UINT16(0, buffer.drops());

    TEST_ASSERT_FALSE(buffer.push(99));
    TEST_ASSERT_EQUAL_UINT16(1, buffer.drops());

    // The four originals survived; 99 is the one that went.
    for (int i = 0; i < 4; ++i) {
        TEST_ASSERT_TRUE(buffer.pop(&value));
        TEST_ASSERT_EQUAL_INT(i, value);
    }
    TEST_ASSERT_FALSE(buffer.pop(&value));
}

void test_indices_survive_wrapping_many_times(void) {
    RingBuffer<int, 4> buffer;
    int value = 0;

    for (int i = 0; i < 1000; ++i) {
        TEST_ASSERT_TRUE(buffer.push(i));
        TEST_ASSERT_TRUE(buffer.pop(&value));
        TEST_ASSERT_EQUAL_INT(i, value);
    }
    TEST_ASSERT_TRUE(buffer.empty());
    TEST_ASSERT_EQUAL_UINT16(0, buffer.drops());
}

void test_high_water_records_the_deepest_occupancy(void) {
    RingBuffer<int, 8> buffer;
    int value = 0;

    for (int i = 0; i < 5; ++i) {
        TEST_ASSERT_TRUE(buffer.push(i));
    }
    for (int i = 0; i < 5; ++i) {
        TEST_ASSERT_TRUE(buffer.pop(&value));
    }

    TEST_ASSERT_TRUE(buffer.empty());
    TEST_ASSERT_EQUAL_size_t(5, buffer.high_water());
}

void test_drop_counter_saturates_instead_of_wrapping(void) {
    // A wrapped counter would report a small number after catastrophic loss,
    // which is worse than reporting a clipped one.
    RingBuffer<uint8_t, 2> buffer;

    TEST_ASSERT_TRUE(buffer.push(1));
    TEST_ASSERT_TRUE(buffer.push(2));
    for (uint32_t i = 0; i < 70000; ++i) {
        TEST_ASSERT_FALSE(buffer.push(3));
    }
    TEST_ASSERT_EQUAL_UINT16(0xFFFF, buffer.drops());
}

void test_interleaved_producer_and_consumer_lose_nothing(void) {
    // Stands in for a producer in interrupt context: the two sides only ever
    // touch their own index, so interleaving them in any order is safe.
    RingBuffer<int, 8> buffer;
    int value = 0;
    int next_expected = 0;
    int produced = 0;

    for (int round = 0; round < 500; ++round) {
        for (int i = 0; i < 3; ++i) {
            if (buffer.push(produced)) {
                ++produced;
            }
        }
        for (int i = 0; i < 3; ++i) {
            if (buffer.pop(&value)) {
                TEST_ASSERT_EQUAL_INT(next_expected, value);
                ++next_expected;
            }
        }
    }
    TEST_ASSERT_EQUAL_INT(produced, next_expected + static_cast<int>(buffer.size()));
}

int main(int, char **) {
    UNITY_BEGIN();
    RUN_TEST(test_push_then_pop_returns_the_value);
    RUN_TEST(test_pop_on_empty_buffer_reports_failure);
    RUN_TEST(test_order_is_first_in_first_out);
    RUN_TEST(test_full_buffer_drops_the_newest_and_counts_it);
    RUN_TEST(test_indices_survive_wrapping_many_times);
    RUN_TEST(test_high_water_records_the_deepest_occupancy);
    RUN_TEST(test_drop_counter_saturates_instead_of_wrapping);
    RUN_TEST(test_interleaved_producer_and_consumer_lose_nothing);
    return UNITY_END();
}
