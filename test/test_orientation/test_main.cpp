#include <math.h>
#include <unity.h>

#include <floodnet/orientation.hpp>

using namespace floodnet;

void setUp(void) {}
void tearDown(void) {}

// The BNO055 reports each Euler angle as an int16 in 1/16 degree, and the
// Adafruit library divides by 16.0. These are headings it can actually emit.
static float from_register(int16_t raw) { return static_cast<float>(raw / 16.0); }

static ImuSample convert(float heading_deg, float roll_deg, float pitch_deg) {
    ImuSample s;
    TEST_ASSERT_TRUE(bno055_euler_to_sample(heading_deg, roll_deg, pitch_deg, &s));
    return s;
}

void test_heading_below_the_old_overflow_point_is_unchanged(void) {
    TEST_ASSERT_EQUAL_INT16(0, convert(0.0f, 0.0f, 0.0f).yaw_cd);
    TEST_ASSERT_EQUAL_INT16(4500, convert(45.0f, 0.0f, 0.0f).yaw_cd);
    TEST_ASSERT_EQUAL_INT16(17994, convert(from_register(2879), 0.0f, 0.0f).yaw_cd);  // 179.9375
}

void test_heading_from_180_wraps_to_negative(void) {
    TEST_ASSERT_EQUAL_INT16(-18000, convert(180.0f, 0.0f, 0.0f).yaw_cd);
    TEST_ASSERT_EQUAL_INT16(-10000, convert(260.0f, 0.0f, 0.0f).yaw_cd);
}

// 327.6875 deg is 32768.75 cd, the first register value the old int16 cast
// could not hold.
void test_heading_past_the_int16_limit_wraps(void) {
    TEST_ASSERT_EQUAL_INT16(-3231, convert(from_register(5243), 0.0f, 0.0f).yaw_cd);
    TEST_ASSERT_EQUAL_INT16(-1000, convert(350.0f, 0.0f, 0.0f).yaw_cd);
    TEST_ASSERT_EQUAL_INT16(-6, convert(from_register(5759), 0.0f, 0.0f).yaw_cd);  // 359.9375
}

void test_heading_of_360_is_north(void) {
    TEST_ASSERT_EQUAL_INT16(0, convert(360.0f, 0.0f, 0.0f).yaw_cd);
}

void test_every_register_heading_fits_and_is_in_range(void) {
    for (int16_t raw = 0; raw <= 5760; ++raw) {
        const float deg = from_register(raw);
        const int16_t cd = convert(deg, 0.0f, 0.0f).yaw_cd;
        TEST_ASSERT_TRUE(cd >= -18000 && cd < 18000);
        const double expected = fmod(round(raw * 6.25) + 18000.0, 36000.0) - 18000.0;
        TEST_ASSERT_EQUAL_INT16(static_cast<int16_t>(expected), cd);
    }
}

// Rounds to the nearest centidegree rather than truncating toward zero:
// 1/16 degree is 6.25 cd, so 0.0625 deg must become 6, and -0.0625 deg -6.
void test_rounds_to_nearest_centidegree(void) {
    TEST_ASSERT_EQUAL_INT16(6, convert(from_register(1), 0.0f, 0.0f).yaw_cd);
    TEST_ASSERT_EQUAL_INT16(19, convert(from_register(3), 0.0f, 0.0f).yaw_cd);  // 18.75
    TEST_ASSERT_EQUAL_INT16(-6, convert(0.0f, from_register(-1), 0.0f).roll_cd);
}

// The BNO055's Euler registers are heading (0x1A), roll (0x1C), pitch (0x1E),
// and Adafruit_BNO055::getEvent copies them into orientation.x, .y, .z in that
// order. So .y is roll and .z is pitch.
void test_roll_and_pitch_land_in_their_own_fields(void) {
    const ImuSample s = convert(10.0f, -20.0f, 30.0f);
    TEST_ASSERT_EQUAL_INT16(1000, s.yaw_cd);
    TEST_ASSERT_EQUAL_INT16(-2000, s.roll_cd);
    TEST_ASSERT_EQUAL_INT16(3000, s.pitch_cd);
}

void test_full_pitch_and_roll_ranges_fit(void) {
    const ImuSample hi = convert(0.0f, 90.0f, 179.9375f);
    TEST_ASSERT_EQUAL_INT16(9000, hi.roll_cd);
    TEST_ASSERT_EQUAL_INT16(17994, hi.pitch_cd);
    const ImuSample lo = convert(0.0f, -90.0f, -180.0f);
    TEST_ASSERT_EQUAL_INT16(-9000, lo.roll_cd);
    TEST_ASSERT_EQUAL_INT16(-18000, lo.pitch_cd);
}

void test_non_finite_input_is_rejected_and_leaves_the_sample_alone(void) {
    ImuSample s;
    s.yaw_cd = 11;
    s.roll_cd = 22;
    s.pitch_cd = 33;
    TEST_ASSERT_FALSE(bno055_euler_to_sample(NAN, 0.0f, 0.0f, &s));
    TEST_ASSERT_FALSE(bno055_euler_to_sample(0.0f, INFINITY, 0.0f, &s));
    TEST_ASSERT_FALSE(bno055_euler_to_sample(0.0f, 0.0f, -INFINITY, &s));
    TEST_ASSERT_EQUAL_INT16(11, s.yaw_cd);
    TEST_ASSERT_EQUAL_INT16(22, s.roll_cd);
    TEST_ASSERT_EQUAL_INT16(33, s.pitch_cd);
}

void test_null_output_is_rejected(void) {
    TEST_ASSERT_FALSE(bno055_euler_to_sample(0.0f, 0.0f, 0.0f, nullptr));
}

int main(int, char **) {
    UNITY_BEGIN();
    RUN_TEST(test_heading_below_the_old_overflow_point_is_unchanged);
    RUN_TEST(test_heading_from_180_wraps_to_negative);
    RUN_TEST(test_heading_past_the_int16_limit_wraps);
    RUN_TEST(test_heading_of_360_is_north);
    RUN_TEST(test_every_register_heading_fits_and_is_in_range);
    RUN_TEST(test_rounds_to_nearest_centidegree);
    RUN_TEST(test_roll_and_pitch_land_in_their_own_fields);
    RUN_TEST(test_full_pitch_and_roll_ranges_fit);
    RUN_TEST(test_non_finite_input_is_rejected_and_leaves_the_sample_alone);
    RUN_TEST(test_null_output_is_rejected);
    return UNITY_END();
}
