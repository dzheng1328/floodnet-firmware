#include <unity.h>
#include <floodnet/pairing.hpp>

using namespace floodnet;

void setUp(void) {}
void tearDown(void) {}

static ImuSample imu_at(uint32_t time_ms) {
    ImuSample s;
    s.time_ms = time_ms;
    s.yaw_cd = 1234;
    s.valid = true;
    return s;
}

static GpsFix fix_at(uint32_t time_ms) {
    GpsFix f;
    f.time_ms = time_ms;
    f.valid = true;
    return f;
}

void test_pairs_imu_within_skew(void) {
    SamplePairer pairer(50);
    pairer.submit_imu(imu_at(1000));

    const SensorRecord record = pairer.pair(fix_at(1030), DiagCounters());
    TEST_ASSERT_TRUE(record.imu.valid);
    TEST_ASSERT_EQUAL_INT16(1234, record.imu.yaw_cd);
}

void test_rejects_imu_beyond_skew(void) {
    SamplePairer pairer(50);
    pairer.submit_imu(imu_at(1000));

    const SensorRecord record = pairer.pair(fix_at(1100), DiagCounters());
    TEST_ASSERT_FALSE(record.imu.valid);
    TEST_ASSERT_TRUE(record.gps.valid);  // the fix itself is still good
}

void test_skew_is_symmetric(void) {
    SamplePairer pairer(50);
    pairer.submit_imu(imu_at(1100));

    // IMU ahead of the fix by more than the skew budget must also be rejected.
    const SensorRecord record = pairer.pair(fix_at(1000), DiagCounters());
    TEST_ASSERT_FALSE(record.imu.valid);
}

void test_no_imu_submitted_yields_invalid_imu(void) {
    SamplePairer pairer(50);
    const SensorRecord record = pairer.pair(fix_at(1000), DiagCounters());
    TEST_ASSERT_FALSE(record.imu.valid);
}

void test_latest_imu_wins(void) {
    SamplePairer pairer(50);
    pairer.submit_imu(imu_at(1000));
    ImuSample newer = imu_at(1020);
    newer.yaw_cd = 999;
    pairer.submit_imu(newer);

    const SensorRecord record = pairer.pair(fix_at(1030), DiagCounters());
    TEST_ASSERT_EQUAL_INT16(999, record.imu.yaw_cd);
}

void test_diag_counters_pass_through(void) {
    SamplePairer pairer(50);
    DiagCounters diag;
    diag.drops = 11;
    diag.crc_errors = 3;

    const SensorRecord record = pairer.pair(fix_at(1000), diag);
    TEST_ASSERT_EQUAL_UINT16(11, record.diag.drops);
    TEST_ASSERT_EQUAL_UINT16(3, record.diag.crc_errors);
}

int main(int, char **) {
    UNITY_BEGIN();
    RUN_TEST(test_pairs_imu_within_skew);
    RUN_TEST(test_rejects_imu_beyond_skew);
    RUN_TEST(test_skew_is_symmetric);
    RUN_TEST(test_no_imu_submitted_yields_invalid_imu);
    RUN_TEST(test_latest_imu_wins);
    RUN_TEST(test_diag_counters_pass_through);
    return UNITY_END();
}
