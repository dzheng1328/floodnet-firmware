#include <unity.h>
#include <floodnet/sample.hpp>

using namespace floodnet;

void setUp(void) {}
void tearDown(void) {}

void test_sensor_record_defaults_are_invalid(void) {
    SensorRecord record;
    TEST_ASSERT_FALSE(record.gps.valid);
    TEST_ASSERT_FALSE(record.imu.valid);
    TEST_ASSERT_EQUAL_UINT16(0, record.diag.drops);
    TEST_ASSERT_EQUAL_UINT16(0, record.diag.crc_errors);
}

int main(int, char **) {
    UNITY_BEGIN();
    RUN_TEST(test_sensor_record_defaults_are_invalid);
    return UNITY_END();
}
