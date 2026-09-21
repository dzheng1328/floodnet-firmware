#include <string.h>
#include <unity.h>
#include <floodnet/nmea.hpp>

using namespace floodnet;

void setUp(void) {}
void tearDown(void) {}

static const char kValidGga[] =
    "$GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,*47";

void test_checksum_accepts_valid_sentence(void) {
    TEST_ASSERT_TRUE(nmea_checksum_ok(kValidGga, strlen(kValidGga)));
}

void test_checksum_rejects_tampered_sentence(void) {
    char bad[128];
    strcpy(bad, kValidGga);
    bad[10] = '9';  // change a payload digit, leave the stated checksum alone
    TEST_ASSERT_FALSE(nmea_checksum_ok(bad, strlen(bad)));
}

void test_parse_extracts_position(void) {
    GpsFix fix;
    TEST_ASSERT_TRUE(parse_gga(kValidGga, strlen(kValidGga), 4242, &fix));

    TEST_ASSERT_TRUE(fix.valid);
    TEST_ASSERT_EQUAL_UINT32(4242, fix.time_ms);
    TEST_ASSERT_EQUAL_INT32(481173000, fix.lat_1e7);   // 48 deg 07.038 min N
    TEST_ASSERT_EQUAL_INT32(115166667, fix.lon_1e7);   // 11 deg 31.000 min E
    TEST_ASSERT_EQUAL_INT32(545400, fix.alt_mm);
    TEST_ASSERT_EQUAL_UINT8(8, fix.satellites);
}

void test_parse_applies_southern_and_western_signs(void) {
    const char sentence[] =
        "$GPGGA,123519,4807.038,S,01131.000,W,1,08,0.9,545.4,M,46.9,M,,*48";
    GpsFix fix;
    TEST_ASSERT_TRUE(parse_gga(sentence, strlen(sentence), 0, &fix));
    TEST_ASSERT_EQUAL_INT32(-481173000, fix.lat_1e7);
    TEST_ASSERT_EQUAL_INT32(-115166667, fix.lon_1e7);
}

void test_parse_marks_no_fix_invalid(void) {
    // Fix quality field is 0, meaning the receiver has no position solution.
    const char sentence[] = "$GPGGA,123519,4807.038,N,01131.000,E,0,00,,,M,,M,,*52";
    GpsFix fix;
    TEST_ASSERT_TRUE(parse_gga(sentence, strlen(sentence), 0, &fix));
    TEST_ASSERT_FALSE(fix.valid);
}

void test_parse_rejects_wrong_sentence_type(void) {
    const char sentence[] = "$GPRMC,123519,A,4807.038,N,01131.000,E,022.4,084.4,230394,003.1,W*6A";
    GpsFix fix;
    TEST_ASSERT_FALSE(parse_gga(sentence, strlen(sentence), 0, &fix));
}

void test_parse_rejects_truncated_sentence(void) {
    const char sentence[] = "$GPGGA,123519,4807.0";
    GpsFix fix;
    TEST_ASSERT_FALSE(parse_gga(sentence, strlen(sentence), 0, &fix));
}

void test_parse_rejects_oversized_sentence(void) {
    char oversized[NMEA_MAX_SENTENCE + 10];
    memset(oversized, 'A', sizeof(oversized));
    oversized[0] = '$';
    GpsFix fix;
    TEST_ASSERT_FALSE(parse_gga(oversized, sizeof(oversized), 0, &fix));
}

int main(int, char **) {
    UNITY_BEGIN();
    RUN_TEST(test_checksum_accepts_valid_sentence);
    RUN_TEST(test_checksum_rejects_tampered_sentence);
    RUN_TEST(test_parse_extracts_position);
    RUN_TEST(test_parse_applies_southern_and_western_signs);
    RUN_TEST(test_parse_marks_no_fix_invalid);
    RUN_TEST(test_parse_rejects_wrong_sentence_type);
    RUN_TEST(test_parse_rejects_truncated_sentence);
    RUN_TEST(test_parse_rejects_oversized_sentence);
    return UNITY_END();
}
