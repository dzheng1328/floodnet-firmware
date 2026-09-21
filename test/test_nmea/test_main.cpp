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

static bool feed_all(NmeaLineAssembler *assembler, const char *text) {
    bool completed = false;
    for (const char *p = text; *p != '\0'; ++p) {
        if (assembler->feed(*p)) {
            completed = true;
        }
    }
    return completed;
}

void test_assembler_completes_a_sentence_on_the_terminator(void) {
    NmeaLineAssembler assembler;
    const char *text = "$GPGGA,123519,4807.038,N*47\r\n";

    TEST_ASSERT_TRUE(feed_all(&assembler, text));
    TEST_ASSERT_EQUAL_STRING("$GPGGA,123519,4807.038,N*47", assembler.sentence());
    TEST_ASSERT_EQUAL_size_t(27, assembler.length());
}

void test_assembler_reports_nothing_until_the_terminator(void) {
    NmeaLineAssembler assembler;

    TEST_ASSERT_FALSE(feed_all(&assembler, "$GPGGA,123519"));
}

void test_assembler_restarts_on_a_dollar_sign(void) {
    // A sentence truncated mid-flight must not contaminate the next one.
    NmeaLineAssembler assembler;

    TEST_ASSERT_FALSE(feed_all(&assembler, "$GPGGA,trunc"));
    TEST_ASSERT_TRUE(feed_all(&assembler, "$GPGGA,123519*47\r\n"));
    TEST_ASSERT_EQUAL_STRING("$GPGGA,123519*47", assembler.sentence());
}

void test_assembler_discards_an_overlong_sentence(void) {
    NmeaLineAssembler assembler;
    char overlong[NMEA_MAX_SENTENCE + 20];
    overlong[0] = '$';
    for (size_t i = 1; i < sizeof(overlong) - 1; ++i) {
        overlong[i] = 'A';
    }
    overlong[sizeof(overlong) - 1] = '\0';

    TEST_ASSERT_FALSE(feed_all(&assembler, overlong));
    // Resynchronises on the next '$' rather than emitting a truncated line.
    TEST_ASSERT_TRUE(feed_all(&assembler, "$GPGGA,ok*47\r\n"));
    TEST_ASSERT_EQUAL_STRING("$GPGGA,ok*47", assembler.sentence());
}

void test_assembler_ignores_a_bare_terminator(void) {
    NmeaLineAssembler assembler;

    TEST_ASSERT_FALSE(feed_all(&assembler, "\r\n\r\n"));
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
    RUN_TEST(test_assembler_completes_a_sentence_on_the_terminator);
    RUN_TEST(test_assembler_reports_nothing_until_the_terminator);
    RUN_TEST(test_assembler_restarts_on_a_dollar_sign);
    RUN_TEST(test_assembler_discards_an_overlong_sentence);
    RUN_TEST(test_assembler_ignores_a_bare_terminator);
    return UNITY_END();
}
