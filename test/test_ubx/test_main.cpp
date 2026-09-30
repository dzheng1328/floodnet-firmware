#include <unity.h>

#include <floodnet/ubx.hpp>

using namespace floodnet;

void setUp(void) {}
void tearDown(void) {}

void test_pmreq_backup_golden_frame(void) {
    uint8_t frame[UBX_PMREQ_BACKUP_SIZE];
    TEST_ASSERT_EQUAL_UINT(UBX_PMREQ_BACKUP_SIZE, ubx_pmreq_backup(frame, sizeof(frame)));

    // Sync, class 0x02 id 0x41, length 16, then version 0, three reserved
    // bytes, duration 0 (indefinite), flags 0x02 (backup), wakeupSources
    // 0x08 (uartrx), then the Fletcher checksum. Computed independently.
    static const uint8_t expected[UBX_PMREQ_BACKUP_SIZE] = {
        0xB5, 0x62, 0x02, 0x41, 0x10, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x02, 0x00, 0x00, 0x00, 0x08, 0x00, 0x00, 0x00, 0x5D, 0x4B,
    };
    TEST_ASSERT_EQUAL_UINT8_ARRAY(expected, frame, UBX_PMREQ_BACKUP_SIZE);
}

void test_pmreq_rejects_short_buffer(void) {
    uint8_t frame[UBX_PMREQ_BACKUP_SIZE - 1];
    TEST_ASSERT_EQUAL_UINT(0, ubx_pmreq_backup(frame, sizeof(frame)));
}

void test_checksum_covers_class_through_payload(void) {
    // UBX-ACK-ACK acknowledging a CFG-PRT (class 0x06, id 0x00):
    // B5 62 05 01 02 00 06 00, checksum 0E 37, worked by hand.
    const uint8_t body[] = {0x05, 0x01, 0x02, 0x00, 0x06, 0x00};
    uint8_t a = 0;
    uint8_t b = 0;
    ubx_checksum(body, sizeof(body), &a, &b);
    TEST_ASSERT_EQUAL_HEX8(0x0E, a);
    TEST_ASSERT_EQUAL_HEX8(0x37, b);
}

int main(int, char **) {
    UNITY_BEGIN();
    RUN_TEST(test_pmreq_backup_golden_frame);
    RUN_TEST(test_pmreq_rejects_short_buffer);
    RUN_TEST(test_checksum_covers_class_through_payload);
    return UNITY_END();
}
