#include <floodnet/nmea.hpp>

#include <math.h>
#include <stdlib.h>
#include <string.h>

namespace floodnet {
namespace {

/// Copies field `index` (comma separated, field 0 is the talker id) into `buf`.
/// Returns false if the field does not exist or does not fit.
bool field(const char *sentence, size_t len, int index, char *buf, size_t buf_len) {
    size_t start = 1;  // skip the leading '$'
    int current = 0;

    for (size_t i = 1; i <= len; ++i) {
        const bool at_end = (i == len) || (sentence[i] == '*');
        if (i == len || sentence[i] == ',' || at_end) {
            if (current == index) {
                const size_t field_len = i - start;
                if (field_len >= buf_len) {
                    return false;
                }
                memcpy(buf, &sentence[start], field_len);
                buf[field_len] = '\0';
                return true;
            }
            if (at_end) {
                return false;
            }
            ++current;
            start = i + 1;
        }
    }
    return false;
}

/// Converts NMEA "ddmm.mmmm" (or "dddmm.mmmm") to degrees x 1e7.
/// `degree_digits` is 2 for latitude and 3 for longitude.
bool to_degrees_1e7(const char *value, int degree_digits, int32_t *out) {
    const size_t len = strlen(value);
    if (len < static_cast<size_t>(degree_digits) + 1) {
        return false;
    }

    char degree_buf[4];
    memcpy(degree_buf, value, static_cast<size_t>(degree_digits));
    degree_buf[degree_digits] = '\0';

    char *end = nullptr;
    const long degrees = strtol(degree_buf, &end, 10);
    if (end == degree_buf) {
        return false;
    }

    const double minutes = strtod(&value[degree_digits], &end);
    if (end == &value[degree_digits]) {
        return false;
    }

    const double total = static_cast<double>(degrees) + (minutes / 60.0);
    *out = static_cast<int32_t>(llround(total * 1e7));
    return true;
}

}  // namespace

bool nmea_checksum_ok(const char *sentence, size_t len) {
    if (sentence == nullptr || len < 4 || len > NMEA_MAX_SENTENCE || sentence[0] != '$') {
        return false;
    }

    size_t star = 0;
    for (size_t i = 1; i < len; ++i) {
        if (sentence[i] == '*') {
            star = i;
            break;
        }
    }
    if (star == 0 || star + 2 >= len + 1 || len < star + 3) {
        return false;
    }

    uint8_t computed = 0;
    for (size_t i = 1; i < star; ++i) {
        computed ^= static_cast<uint8_t>(sentence[i]);
    }

    char stated_buf[3] = {sentence[star + 1], sentence[star + 2], '\0'};
    char *end = nullptr;
    const long stated = strtol(stated_buf, &end, 16);
    if (end != stated_buf + 2) {
        return false;
    }

    return static_cast<uint8_t>(stated) == computed;
}

bool parse_gga(const char *sentence, size_t len, uint32_t time_ms, GpsFix *out) {
    if (out == nullptr || !nmea_checksum_ok(sentence, len)) {
        return false;
    }

    char buf[16];
    if (!field(sentence, len, 0, buf, sizeof(buf))) {
        return false;
    }
    // Accept any talker id: GPGGA, GNGGA, GLGGA all carry the same payload.
    if (strlen(buf) != 5 || strcmp(&buf[2], "GGA") != 0) {
        return false;
    }

    GpsFix fix;
    fix.time_ms = time_ms;

    if (!field(sentence, len, 6, buf, sizeof(buf))) {
        return false;
    }
    const long quality = strtol(buf, nullptr, 10);

    if (!field(sentence, len, 7, buf, sizeof(buf))) {
        return false;
    }
    fix.satellites = static_cast<uint8_t>(strtol(buf, nullptr, 10));

    if (quality == 0) {
        fix.valid = false;
        *out = fix;
        return true;
    }

    char hemisphere[4];
    if (!field(sentence, len, 2, buf, sizeof(buf)) ||
        !field(sentence, len, 3, hemisphere, sizeof(hemisphere)) ||
        !to_degrees_1e7(buf, 2, &fix.lat_1e7)) {
        return false;
    }
    if (hemisphere[0] == 'S') {
        fix.lat_1e7 = -fix.lat_1e7;
    }

    if (!field(sentence, len, 4, buf, sizeof(buf)) ||
        !field(sentence, len, 5, hemisphere, sizeof(hemisphere)) ||
        !to_degrees_1e7(buf, 3, &fix.lon_1e7)) {
        return false;
    }
    if (hemisphere[0] == 'W') {
        fix.lon_1e7 = -fix.lon_1e7;
    }

    if (!field(sentence, len, 9, buf, sizeof(buf))) {
        return false;
    }
    fix.alt_mm = static_cast<int32_t>(llround(strtod(buf, nullptr) * 1000.0));

    fix.valid = true;
    *out = fix;
    return true;
}

}  // namespace floodnet
