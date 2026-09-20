#ifndef FLOODNET_NMEA_HPP
#define FLOODNET_NMEA_HPP

#include <stddef.h>
#include <stdint.h>

#include <floodnet/sample.hpp>

namespace floodnet {

/// NMEA 0183 caps a sentence at 82 characters including delimiters.
const size_t NMEA_MAX_SENTENCE = 82;

/// Verifies the trailing "*hh" against an XOR of everything between '$' and '*'.
bool nmea_checksum_ok(const char *sentence, size_t len);

/// Parses a GGA sentence into `*out`, stamping it with `time_ms`.
/// Returns false if the sentence is malformed, oversized, not GGA, or fails checksum.
/// A well-formed sentence reporting no fix returns true with `out->valid == false`.
bool parse_gga(const char *sentence, size_t len, uint32_t time_ms, GpsFix *out);

}  // namespace floodnet

#endif  // FLOODNET_NMEA_HPP
