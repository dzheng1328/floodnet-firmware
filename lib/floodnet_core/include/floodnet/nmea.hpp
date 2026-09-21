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

/// Assembles NMEA sentences from a byte stream one character at a time.
///
/// Extracted so the polling and interrupt samplers share one definition of
/// where a sentence starts, where it ends, and what happens to a malformed
/// one. Two copies of those rules would drift.
///
/// Resynchronisation rules, which are the whole point of the class:
/// a '$' restarts the line wherever it appears, a bare terminator is ignored,
/// and a sentence longer than NMEA_MAX_SENTENCE is discarded rather than
/// truncated, because a truncated sentence would fail its checksum and waste
/// a fix that a clean resynchronisation might still catch.
class NmeaLineAssembler {
  public:
    NmeaLineAssembler() : len_(0), complete_len_(0) { line_[0] = '\0'; }

    /// Feeds one character. True when a complete sentence is ready, in which
    /// case sentence() and length() describe it until the next completion.
    bool feed(char c);

    /// Valid only after feed() returned true. NUL-terminated.
    const char *sentence() const { return line_; }

    /// Length of the sentence, excluding the terminator and the NUL.
    size_t length() const { return complete_len_; }

  private:
    char line_[NMEA_MAX_SENTENCE + 1];
    size_t len_;
    size_t complete_len_;
};

}  // namespace floodnet

#endif  // FLOODNET_NMEA_HPP
