#ifndef FLOODNET_GATEWAY_FORMAT_HPP
#define FLOODNET_GATEWAY_FORMAT_HPP

#include <stddef.h>
#include <stdint.h>

#include <floodnet/packet.hpp>

namespace floodnet {

/// Enough for a REC line with every field at its widest.
const size_t REC_LINE_MAX = 256;

/// The gateway's REC line, without a line ending. The board prints it with
/// Serial.println and the simulation writes it to a transcript, so both
/// produce the same bytes. Returns the length written, or 0 if `out_len` is
/// too small.
size_t format_rec_line(const Packet &p, int16_t rssi, char *out, size_t out_len);

/// "ERR,decode,<count>", without a line ending. Returns the length written,
/// or 0 if `out_len` is too small.
size_t format_decode_error_line(uint16_t count, char *out, size_t out_len);

}  // namespace floodnet

#endif  // FLOODNET_GATEWAY_FORMAT_HPP
