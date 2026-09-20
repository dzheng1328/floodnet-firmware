#include "sampler_polling.hpp"

namespace floodnet {

namespace {
/// Skew budget between a GPS fix and the IMU sample paired with it.
const uint32_t PAIRING_SKEW_MS = 250;
}  // namespace

PollingSampler::PollingSampler(IGpsSource &gps, IImuSource &imu, IRadio &radio, IClock &clock,
                               uint16_t node_id, uint8_t ttl)
    : gps_(gps),
      imu_(imu),
      radio_(radio),
      clock_(clock),
      pairer_(PAIRING_SKEW_MS),
      node_id_(node_id),
      ttl_(ttl),
      seq_(0),
      packets_sent_(0),
      diag_(),
      line_len_(0) {
    line_[0] = '\0';
}

void PollingSampler::collect_gps_bytes(bool *sentence_ready) {
    *sentence_ready = false;

    for (;;) {
        const int value = gps_.read_byte();
        if (value < 0) {
            return;
        }

        const char c = static_cast<char>(value);
        if (c == '$') {
            line_len_ = 0;
        }

        if (c == '\r' || c == '\n') {
            if (line_len_ > 0) {
                line_[line_len_] = '\0';
                *sentence_ready = true;
                return;
            }
            continue;
        }

        if (line_len_ < NMEA_MAX_SENTENCE) {
            line_[line_len_++] = c;
        } else {
            // Overlong sentence: discard and resynchronise on the next '$'.
            line_len_ = 0;
        }
    }
}

void PollingSampler::step() {
    bool sentence_ready = false;
    collect_gps_bytes(&sentence_ready);

    GpsFix fix;
    const bool have_fix =
        sentence_ready && parse_gga(line_, line_len_, clock_.now_ms(), &fix) && fix.valid;
    if (sentence_ready) {
        line_len_ = 0;
    }

    // Blocking. Any GPS byte arriving now has no reader and no buffer.
    ImuSample sample;
    if (imu_.read(&sample)) {
        pairer_.submit_imu(sample);
    }

    if (!have_fix) {
        diag_.drops = gps_.bytes_dropped();
        return;
    }

    diag_.drops = gps_.bytes_dropped();

    Packet packet;
    packet.node_id = node_id_;
    packet.seq = seq_;
    packet.ttl = ttl_;
    packet.record = pairer_.pair(fix, diag_);

    uint8_t buffer[PACKET_SIZE];
    if (encode_packet(packet, buffer, sizeof(buffer)) != PACKET_SIZE) {
        return;
    }

    // Blocking, and the longest stall in the loop.
    if (radio_.transmit(buffer, PACKET_SIZE)) {
        ++seq_;
        ++packets_sent_;
    }

    diag_.drops = gps_.bytes_dropped();
}

}  // namespace floodnet
