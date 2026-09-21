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
      diag_() {}

void PollingSampler::collect_gps_bytes(GpsFix *fix, bool *have_fix) {
    *have_fix = false;

    for (;;) {
        const int value = gps_.read_byte();
        if (value < 0) {
            return;  // The receive buffer is genuinely empty, not merely mid-sentence.
        }

        // Not a complete sentence yet: take the next byte.
        if (!line_.feed(static_cast<char>(value))) {
            continue;
        }

        GpsFix parsed;
        if (parse_gga(line_.sentence(), line_.length(), clock_.now_ms(), &parsed) &&
            parsed.valid) {
            *fix = parsed;
            *have_fix = true;
        }

        // Deliberately no return here. Draining continues until read_byte()
        // reports the buffer genuinely empty, because bytes left behind would
        // be lost during the next blocking stall - which is the whole defect
        // this sampler exists to demonstrate. An early return after a
        // completed sentence would look like a tidy-up and would silently
        // change what this file measures.
    }
}

void PollingSampler::step() {
    GpsFix fix;
    bool have_fix = false;
    collect_gps_bytes(&fix, &have_fix);

    // Blocking. Any GPS byte arriving now has no reader and no buffer.
    ImuSample sample;
    if (imu_.read(&sample)) {
        pairer_.submit_imu(sample);
    }

    diag_.drops = gps_.rx_overflows();

    if (!have_fix) {
        return;
    }

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

    diag_.drops = gps_.rx_overflows();
}

}  // namespace floodnet
