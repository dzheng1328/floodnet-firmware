#include "sampler_interrupt.hpp"

namespace floodnet {

namespace {
/// Skew budget between a GPS fix and the IMU sample paired with it. Same value
/// as the polling sampler uses, so pairing behaviour is not a variable in the
/// comparison between them.
const uint32_t PAIRING_SKEW_MS = 250;
}  // namespace

InterruptSampler::InterruptSampler(IGpsSource &gps, IImuSource &imu, IAsyncRadio &radio,
                                   IClock &clock, uint16_t node_id, uint8_t ttl)
    : gps_(gps),
      imu_(imu),
      radio_(radio),
      clock_(clock),
      pairer_(PAIRING_SKEW_MS),
      line_(),
      tx_queue_(),
      node_id_(node_id),
      ttl_(ttl),
      seq_(0),
      packets_sent_(0),
      tx_in_flight_(false),
      diag_() {}

bool InterruptSampler::collect_imu() {
    // The whole improvement on this path in one line: no read unless the
    // sensor says there is something to read.
    if (!imu_.data_ready()) {
        return false;
    }

    ImuSample sample;
    if (!imu_.read(&sample)) {
        return false;
    }
    pairer_.submit_imu(sample);
    return true;
}

size_t InterruptSampler::collect_gps() {
    size_t fixes = 0;

    for (;;) {
        const int value = gps_.read_byte();
        if (value < 0) {
            return fixes;  // genuinely empty, not merely mid-sentence
        }

        if (!line_.feed(static_cast<char>(value))) {
            continue;
        }

        GpsFix parsed;
        if (parse_gga(line_.sentence(), line_.length(), clock_.now_ms(), &parsed) &&
            parsed.valid) {
            enqueue(parsed);
            ++fixes;
        }
    }
}

void InterruptSampler::enqueue(const GpsFix &fix) {
    Packet packet;
    packet.node_id = node_id_;
    // Assigned here, not at transmit time. A packet the queue discards leaves
    // a gap, which is how a receiver learns the node had more to say than it
    // could send.
    packet.seq = seq_++;
    packet.ttl = ttl_;
    packet.record = pairer_.pair(fix, diag_);

    // Never fails: a full queue discards its oldest entry and counts it.
    tx_queue_.push(packet);
}

bool InterruptSampler::service_radio() {
    const bool busy = radio_.tx_busy();

    // A transmit that was in flight and no longer is has completed. Counting
    // completions rather than starts is what keeps packets_sent() comparable
    // with the polling sampler's, which can only count completions.
    if (tx_in_flight_ && !busy) {
        tx_in_flight_ = false;
        ++packets_sent_;
        return true;
    }

    if (busy || tx_queue_.empty()) {
        return false;
    }

    Packet packet;
    if (!tx_queue_.pop(&packet)) {
        return false;
    }

    uint8_t buffer[PACKET_SIZE];
    if (encode_packet(packet, buffer, sizeof(buffer)) != PACKET_SIZE) {
        return false;
    }
    if (!radio_.begin_transmit(buffer, PACKET_SIZE)) {
        return false;
    }

    tx_in_flight_ = true;
    return true;
}

void InterruptSampler::step() {
    diag_.drops = gps_.rx_overflows();

    // IMU first, so a fix completed in this same pass pairs with the freshest
    // orientation available rather than one a pass old.
    bool did_work = collect_imu();

    if (collect_gps() > 0) {
        did_work = true;
    }
    if (service_radio()) {
        did_work = true;
    }

    if (!did_work) {
        // Nothing to do. Yield rather than spin, which on hardware lets the
        // core sleep and in simulation is what allows time to advance.
        clock_.wait_for_event(IDLE_CAP_MS);
    }
}

}  // namespace floodnet
