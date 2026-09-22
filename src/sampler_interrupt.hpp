#ifndef FLOODNET_SAMPLER_INTERRUPT_HPP
#define FLOODNET_SAMPLER_INTERRUPT_HPP

#include <stddef.h>
#include <stdint.h>

#include <floodnet/hal/clock.hpp>
#include <floodnet/hal/gps.hpp>
#include <floodnet/hal/imu.hpp>
#include <floodnet/hal/radio.hpp>
#include <floodnet/nmea.hpp>
#include <floodnet/packet.hpp>
#include <floodnet/packet_queue.hpp>
#include <floodnet/pairing.hpp>

namespace floodnet {

/// Acquisition without blocking. Each pass collects whatever is ready and
/// returns; nothing in step() waits on a peripheral. Bytes arriving during a
/// radio transmission therefore have both a reader and somewhere to go.
///
/// What this does NOT do is make the radio faster. At SF12 a node produces
/// fixes far faster than the modem can send them, and the surplus is dropped
/// from the outbound queue and counted. That is the point: the loss is the
/// same physics as milestone 1, but it is explicit, chosen, and visible to a
/// receiver as a sequence gap instead of silently corrupting sentences.
class InterruptSampler {
  public:
    /// Outbound depth. See the milestone 2 design doc, "Buffer capacities".
    static const size_t TX_QUEUE_DEPTH = 8;

    /// Cap on a single idle, matching the Teensy systick period that wakes
    /// the `wfi` instruction when no peripheral interrupt arrives first.
    static const uint32_t IDLE_CAP_MS = 1;

    InterruptSampler(IGpsSource &gps, IImuSource &imu, IAsyncRadio &radio, IClock &clock,
                     uint16_t node_id, uint8_t ttl);

    /// One pass of the main loop.
    void step();

    /// Transmissions that completed. Counts the same event
    /// PollingSampler::packets_sent() counts, so the two are comparable.
    uint32_t packets_sent() const { return packets_sent_; }

    DiagCounters diag() const { return diag_; }

    /// Packets the node chose not to send because the outbound queue was full.
    uint16_t tx_queue_drops() const { return tx_queue_.drops(); }

    size_t tx_queue_high_water() const { return tx_queue_.high_water(); }

    /// The sequence number the next queued packet will carry. Assigned at
    /// queue time rather than send time, so a drop leaves a gap a receiver
    /// can count.
    uint32_t next_seq() const { return seq_; }

  private:
    bool collect_imu();
    size_t collect_gps();
    void enqueue(const GpsFix &fix);
    bool service_radio();

    IGpsSource &gps_;
    IImuSource &imu_;
    IAsyncRadio &radio_;
    IClock &clock_;

    SamplePairer pairer_;
    NmeaLineAssembler line_;
    PacketQueue<TX_QUEUE_DEPTH> tx_queue_;

    uint16_t node_id_;
    uint8_t ttl_;
    uint32_t seq_;
    uint32_t packets_sent_;
    bool tx_in_flight_;
    DiagCounters diag_;
};

}  // namespace floodnet

#endif  // FLOODNET_SAMPLER_INTERRUPT_HPP
