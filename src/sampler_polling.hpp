#ifndef FLOODNET_SAMPLER_POLLING_HPP
#define FLOODNET_SAMPLER_POLLING_HPP

#include <stddef.h>
#include <stdint.h>

#include <floodnet/hal/clock.hpp>
#include <floodnet/hal/gps.hpp>
#include <floodnet/hal/imu.hpp>
#include <floodnet/hal/radio.hpp>
#include <floodnet/nmea.hpp>
#include <floodnet/packet.hpp>
#include <floodnet/pairing.hpp>

namespace floodnet {

/// Acquisition by polling: each iteration visits every sensor in turn and
/// blocks on it. Simple to follow, and it cannot collect from one peripheral
/// while waiting on another, so bytes arriving during a blocking call are lost.
class PollingSampler {
  public:
    PollingSampler(IGpsSource &gps, IImuSource &imu, IRadio &radio, IClock &clock,
                   uint16_t node_id, uint8_t ttl);

    /// One pass of the superloop.
    void step();

    uint32_t packets_sent() const { return packets_sent_; }
    DiagCounters diag() const { return diag_; }

  private:
    void collect_gps_bytes(GpsFix *fix, bool *have_fix);

    IGpsSource &gps_;
    IImuSource &imu_;
    IRadio &radio_;
    IClock &clock_;

    SamplePairer pairer_;
    uint16_t node_id_;
    uint8_t ttl_;
    uint32_t seq_;
    uint32_t packets_sent_;
    DiagCounters diag_;

    /// Shared with InterruptSampler so both samplers agree on where a sentence
    /// starts and ends. One assembly area for one sentence at a time, which is
    /// all a strictly sequential loop can make use of.
    NmeaLineAssembler line_;
};

}  // namespace floodnet

#endif  // FLOODNET_SAMPLER_POLLING_HPP
