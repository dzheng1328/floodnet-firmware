#ifndef FLOODNET_DUTY_CYCLED_NODE_HPP
#define FLOODNET_DUTY_CYCLED_NODE_HPP

#include <stddef.h>
#include <stdint.h>

#include <floodnet/hal/clock.hpp>
#include <floodnet/hal/gps.hpp>
#include <floodnet/hal/imu.hpp>
#include <floodnet/hal/persistent_store.hpp>
#include <floodnet/hal/power.hpp>
#include <floodnet/hal/radio.hpp>
#include <floodnet/hal/watchdog.hpp>
#include <floodnet/node_state.hpp>

#include "sampler_interrupt.hpp"

namespace floodnet {

struct DutyCycledNodeConfig {
    uint16_t node_id = 1;
    uint8_t ttl = 3;
    NodeTiming timing = NodeTiming();
    /// Above the longest legitimate gap between transitions (ACQUIRE's 60 s
    /// timeout), below WDOG1's 128 s ceiling.
    uint32_t watchdog_timeout_ms = 90000;
};

/// One fix every report interval, asleep in between, recovering from the
/// faults in the milestone 3 design doc.
///
/// NodeStateMachine decides what is powered and when; InterruptSampler does
/// the acquisition and transmission exactly as it does in node_interrupt.
/// This class only carries events from one to the other.
class DutyCycledNode {
  public:
    /// Persistent slot holding the boot counter.
    static const size_t BOOT_COUNT_SLOT = 0;

    DutyCycledNode(IGpsSource &gps, IImuSource &imu, IAsyncRadio &radio, IClock &clock,
                   IPower &power, IWatchdog &watchdog, IPersistentStore &store,
                   const DutyCycledNodeConfig &config);

    /// Call once, after the drivers are initialised.
    void begin();

    /// One pass of the main loop. May sleep for up to one watchdog chunk.
    void step();

    NodeState state() const { return fsm_.state(); }
    uint16_t boot_count() const { return boot_count_; }
    const InterruptSampler &sampler() const { return sampler_; }

  private:
    void apply(const NodeActions &actions);
    void set_power(Peripheral p, bool on, bool *applied);
    void drain_gps();

    IGpsSource &gps_;
    IClock &clock_;
    IPower &power_;
    IWatchdog &watchdog_;
    IPersistentStore &store_;
    DutyCycledNodeConfig config_;
    InterruptSampler sampler_;
    NodeStateMachine fsm_;
    PowerPlan applied_;
    bool power_known_;
    uint32_t seq_at_acquire_;
    uint32_t sent_at_transmit_;
    uint32_t sleep_until_ms_;
    uint16_t boot_count_;
};

}  // namespace floodnet

#endif  // FLOODNET_DUTY_CYCLED_NODE_HPP
