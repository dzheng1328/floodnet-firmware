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
    /// Persistent slot holding the boot counter: SNVS_LPGPR3 on the Teensy.
    ///
    /// WARNING: not slot 0. imxrt.h defines SNVS_LPGPR at offset 0x68, the
    /// legacy alias of LPGPR0 in the RT1060 reference manual's SNVS map, and
    /// Snooze's SnoozeAlarm (6.3.9, hal/TEENSY_40/SnoozeAlarm.cpp:73) writes
    /// SNVS_DEFAULT_PGD_VALUE there. Moving the wake source to SnoozeAlarm
    /// would then overwrite a boot count kept in slot 0, and with it the
    /// gateway's dedup key. Keep this slot clear of anything Snooze writes.
    static const size_t BOOT_COUNT_SLOT = 3;
    static_assert(BOOT_COUNT_SLOT < PERSISTENT_SLOTS, "boot counter slot out of range");

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

    /// Passed to the sampler, which does the transmitting.
    void set_tx_listener(ITxListener *listener) { sampler_.set_tx_listener(listener); }

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
