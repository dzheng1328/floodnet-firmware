#include "duty_cycled_node.hpp"

namespace floodnet {

DutyCycledNode::DutyCycledNode(IGpsSource &gps, IImuSource &imu, IAsyncRadio &radio,
                               IClock &clock, IPower &power, IWatchdog &watchdog,
                               IPersistentStore &store, const DutyCycledNodeConfig &config)
    : gps_(gps),
      clock_(clock),
      power_(power),
      watchdog_(watchdog),
      store_(store),
      config_(config),
      sampler_(gps, imu, radio, clock, config.node_id, config.ttl, WireVersion::V2),
      fsm_(config.timing),
      applied_(),
      power_known_(false),
      seq_at_acquire_(0),
      sent_at_transmit_(0),
      sleep_until_ms_(0),
      boot_count_(0) {}

void DutyCycledNode::begin() {
    // Counted first, so every boot is visible at the gateway, including one
    // that ends in another reset before it reports.
    const uint32_t stored = store_.read_u32(BOOT_COUNT_SLOT);
    boot_count_ = stored >= 0xFFFF ? 0xFFFF : static_cast<uint16_t>(stored + 1);
    store_.write_u32(BOOT_COUNT_SLOT, boot_count_);

    watchdog_.begin(config_.watchdog_timeout_ms);
    apply(fsm_.boot(clock_.now_ms()));
    // The caller initialised the drivers before begin(), so the peripherals
    // are ready as soon as they are powered.
    apply(fsm_.handle(NodeEvent::PeripheralsReady, clock_.now_ms()));
}

void DutyCycledNode::step() {
    NodeEvent event = NodeEvent::Tick;

    switch (fsm_.state()) {
    case NodeState::Boot:
        event = NodeEvent::PeripheralsReady;
        break;

    case NodeState::Acquire:
        sampler_.step();
        if (sampler_.next_seq() != seq_at_acquire_) {
            event = sampler_.last_record_imu_valid() ? NodeEvent::FixQueued
                                                     : NodeEvent::FixQueuedNoImu;
        }
        break;

    case NodeState::Transmit:
        // The GPS is going into backup, but bytes already on the wire still
        // land in the receive buffer. Discarding them is what keeps this wake
        // to one fix: the sampler would otherwise queue a second.
        drain_gps();
        sampler_.step();
        if (sampler_.tx_queue_size() == 0 && !sampler_.tx_in_flight()) {
            event = sampler_.packets_sent() != sent_at_transmit_ ? NodeEvent::TxSucceeded
                                                                 : NodeEvent::TxFailed;
        }
        break;

    case NodeState::Sleep:
        power_.sleep_until(sleep_until_ms_);
        break;
    }

    apply(fsm_.handle(event, clock_.now_ms()));
}

void DutyCycledNode::apply(const NodeActions &actions) {
    if (actions.kick_watchdog) {
        watchdog_.kick();
    }
    // A power cycle leaves the part on; the plan below then applies the
    // state the new state wants.
    if (actions.power_cycle_radio) {
        power_.power_cycle(Peripheral::Radio);
        applied_.radio = true;
    }
    if (actions.power_cycle_imu) {
        power_.power_cycle(Peripheral::Imu);
        applied_.imu = true;
    }
    if (!actions.transitioned) {
        return;
    }

    switch (fsm_.state()) {
    case NodeState::Acquire:
        // Before the GPS is powered: a sentence left in the receive buffer
        // from the previous wake would otherwise be parsed now and stamped
        // with this wake's time, reporting a stale position as fresh.
        drain_gps();
        sampler_.set_node_status(boot_count_, power_.battery_mv());
        seq_at_acquire_ = sampler_.next_seq();
        break;
    case NodeState::Transmit:
        if (actions.queue_heartbeat) {
            sampler_.enqueue_heartbeat();
        }
        sent_at_transmit_ = sampler_.packets_sent();
        break;
    case NodeState::Sleep:
        sleep_until_ms_ = actions.sleep_until_ms;
        break;
    case NodeState::Boot:
        break;
    }

    set_power(Peripheral::Gps, actions.power.gps, &applied_.gps);
    set_power(Peripheral::Imu, actions.power.imu, &applied_.imu);
    set_power(Peripheral::Radio, actions.power.radio, &applied_.radio);
    power_known_ = true;
}

void DutyCycledNode::set_power(Peripheral p, bool on, bool *applied) {
    // Only on change: waking the GPS or resuming the IMU is not free on
    // hardware, and a sleep chunk renewal must not repeat it.
    if (power_known_ && *applied == on) {
        return;
    }
    power_.set_power(p, on ? PowerState::On : PowerState::Low);
    *applied = on;
}

void DutyCycledNode::drain_gps() {
    while (gps_.read_byte() >= 0) {
    }
}

}  // namespace floodnet
