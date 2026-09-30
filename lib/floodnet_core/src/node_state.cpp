#include <floodnet/node_state.hpp>

namespace floodnet {

namespace {

PowerPlan plan_for(NodeState state) {
    PowerPlan plan;
    switch (state) {
    case NodeState::Boot:
        plan.gps = true;
        plan.imu = true;
        plan.radio = true;
        break;
    case NodeState::Acquire:
        plan.gps = true;
        plan.imu = true;
        break;
    case NodeState::Transmit:
        plan.radio = true;
        break;
    case NodeState::Sleep:
        break;
    }
    return plan;
}

}  // namespace

bool time_reached(uint32_t now_ms, uint32_t deadline_ms) {
    return static_cast<int32_t>(now_ms - deadline_ms) >= 0;
}

NodeStateMachine::NodeStateMachine(const NodeTiming &timing)
    : timing_(timing),
      state_(NodeState::Boot),
      entered_ms_(0),
      next_slot_ms_(0),
      sleep_until_ms_(0),
      radio_failures_(0),
      imu_failures_(0) {}

NodeActions NodeStateMachine::boot(uint32_t now_ms) {
    radio_failures_ = 0;
    imu_failures_ = 0;
    next_slot_ms_ = now_ms;
    return enter(NodeState::Boot, now_ms);
}

NodeActions NodeStateMachine::handle(NodeEvent event, uint32_t now_ms) {
    switch (state_) {
    case NodeState::Boot:
        if (event == NodeEvent::PeripheralsReady) {
            return enter(NodeState::Acquire, now_ms);
        }
        break;

    case NodeState::Acquire:
        if (event == NodeEvent::FixQueued) {
            imu_failures_ = 0;
            return enter(NodeState::Transmit, now_ms);
        }
        if (event == NodeEvent::FixQueuedNoImu) {
            NodeActions actions = enter(NodeState::Transmit, now_ms);
            if (++imu_failures_ >= timing_.imu_fail_limit) {
                actions.power_cycle_imu = true;
                imu_failures_ = 0;
            }
            return actions;
        }
        if (event == NodeEvent::Tick &&
            time_reached(now_ms, entered_ms_ + timing_.acquire_timeout_ms)) {
            // No fix: still transmit, so the gateway can tell a node that
            // cannot see the sky from a node that is dead. Not an IMU failure.
            NodeActions actions = enter(NodeState::Transmit, now_ms);
            actions.queue_heartbeat = true;
            return actions;
        }
        break;

    case NodeState::Transmit:
        if (event == NodeEvent::TxSucceeded) {
            radio_failures_ = 0;
            return enter(NodeState::Sleep, now_ms);
        }
        if (event == NodeEvent::TxFailed ||
            (event == NodeEvent::Tick &&
             time_reached(now_ms, entered_ms_ + timing_.transmit_timeout_ms))) {
            NodeActions actions = enter(NodeState::Sleep, now_ms);
            if (++radio_failures_ >= timing_.radio_fail_limit) {
                actions.power_cycle_radio = true;
                radio_failures_ = 0;
            }
            return actions;
        }
        break;

    case NodeState::Sleep:
        if (event == NodeEvent::Tick) {
            if (time_reached(now_ms, next_slot_ms_)) {
                return enter(NodeState::Acquire, now_ms);
            }
            if (time_reached(now_ms, sleep_until_ms_)) {
                return enter(NodeState::Sleep, now_ms);  // next chunk
            }
        }
        break;
    }
    return hold();
}

NodeActions NodeStateMachine::hold() const {
    NodeActions actions;
    actions.power = plan_for(state_);
    actions.sleep_until_ms = sleep_until_ms_;
    return actions;
}

NodeActions NodeStateMachine::enter(NodeState next, uint32_t now_ms) {
    state_ = next;
    entered_ms_ = now_ms;

    if (next == NodeState::Acquire) {
        // Anchored: advance from the slot just taken, not from now, so a slow
        // acquisition delays one report rather than every report after it.
        // A slot already missed is skipped rather than chased.
        next_slot_ms_ += timing_.report_interval_ms;
        while (time_reached(now_ms, next_slot_ms_)) {
            next_slot_ms_ += timing_.report_interval_ms;
        }
    }
    if (next == NodeState::Sleep) {
        const uint32_t chunk_end = now_ms + timing_.sleep_chunk_ms;
        sleep_until_ms_ = time_reached(chunk_end, next_slot_ms_) ? next_slot_ms_ : chunk_end;
    }

    NodeActions actions;
    actions.transitioned = true;
    actions.kick_watchdog = true;
    actions.power = plan_for(next);
    actions.sleep_until_ms = sleep_until_ms_;
    return actions;
}

}  // namespace floodnet
