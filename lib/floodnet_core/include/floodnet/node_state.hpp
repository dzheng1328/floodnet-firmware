#ifndef FLOODNET_NODE_STATE_HPP
#define FLOODNET_NODE_STATE_HPP

#include <stdint.h>

namespace floodnet {

enum class NodeState : uint8_t { Boot, Acquire, Transmit, Sleep };

/// What the caller observed since the last call. `Tick` carries only the
/// time, and is how timeouts, slot wakes and watchdog chunks are detected.
enum class NodeEvent : uint8_t {
    PeripheralsReady,
    FixQueued,       ///< one valid fix queued, with a paired IMU sample
    FixQueuedNoImu,  ///< one valid fix queued, without one
    TxSucceeded,     ///< queue drained and at least one transmit completed
    TxFailed,        ///< queue drained and no transmit completed
    Tick,
};

/// Every value is from the milestone 3 design doc, "Watchdog timing".
struct NodeTiming {
    uint32_t report_interval_ms = 300000;
    /// Over twice the NEO-M8N's 26 s typical cold-start TTFF.
    uint32_t acquire_timeout_ms = 60000;
    /// Twice the sampler's 5 s transmit timeout, so a radio refusing every
    /// begin_transmit() cannot hold the node in TRANSMIT.
    uint32_t transmit_timeout_ms = 10000;
    /// A 5-minute sleep outlasts the configured 90 s watchdog timeout (the
    /// binding limit, not WDOG1's 128 s hardware ceiling), so sleep is taken
    /// in chunks, each one a transition that kicks the watchdog.
    uint32_t sleep_chunk_ms = 60000;
    uint8_t radio_fail_limit = 3;
    uint8_t imu_fail_limit = 3;
};

/// Which peripherals should be powered. Reported on every call, so a caller
/// can apply it only when it changes.
struct PowerPlan {
    bool gps = false;
    bool imu = false;
    bool radio = false;
};

struct NodeActions {
    /// State entered (or a sleep chunk renewed) on this call.
    bool transitioned = false;
    /// Set exactly when `transitioned` is. Kicking only on progress is the
    /// point: a loop that spins while stuck in one state is still reset.
    bool kick_watchdog = false;
    PowerPlan power;
    bool queue_heartbeat = false;
    bool power_cycle_radio = false;
    bool power_cycle_imu = false;
    /// Meaningful only in SLEEP: when to call back with a Tick.
    uint32_t sleep_until_ms = 0;
};

/// Wrap-safe "has `deadline_ms` passed": correct across the millis() rollover
/// for any deadline within 2^31 ms (24.8 days) of now.
bool time_reached(uint32_t now_ms, uint32_t deadline_ms);

/// The duty-cycle and recovery policy, as a pure transition function.
///
/// Holds no hardware reference: the caller reports events and applies the
/// returned actions. That is what lets every transition be tested on the host
/// with hand-built event sequences. See the milestone 3 design doc.
class NodeStateMachine {
  public:
    explicit NodeStateMachine(const NodeTiming &timing = NodeTiming());

    /// Enters BOOT. The first report slot is `now_ms`.
    NodeActions boot(uint32_t now_ms);

    NodeActions handle(NodeEvent event, uint32_t now_ms);

    NodeState state() const { return state_; }
    uint32_t next_slot_ms() const { return next_slot_ms_; }
    uint8_t radio_failures() const { return radio_failures_; }
    uint8_t imu_failures() const { return imu_failures_; }

  private:
    NodeActions enter(NodeState next, uint32_t now_ms);
    NodeActions hold() const;

    NodeTiming timing_;
    NodeState state_;
    uint32_t entered_ms_;
    uint32_t next_slot_ms_;
    uint32_t sleep_until_ms_;
    uint8_t radio_failures_;
    uint8_t imu_failures_;
};

}  // namespace floodnet

#endif  // FLOODNET_NODE_STATE_HPP
