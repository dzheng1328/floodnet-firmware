#ifndef FLOODNET_TEST_POWER_RIG_HPP
#define FLOODNET_TEST_POWER_RIG_HPP

#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include <functional>
#include <memory>
#include <vector>

#include <floodnet/downtime.hpp>
#include <floodnet/gateway_format.hpp>
#include <floodnet/hal/tx_listener.hpp>
#include <floodnet/mesh.hpp>
#include <floodnet/packet.hpp>

#include "current_model.hpp"
#include "duty_cycled_node.hpp"
#include "fake_gps.hpp"
#include "fake_imu.hpp"
#include "fake_persistent_store.hpp"
#include "fake_power.hpp"
#include "fake_radio.hpp"
#include "fake_watchdog.hpp"
#include "hang_breaker.hpp"
#include "noisy_channel.hpp"
#include "sampler_interrupt.hpp"
#include "sim_clock.hpp"

namespace floodnet {

enum class Build { Interrupt, DutyCycled };

enum class FaultKind { LostCompletion, SkyBlockage, ImuFailing, RadioWedge, Hang };

// The milestone 2 benchmark's sentence, byte rate, IMU read cost and SF12
// airtime, so both milestones measure the same simulated hardware.
const double kRigGpsByteRate = 0.96;
const char kRigFixSentence[] =
    "$GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,*47\r\n";
const uint32_t kRigImuReadMs = 10;
const uint32_t kRigRadioSf12Ms = 3023;
const uint16_t kRigNodeId = 0x0042;
const uint8_t kRigTtl = 3;

/// FakeRadio keeps at most 64 payload bytes, so a channel copy never needs more.
const size_t kRigMaxFrame = 64;

/// How far past its end time run_until_quiet() may go to let a transmit in
/// flight finish. SF12 airtime is about 3 s and the sampler's timeout 5 s.
const uint32_t kRigQuietLimitMs = 10000;

/// Ends a hang when the run ends: for the build that has no watchdog.
class ScenarioEnd : public IHangBreaker {
  public:
    explicit ScenarioEnd(SimClock &clock) : clock_(clock), end_ms_(0) {}
    void set_end(uint32_t end_ms) { end_ms_ = end_ms; }
    bool should_break() const override { return clock_.now_ms() >= end_ms_; }

  private:
    SimClock &clock_;
    uint32_t end_ms_;
};

/// One node, either build, on the same fakes, with a gateway that keeps what
/// it receives. Faults are applied between node steps, so one scheduled
/// inside a sleep jump lands up to SLEEP_CHUNK_MS late for the duty-cycled
/// build; the README states this.
class PowerRig {
  public:
    /// `capacity_nA_ms` of 0 means unlimited.
    PowerRig(Build build, uint32_t teensy_sleep_nA, uint64_t capacity_nA_ms)
        : build_(build),
          clock_(),
          gps_(kRigFixSentence, kRigGpsByteRate, FakeGps::MAX_FIFO_DEPTH),
          imu_(clock_, kRigImuReadMs),
          radio_(clock_, kRigRadioSf12Ms),
          power_(clock_, gps_, imu_, radio_, CurrentModel::with_teensy_sleep(teensy_sleep_nA),
                 capacity_nA_ms),
          watchdog_(),
          store_(),
          end_(clock_),
          dedup_(),
          last_sent_(0),
          reboots_(0),
          channel_(nullptr),
          queued_before_boot_(0),
          tx_listener_(nullptr),
          decode_failures_(0),
          resets_mid_transmit_(0),
          unmatched_accepted_(0),
          timeouts_before_boot_(0) {
        // FakePower first: each tick is charged at the state it began in.
        clock_.add_observer(&power_);
        clock_.add_observer(&gps_);
        clock_.add_observer(&imu_);
        clock_.add_observer(&radio_);
        clock_.add_observer(&watchdog_);

        // Both builds start with the GPS cold, so their startups are alike.
        gps_.set_powered(false);
        if (build_ == Build::DutyCycled) {
            boot();
        } else {
            gps_.set_powered(true);
            sampler_.reset(
                new InterruptSampler(gps_, imu_, radio_, clock_, kRigNodeId, kRigTtl));
        }
    }

    void add_fault(FaultKind kind, uint32_t start_ms, uint32_t end_ms) {
        Fault fault;
        fault.kind = kind;
        fault.start_ms = start_ms;
        fault.end_ms = end_ms;
        fault.started = false;
        fault.ended = false;
        faults_.push_back(fault);
    }

    /// Runs until `end_ms` or until the battery is empty, whichever is first.
    void run_until(uint32_t end_ms) {
        end_.set_end(end_ms);
        while (clock_.now_ms() < end_ms && !power_.depleted()) {
            step_once();
        }
    }

    /// run_until(end_ms), then one pass at a time until no transmit is in
    /// flight, at most kRigQuietLimitMs past `end_ms`. A transcript that ends
    /// between the radio completing a frame and the sampler observing it
    /// would show a REC line with no TX line.
    void run_until_quiet(uint32_t end_ms) {
        run_until(end_ms);
        const uint32_t limit = end_ms + kRigQuietLimitMs;
        end_.set_end(limit);
        while (tx_in_flight() && clock_.now_ms() < limit && !power_.depleted()) {
            step_once();
        }
    }

    DowntimeReport downtime(uint32_t scenario_ms) const {
        DowntimeInput in;
        in.records = records_.data();
        in.count = records_.size();
        in.scenario_ms = scenario_ms;
        in.died = power_.depleted();
        in.died_at_ms = power_.depleted_at_ms();
        return compute_downtime(in);
    }

    SimClock &clock() { return clock_; }
    FakeGps &gps() { return gps_; }
    FakeImu &imu() { return imu_; }
    FakeRadio &radio() { return radio_; }
    FakePower &power() { return power_; }
    FakeWatchdog &watchdog() { return watchdog_; }
    FakePersistentStore &store() { return store_; }
    const DutyCycledNode &node() const { return *node_; }
    const std::vector<Packet> &deliveries() const { return deliveries_; }
    const std::vector<GatewayRecord> &records() const { return records_; }
    size_t reboots() const { return reboots_; }

    /// Puts `channel` between the node's radio and the gateway's decode.
    /// nullptr, the default, is no channel: the delivery path is then exactly
    /// the milestone 3 path.
    void set_channel(NoisyChannel *channel) { channel_ = channel; }

    /// Sequence numbers the node issued, summed over every boot: every record
    /// it queued for transmission, whether or not it was ever sent.
    uint32_t records_queued() const {
        const uint32_t current = node_ ? node_->sampler().next_seq() : sampler_->next_seq();
        return queued_before_boot_ + current;
    }

    /// Told of every transmit's end by whichever node is running, including
    /// one booted after a watchdog reset.
    void set_tx_listener(ITxListener *listener) {
        tx_listener_ = listener;
        if (node_) {
            node_->set_tx_listener(listener);
        } else {
            sampler_->set_tx_listener(listener);
        }
    }

    /// Receives the gateway's lines, exactly as the board would print them,
    /// with the rig's clock. RSSI is 0: nothing models it.
    void set_gateway_log(std::function<void(uint32_t t_ms, const char *line)> sink) {
        gateway_log_ = sink;
    }

    bool tx_in_flight() const {
        return node_ ? node_->sampler().tx_in_flight() : sampler_->tx_in_flight();
    }

    size_t decode_failures() const { return decode_failures_; }

    /// Watchdog resets that struck with a transmit in flight. On hardware the
    /// frame could still complete with no TX line ever printed for it; in the
    /// duty-cycled build the IMU (the only fault that hangs) and the radio are
    /// never powered together, so this stays 0 and the transcripts assert it.
    size_t resets_mid_transmit() const { return resets_mid_transmit_; }

    /// Accepted frames that no TX line will match: frames whose
    /// (node_id, boot_count, seq) the channel altered without the CRC noticing.
    size_t unmatched_accepted() const { return unmatched_accepted_; }

    uint32_t tx_timeouts_total() const {
        const uint32_t current =
            node_ ? node_->sampler().tx_timeouts() : sampler_->tx_timeouts();
        return timeouts_before_boot_ + current;
    }

    /// Scheduled faults, in add_fault() order, and whether each has begun.
    /// A run that never reached a fault's start did not measure that fault.
    size_t fault_count() const { return faults_.size(); }
    bool fault_started(size_t i) const { return faults_[i].started; }
    uint32_t fault_start_ms(size_t i) const { return faults_[i].start_ms; }

  private:
    struct Fault {
        FaultKind kind;
        uint32_t start_ms;
        uint32_t end_ms;  ///< 0: never ends by itself
        bool started;
        bool ended;
    };

    static DutyCycledNodeConfig config() {
        DutyCycledNodeConfig c;
        c.node_id = kRigNodeId;
        c.ttl = kRigTtl;
        return c;
    }

    void boot() {
        node_.reset(new DutyCycledNode(gps_, imu_, radio_, clock_, power_, watchdog_, store_,
                                       config()));
        node_->set_tx_listener(tx_listener_);
        node_->begin();
    }

    /// What a watchdog reset does: RAM is lost, the SNVS store and the
    /// peripherals keep their state, and WDOG1 is disabled until begin().
    void reboot() {
        ++reboots_;
        if (node_->sampler().tx_in_flight() || radio_.transmitting()) {
            ++resets_mid_transmit_;
        }
        queued_before_boot_ += node_->sampler().next_seq();
        timeouts_before_boot_ += node_->sampler().tx_timeouts();
        node_.reset();
        watchdog_.on_mcu_reset();
        boot();
    }

    void step_once() {
        apply_faults();
        if (node_) {
            node_->step();
        } else {
            sampler_->step();
        }
        collect_delivery();
        if (node_ && watchdog_.expired()) {
            reboot();
        }
    }

    void collect_delivery() {
        if (radio_.sent_count() == last_sent_) {
            return;
        }
        last_sent_ = radio_.sent_count();

        // What was transmitted, before the channel, so an accepted frame whose
        // key the channel altered can be recognised.
        Packet sent;
        const bool sent_decodes = decode_packet(radio_.last_payload(), radio_.last_length(), &sent);

        Packet packet;
        const uint8_t *payload = radio_.last_payload();
        size_t length = radio_.last_length();
        uint8_t received[kRigMaxFrame];
        if (channel_ != nullptr) {
            if (length > kRigMaxFrame) {
                length = kRigMaxFrame;
            }
            memcpy(received, payload, length);
            channel_->corrupt(received, length);
            payload = received;
        }
        if (!decode_packet(payload, length, &packet)) {
            ++decode_failures_;
            if (gateway_log_) {
                char line[32];
                const uint16_t count =
                    decode_failures_ > 0xFFFF ? 0xFFFF : static_cast<uint16_t>(decode_failures_);
                if (format_decode_error_line(count, line, sizeof(line)) > 0) {
                    gateway_log_(clock_.now_ms(), line);
                }
            }
            return;
        }
        // The gateway's own dedup, so a key bug would show up as lost records.
        if (dedup_.seen(packet.node_id, packet.boot_count, packet.seq)) {
            return;
        }
        deliveries_.push_back(packet);
        const bool key_altered = sent_decodes && (packet.node_id != sent.node_id ||
                                                  packet.boot_count != sent.boot_count ||
                                                  packet.seq != sent.seq);
        if (key_altered) {
            ++unmatched_accepted_;
        }
        GatewayRecord record;
        record.arrival_ms = clock_.now_ms();
        record.gps_valid = packet.record.gps.valid;
        record.boot_count = packet.boot_count;
        record.tx_timeouts = packet.tx_timeouts;
        records_.push_back(record);
        if (gateway_log_) {
            char line[REC_LINE_MAX];
            if (format_rec_line(packet, 0, line, sizeof(line)) > 0) {
                gateway_log_(clock_.now_ms(), line);
            }
        }
    }

    void apply_faults() {
        const uint32_t now = clock_.now_ms();
        for (size_t i = 0; i < faults_.size(); ++i) {
            Fault &f = faults_[i];
            if (!f.started && now >= f.start_ms) {
                f.started = true;
                begin_fault(f.kind);
            }
            if (f.started && !f.ended && f.end_ms != 0 && now >= f.end_ms) {
                f.ended = true;
                end_fault(f.kind);
            }
        }
    }

    void begin_fault(FaultKind kind) {
        switch (kind) {
        case FaultKind::LostCompletion:
            radio_.lose_next_completion();
            break;
        case FaultKind::SkyBlockage:
            gps_.set_sky_blocked(true);
            break;
        case FaultKind::ImuFailing:
            imu_.set_failing(true);
            break;
        case FaultKind::RadioWedge:
            radio_.set_wedged(true);  // cleared only by a power cycle
            break;
        case FaultKind::Hang:
            if (node_) {
                imu_.hang_next_read(&watchdog_);
            } else {
                imu_.hang_next_read(&end_);
            }
            break;
        }
    }

    void end_fault(FaultKind kind) {
        if (kind == FaultKind::SkyBlockage) {
            gps_.set_sky_blocked(false);
        } else if (kind == FaultKind::ImuFailing) {
            imu_.set_failing(false);
        }
    }

    Build build_;
    SimClock clock_;
    FakeGps gps_;
    FakeImu imu_;
    FakeRadio radio_;
    FakePower power_;
    FakeWatchdog watchdog_;
    FakePersistentStore store_;
    ScenarioEnd end_;
    DedupTable dedup_;
    std::unique_ptr<DutyCycledNode> node_;
    std::unique_ptr<InterruptSampler> sampler_;
    std::vector<Fault> faults_;
    std::vector<Packet> deliveries_;
    std::vector<GatewayRecord> records_;
    size_t last_sent_;
    size_t reboots_;
    NoisyChannel *channel_;
    uint32_t queued_before_boot_;
    ITxListener *tx_listener_;
    std::function<void(uint32_t, const char *)> gateway_log_;
    size_t decode_failures_;
    size_t resets_mid_transmit_;
    size_t unmatched_accepted_;
    uint32_t timeouts_before_boot_;
};

}  // namespace floodnet

#endif  // FLOODNET_TEST_POWER_RIG_HPP
