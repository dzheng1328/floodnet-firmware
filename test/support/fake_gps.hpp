#ifndef FLOODNET_TEST_FAKE_GPS_HPP
#define FLOODNET_TEST_FAKE_GPS_HPP

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <floodnet/hal/gps.hpp>

#include "sim_clock.hpp"

namespace floodnet {

/// Emits a sentence repeatedly at a fixed byte rate into a small FIFO,
/// modelling a UART peripheral that discards bytes nobody collected.
/// It can also be powered down into backup, and models the NEO-M8N's time to
/// first fix when powered back up.
class FakeGps : public IGpsSource, public ISimTick {
  public:
    static const size_t DEFAULT_FIFO_DEPTH = 64;
    static const size_t MAX_FIFO_DEPTH = 4096;

    /// What a NEO-M8N sends with no fix: a well-formed GGA with fix quality 0.
    /// parse_gga() accepts it and reports valid == false.
    static constexpr const char *NO_FIX_SENTENCE = "$GPGGA,123519,,,,,0,00,,,M,,M,,*6B\r\n";

    /// NEO-M8 data sheet UBX-15031086, TTFF, GPS + GLONASS.
    static const uint32_t HOT_START_MS = 1000;
    static const uint32_t COLD_START_MS = 26000;

    /// How old the last fix may be for a hot start. The data sheet states no
    /// ephemeris lifetime; this is the milestone 3 spec's stated assumption.
    static const uint32_t EPHEMERIS_LIFETIME_MS = 2UL * 60UL * 60UL * 1000UL;

    FakeGps(const char *sentence, double bytes_per_ms,
            size_t fifo_depth = DEFAULT_FIFO_DEPTH)
        : sentence_(sentence),
          current_(sentence),
          current_len_(strlen(sentence)),
          source_index_(0),
          bytes_per_ms_(bytes_per_ms),
          pending_(0.0),
          fifo_depth_(fifo_depth),
          head_(0),
          tail_(0),
          count_(0),
          dropped_(0),
          powered_(true),
          sky_blocked_(false),
          ttff_ms_(0),
          since_power_on_ms_(0),
          has_fixed_(false),
          since_fix_ms_(0) {
        // Not assert(): this guards the fifo_ indexing below, and a guard
        // that NDEBUG can strip would turn a bad depth into memory corruption.
        if (fifo_depth == 0 || fifo_depth > MAX_FIFO_DEPTH) {
            fprintf(stderr, "FakeGps depth must be between 1 and MAX_FIFO_DEPTH\n");
            abort();
        }
    }

    void on_tick(uint32_t elapsed_ms) override {
        since_fix_ms_ = saturating_add(since_fix_ms_, elapsed_ms);
        if (!powered_) {
            return;  // in backup: sends nothing, but ephemeris keeps ageing
        }
        since_power_on_ms_ = saturating_add(since_power_on_ms_, elapsed_ms);

        pending_ += static_cast<double>(elapsed_ms) * bytes_per_ms_;
        while (pending_ >= 1.0) {
            // Chosen only at a sentence boundary, so a switch never splices
            // two sentences together.
            if (source_index_ == 0) {
                choose_sentence();
            }
            push(current_[source_index_]);
            source_index_ = (source_index_ + 1) % current_len_;
            // Fires the moment the '\r' before the trailing "\r\n" has gone
            // out, not on the wraparound to index 0. NmeaLineAssembler (and
            // any real receiver) completes a sentence on that '\r', one byte
            // before the fake's internal cursor wraps; gating on the wrap
            // itself would leave has_fixed_ false forever in the (common)
            // case where a caller powers the GPS off the instant it observes
            // the fix, since the trailing '\n' would never get sent.
            if (source_index_ == current_len_ - 1 && current_ == sentence_) {
                has_fixed_ = true;
                since_fix_ms_ = 0;
            }
            pending_ -= 1.0;
        }
    }

    int read_byte() override {
        if (count_ == 0) {
            return -1;
        }
        const uint8_t value = fifo_[tail_];
        tail_ = (tail_ + 1) % fifo_depth_;
        --count_;
        return static_cast<int>(value);
    }

    uint16_t rx_overflows() const override { return dropped_; }

    bool pending() const override { return count_ > 0; }

    /// Off models software backup: nothing is sent, ephemeris is kept. On
    /// starts a hot start if the last fix is recent enough, a cold one if not.
    /// Only changes of state do anything.
    void set_powered(bool on) {
        if (on == powered_) {
            return;
        }
        powered_ = on;
        if (on) {
            const bool hot = has_fixed_ && since_fix_ms_ < EPHEMERIS_LIFETIME_MS;
            ttff_ms_ = hot ? HOT_START_MS : COLD_START_MS;
            since_power_on_ms_ = 0;
            pending_ = 0.0;
            source_index_ = 0;
        }
    }

    bool powered() const { return powered_; }

    /// No satellites visible: only no-fix sentences until unblocked.
    void set_sky_blocked(bool blocked) { sky_blocked_ = blocked; }

    /// Powered and not yet producing fixes: the acquisition current applies.
    bool acquiring() const {
        return powered_ && (sky_blocked_ || since_power_on_ms_ < ttff_ms_);
    }

    /// Places bytes in the receive buffer directly, as if left over from an
    /// earlier wake. Test hook.
    void inject(const char *bytes) {
        for (const char *p = bytes; *p != '\0'; ++p) {
            push(*p);
        }
    }

  private:
    static uint32_t saturating_add(uint32_t a, uint32_t b) {
        return a > UINT32_MAX - b ? UINT32_MAX : a + b;
    }

    void choose_sentence() {
        const bool fixing = !sky_blocked_ && since_power_on_ms_ >= ttff_ms_;
        current_ = fixing ? sentence_ : NO_FIX_SENTENCE;
        current_len_ = strlen(current_);
    }

    void push(char value) {
        if (count_ == fifo_depth_) {
            if (dropped_ < 0xFFFF) {
                ++dropped_;
            }
            return;
        }
        fifo_[head_] = static_cast<uint8_t>(value);
        head_ = (head_ + 1) % fifo_depth_;
        ++count_;
    }

    const char *sentence_;
    const char *current_;
    size_t current_len_;
    size_t source_index_;
    double bytes_per_ms_;
    double pending_;

    size_t fifo_depth_;
    uint8_t fifo_[MAX_FIFO_DEPTH];
    size_t head_;
    size_t tail_;
    size_t count_;
    uint16_t dropped_;

    bool powered_;
    bool sky_blocked_;
    uint32_t ttff_ms_;
    uint32_t since_power_on_ms_;
    bool has_fixed_;
    uint32_t since_fix_ms_;
};

}  // namespace floodnet

#endif  // FLOODNET_TEST_FAKE_GPS_HPP
