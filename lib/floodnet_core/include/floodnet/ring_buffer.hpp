#ifndef FLOODNET_RING_BUFFER_HPP
#define FLOODNET_RING_BUFFER_HPP

#include <stddef.h>
#include <stdint.h>

#include <atomic>

namespace floodnet {

/// Lock-free single-producer single-consumer ring buffer.
///
/// NOT ON ANY DATA PATH IN THIS FIRMWARE.
///
/// Milestone 2's design mapped each acquisition path onto the real hardware
/// and found no consumer for this: `HardwareSerial` owns the GPS receive
/// interrupt, RadioHead owns the radio's, the IMU handler can only set a flag
/// because retrieving a sample needs a blocking I2C transaction, and the
/// outbound queue needs drop-oldest, which SPSC ordering forbids. It is built
/// and tested on its own merits. If it ever appears on a data path without
/// this notice being removed, that is a bug. See
/// docs/superpowers/specs/2026-09-21-milestone-2-interrupt-acquisition-design.md.
///
/// Safe for one producer and one consumer running concurrently, including a
/// producer in interrupt context, with no lock and no disabling of interrupts.
/// It is NOT safe for two producers or two consumers.
///
/// A full buffer discards the NEWEST element. Discarding the oldest would
/// require the producer to advance the consumer's index, which is exactly what
/// the single-producer single-consumer discipline forbids.
template <typename T, size_t Capacity>
class RingBuffer {
    static_assert(Capacity > 1, "capacity must be greater than one");
    static_assert((Capacity & (Capacity - 1)) == 0, "capacity must be a power of two");

  public:
    RingBuffer() : head_(0), tail_(0), drops_(0), high_water_(0) {}

    /// Producer side. False when the buffer was full, in which case `value`
    /// was discarded and `drops()` incremented.
    bool push(const T &value) {
        const uint32_t head = head_.load(std::memory_order_relaxed);
        const uint32_t tail = tail_.load(std::memory_order_acquire);

        // Unsigned subtraction, so this stays correct across index wraparound.
        if (head - tail >= Capacity) {
            uint16_t drop_count = drops_.load(std::memory_order_relaxed);
            if (drop_count < 0xFFFF) {
                drops_.store(drop_count + 1, std::memory_order_relaxed);
            }
            return false;
        }

        slots_[head & MASK] = value;
        // Release: the slot write above must be visible before the consumer
        // can observe the new head and read that slot.
        head_.store(head + 1, std::memory_order_release);

        const size_t used = static_cast<size_t>(head + 1 - tail);
        const size_t current_high_water = high_water_.load(std::memory_order_relaxed);
        if (used > current_high_water) {
            high_water_.store(used, std::memory_order_relaxed);
        }
        return true;
    }

    /// Consumer side. False when the buffer was empty, leaving `*out` untouched.
    bool pop(T *out) {
        const uint32_t tail = tail_.load(std::memory_order_relaxed);
        // Acquire: pairs with the producer's release so the slot read below
        // sees the fully written value.
        if (tail == head_.load(std::memory_order_acquire)) {
            return false;
        }

        *out = slots_[tail & MASK];
        tail_.store(tail + 1, std::memory_order_release);
        return true;
    }

    bool empty() const {
        return head_.load(std::memory_order_acquire) == tail_.load(std::memory_order_acquire);
    }

    size_t size() const {
        return static_cast<size_t>(head_.load(std::memory_order_acquire) -
                                   tail_.load(std::memory_order_acquire));
    }

    /// Elements discarded because the buffer was full, saturating at 65535.
    /// Relaxed atomic: may be stale but prevents compiler caching.
    uint16_t drops() const { return drops_.load(std::memory_order_relaxed); }

    /// Deepest occupancy this buffer has reached.
    /// Relaxed atomic: may be stale but prevents compiler caching.
    size_t high_water() const { return high_water_.load(std::memory_order_relaxed); }

    static size_t capacity() { return Capacity; }

  private:
    static const uint32_t MASK = static_cast<uint32_t>(Capacity - 1);

    T slots_[Capacity];
    std::atomic<uint32_t> head_;
    std::atomic<uint32_t> tail_;
    std::atomic<uint16_t> drops_;
    std::atomic<size_t> high_water_;
};

}  // namespace floodnet

#endif  // FLOODNET_RING_BUFFER_HPP
