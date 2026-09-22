#ifndef FLOODNET_PACKET_QUEUE_HPP
#define FLOODNET_PACKET_QUEUE_HPP

#include <stddef.h>
#include <stdint.h>

#include <floodnet/packet.hpp>

namespace floodnet {

/// Packets waiting for the radio.
///
/// Produced and consumed entirely in main context, so unlike `RingBuffer` it
/// is under no single-producer single-consumer constraint and is free to
/// discard whichever end is less useful. It discards the OLDEST.
///
/// That is the right choice for position data specifically: a receiver would
/// rather have where the node is now than where it was several seconds ago.
/// At SF12 a node produces fixes roughly forty times faster than the radio can
/// send them, so something must be discarded, and this makes that choice
/// explicit and counted instead of letting it happen silently in a UART buffer.
template <size_t Capacity>
class PacketQueue {
    static_assert(Capacity > 0, "capacity must be greater than zero");

  public:
    PacketQueue() : head_(0), count_(0), drops_(0), high_water_(0) {}

    /// Always accepts `p`. When the queue is full the oldest entry is
    /// discarded first and `drops()` increments.
    void push(const Packet &p) {
        if (count_ == Capacity) {
            head_ = (head_ + 1) % Capacity;
            --count_;
            if (drops_ < 0xFFFF) {
                ++drops_;
            }
        }
        slots_[(head_ + count_) % Capacity] = p;
        ++count_;
        if (count_ > high_water_) {
            high_water_ = count_;
        }
    }

    /// False when the queue was empty, leaving `*out` untouched.
    bool pop(Packet *out) {
        if (count_ == 0) {
            return false;
        }
        *out = slots_[head_];
        head_ = (head_ + 1) % Capacity;
        --count_;
        return true;
    }

    bool empty() const { return count_ == 0; }
    size_t size() const { return count_; }

    /// Packets discarded because the queue was full, saturating at 65535.
    uint16_t drops() const { return drops_; }

    /// Deepest occupancy this queue has reached.
    size_t high_water() const { return high_water_; }

    static size_t capacity() { return Capacity; }

  private:
    Packet slots_[Capacity];
    size_t head_;
    size_t count_;
    uint16_t drops_;
    size_t high_water_;
};

}  // namespace floodnet

#endif  // FLOODNET_PACKET_QUEUE_HPP
