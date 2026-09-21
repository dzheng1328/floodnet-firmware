#ifndef FLOODNET_MESH_HPP
#define FLOODNET_MESH_HPP

#include <stddef.h>
#include <stdint.h>

#include <floodnet/packet.hpp>

namespace floodnet {

/// Sightings retained per node. Fixed and small: nodes have no heap, and a
/// packet older than this many sightings is no longer worth suppressing.
const size_t DEDUP_CAPACITY = 32;

/// Remembers recently seen (node_id, seq) pairs, evicting the oldest first.
class DedupTable {
  public:
    DedupTable();

    /// Records the pair and reports whether it had already been recorded.
    bool seen(uint16_t node_id, uint32_t seq);

  private:
    struct Entry {
        uint16_t node_id;
        uint32_t seq;
        bool used;
    };

    Entry entries_[DEDUP_CAPACITY];
    size_t next_;
};

/// True when `p` still has hops left and has not been seen before.
/// An expired packet is rejected without consuming a table slot.
bool should_relay(DedupTable &table, const Packet &p);

/// Decrements `p->ttl`. Returns false once the packet has no hops remaining.
bool prepare_relay(Packet *p);

}  // namespace floodnet

#endif  // FLOODNET_MESH_HPP
