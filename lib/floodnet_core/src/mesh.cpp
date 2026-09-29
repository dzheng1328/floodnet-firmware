#include <floodnet/mesh.hpp>

namespace floodnet {

DedupTable::DedupTable() : entries_(), next_(0) {
    for (size_t i = 0; i < DEDUP_CAPACITY; ++i) {
        entries_[i].node_id = 0;
        entries_[i].boot_count = 0;
        entries_[i].seq = 0;
        entries_[i].used = false;
    }
}

bool DedupTable::seen(uint16_t node_id, uint16_t boot_count, uint32_t seq) {
    for (size_t i = 0; i < DEDUP_CAPACITY; ++i) {
        if (entries_[i].used && entries_[i].node_id == node_id &&
            entries_[i].boot_count == boot_count && entries_[i].seq == seq) {
            return true;
        }
    }

    entries_[next_].node_id = node_id;
    entries_[next_].boot_count = boot_count;
    entries_[next_].seq = seq;
    entries_[next_].used = true;
    next_ = (next_ + 1) % DEDUP_CAPACITY;
    return false;
}

bool should_relay(DedupTable &table, const Packet &p) {
    if (p.ttl == 0) {
        return false;
    }
    return !table.seen(p.node_id, p.boot_count, p.seq);
}

bool prepare_relay(Packet *p) {
    if (p == nullptr || p->ttl == 0) {
        return false;
    }
    p->ttl = static_cast<uint8_t>(p->ttl - 1);
    return p->ttl > 0;
}

}  // namespace floodnet
