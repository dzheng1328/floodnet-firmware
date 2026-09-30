#ifndef FLOODNET_TEST_FAKE_PERSISTENT_STORE_HPP
#define FLOODNET_TEST_FAKE_PERSISTENT_STORE_HPP

#include <floodnet/hal/persistent_store.hpp>

namespace floodnet {

/// Outlives the node it serves, as the SNVS registers outlive a reset.
class FakePersistentStore : public IPersistentStore {
  public:
    FakePersistentStore() : slots_() {}

    uint32_t read_u32(size_t slot) override { return slot < PERSISTENT_SLOTS ? slots_[slot] : 0; }

    void write_u32(size_t slot, uint32_t value) override {
        if (slot < PERSISTENT_SLOTS) {
            slots_[slot] = value;
        }
    }

  private:
    uint32_t slots_[PERSISTENT_SLOTS];
};

}  // namespace floodnet

#endif  // FLOODNET_TEST_FAKE_PERSISTENT_STORE_HPP
