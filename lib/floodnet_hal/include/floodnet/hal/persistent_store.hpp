#ifndef FLOODNET_HAL_PERSISTENT_STORE_HPP
#define FLOODNET_HAL_PERSISTENT_STORE_HPP

#include <stddef.h>
#include <stdint.h>

namespace floodnet {

/// The i.MX RT1062 has four SNVS low-power general purpose registers.
const size_t PERSISTENT_SLOTS = 4;

/// Words that survive a watchdog reset. Not flash: nothing here wears out.
class IPersistentStore {
  public:
    virtual ~IPersistentStore() {}
    /// 0 for a slot never written or out of range.
    virtual uint32_t read_u32(size_t slot) = 0;
    /// Ignored for a slot out of range.
    virtual void write_u32(size_t slot, uint32_t value) = 0;
};

}  // namespace floodnet

#endif  // FLOODNET_HAL_PERSISTENT_STORE_HPP
