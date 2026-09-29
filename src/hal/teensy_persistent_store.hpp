#ifndef FLOODNET_TEENSY_PERSISTENT_STORE_HPP
#define FLOODNET_TEENSY_PERSISTENT_STORE_HPP

#include <Arduino.h>

#include <floodnet/hal/persistent_store.hpp>

namespace floodnet {

/// The SNVS low-power general purpose registers, retained across resets while
/// SNVS is powered (i.MX RT1060 reference manual, section 19.4.2.1). Registers
/// rather than flash, so writing on every boot wears nothing out. Retention
/// across a WDOG1 reset specifically is unverified on a board.
///
/// WARNING: slot 0 is SNVS_LPGPR0, which the reference manual's SNVS map also
/// exposes at the legacy offset 0x68. imxrt.h names that alias SNVS_LPGPR
/// (cores/teensy4/imxrt.h:8735), and Snooze 6.3.9's SnoozeAlarm writes
/// SNVS_DEFAULT_PGD_VALUE to it (hal/TEENSY_40/SnoozeAlarm.cpp:73). Do not keep
/// anything in slot 0 that must survive a SnoozeAlarm wake; the boot counter
/// uses slot 3 for this reason (DutyCycledNode::BOOT_COUNT_SLOT).
class TeensyPersistentStore : public IPersistentStore {
  public:
    uint32_t read_u32(size_t slot) override {
        switch (slot) {
        case 0:
            return SNVS_LPGPR0;
        case 1:
            return SNVS_LPGPR1;
        case 2:
            return SNVS_LPGPR2;
        case 3:
            return SNVS_LPGPR3;
        default:
            return 0;
        }
    }

    void write_u32(size_t slot, uint32_t value) override {
        switch (slot) {
        case 0:
            SNVS_LPGPR0 = value;
            break;
        case 1:
            SNVS_LPGPR1 = value;
            break;
        case 2:
            SNVS_LPGPR2 = value;
            break;
        case 3:
            SNVS_LPGPR3 = value;
            break;
        default:
            break;
        }
    }
};

}  // namespace floodnet

#endif  // FLOODNET_TEENSY_PERSISTENT_STORE_HPP
