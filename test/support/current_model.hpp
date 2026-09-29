#ifndef FLOODNET_TEST_CURRENT_MODEL_HPP
#define FLOODNET_TEST_CURRENT_MODEL_HPP

#include <stdint.h>

namespace floodnet {

/// Supply currents, in nanoamps so the RFM95W's 0.2 uA sleep current is
/// representable. Taken at face value from the battery: regulator efficiency
/// and quiescent current are not modelled (milestone 3 design doc, "Energy
/// accounting"). Every value is cited; none may be changed to move a result.
struct CurrentModel {
    /// PJRC Teensy 4.1 product page, "approximately 100 mA" at 600 MHz (the
    /// sentence names the Teensy 4.0: same MCU, same clock).
    uint32_t teensy_awake_nA = 100000000;
    /// PJRC forum, "Teensy 4.1 deep sleep and watchdog": 6 mA from 5 V in
    /// Snooze deepSleep. A user measurement, not a data sheet value.
    uint32_t teensy_sleep_nA = 6000000;
    /// NEO-M8 data sheet UBX-15031086, Table 11, NEO-M8N, GPS, at 3 V.
    uint32_t gps_acquisition_nA = 25000000;
    uint32_t gps_tracking_nA = 23000000;  ///< same table, continuous mode
    uint32_t gps_backup_nA = 30000;       ///< same data sheet, I_SWBCKP at VCC = 3 V
    /// BNO055 data sheet BST-BNO055-DS000: normal mode, and suspend mode.
    uint32_t imu_normal_nA = 12300000;
    uint32_t imu_suspend_nA = 40000;
    /// RFM95/96/97/98W data sheet, Table 51.
    uint32_t radio_tx_nA = 120000000;    ///< +20 dBm on PA_BOOST
    uint32_t radio_standby_nA = 1600000; ///< IDDST
    uint32_t radio_sleep_nA = 200;       ///< IDDSL

    static CurrentModel with_teensy_sleep(uint32_t sleep_nA) {
        CurrentModel model;
        model.teensy_sleep_nA = sleep_nA;
        return model;
    }
};

/// The two measured Snooze deepSleep currents; the battery-life result is
/// reported at both because neither is a data sheet value.
const uint32_t TEENSY_SLEEP_LOW_NA = 6000000;    ///< PJRC forum, see above
/// PJRC forum, "Teensy 4.1 using Snooze library with deepSleep": 25.86 mA.
const uint32_t TEENSY_SLEEP_HIGH_NA = 25860000;

/// Panasonic NCR18650B data sheet: rated capacity 3200 mAh (minimum).
const uint64_t NCR18650B_CAPACITY_NA_MS = 3200ULL * 1000000ULL * 3600000ULL;

}  // namespace floodnet

#endif  // FLOODNET_TEST_CURRENT_MODEL_HPP
