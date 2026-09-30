#ifndef FLOODNET_TEENSY_WATCHDOG_HPP
#define FLOODNET_TEENSY_WATCHDOG_HPP

#include <Arduino.h>

#include <floodnet/hal/watchdog.hpp>

namespace floodnet {

/// WDOG1, programmed directly from imxrt.h rather than through a library.
///
/// WCR[WT] counts in half seconds: the timeout is (WT + 1) x 0.5 s, up to
/// 128 s (Linux imx2_wdt, IMX2_WDT_MAX_TIME).
///
/// Both low-power suspend bits are left clear on purpose, because suspending
/// the watchdog would let a wake timer that never fires hang the node with
/// nothing to reset it: WCR[WDW] (bit 7), which suspends WDOG1 in WAIT mode,
/// and WCR[WDZST] (bit 0), which suspends it in STOP and DOZE. Snooze 6.3.9's
/// hal_deepSleep() enters WAIT (src/hal/TEENSY_40/hal.c:789 sets
/// CCM_CLPCR_LPM(0x01)), so WDW is the bit that governs this build. The WDW to
/// WAIT mapping is read from the RT1060 reference manual's WDOG_WCR
/// description, not verified on a board. See the milestone 3 design doc,
/// "Watchdog timing".
///
/// Unverified without a board, and the first bench test: whether WDOG1 counts
/// at all during deepSleep. hal_deepSleep() rewrites CCM_CCGR3 (hal.c:805),
/// clearing the WDOG1 clock gate, and restores it only on wake (hal.c:851).
class TeensyWatchdog : public IWatchdog {
  public:
    void begin(uint32_t timeout_ms) override {
        CCM_CCGR3 |= CCM_CCGR3_WDOG1(CCM_CCGR_ON);
        const uint32_t half_seconds = timeout_ms / 500;
        const uint32_t wt = half_seconds == 0 ? 0 : (half_seconds > 256 ? 255 : half_seconds - 1);
        WDOG1_WMCR = 0;  // power-down counter off: it would reset us after 16 s
        WDOG1_WCR = WDOG_WCR_WDE | WDOG_WCR_SRS | WDOG_WCR_WDA |
                    WDOG_WCR_WT(static_cast<uint8_t>(wt));
        kick();
    }

    void kick() override {
        // The documented service sequence.
        WDOG1_WSR = 0x5555;
        WDOG1_WSR = 0xAAAA;
    }
};

}  // namespace floodnet

#endif  // FLOODNET_TEENSY_WATCHDOG_HPP
