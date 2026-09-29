#ifndef FLOODNET_TEENSY_WATCHDOG_HPP
#define FLOODNET_TEENSY_WATCHDOG_HPP

#include <Arduino.h>

#include <floodnet/hal/watchdog.hpp>

namespace floodnet {

/// WDOG1, programmed directly from imxrt.h rather than through a library.
///
/// WCR[WT] counts in half seconds: the timeout is (WT + 1) x 0.5 s, up to
/// 128 s (Linux imx2_wdt, IMX2_WDT_MAX_TIME). WDZST is left clear on purpose:
/// suspending the watchdog in low-power modes would let a wake timer that
/// never fires hang the node with nothing to reset it. See the milestone 3
/// design doc, "Watchdog timing". Whether WDOG1 keeps counting through Snooze
/// deepSleep is unverified without a board.
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
