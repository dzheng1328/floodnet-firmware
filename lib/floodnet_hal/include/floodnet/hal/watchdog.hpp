#ifndef FLOODNET_HAL_WATCHDOG_HPP
#define FLOODNET_HAL_WATCHDOG_HPP

#include <stdint.h>

namespace floodnet {

/// A hardware timer that resets the MCU unless kicked within its timeout.
class IWatchdog {
  public:
    virtual ~IWatchdog() {}
    virtual void begin(uint32_t timeout_ms) = 0;
    virtual void kick() = 0;
};

}  // namespace floodnet

#endif  // FLOODNET_HAL_WATCHDOG_HPP
