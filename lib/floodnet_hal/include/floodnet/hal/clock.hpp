#ifndef FLOODNET_HAL_CLOCK_HPP
#define FLOODNET_HAL_CLOCK_HPP

#include <stdint.h>

namespace floodnet {

class IClock {
  public:
    virtual ~IClock() {}
    virtual uint32_t now_ms() = 0;
    virtual void delay_ms(uint32_t ms) = 0;

    /// Yield until an interrupt has something for us, or `max_ms` has passed,
    /// whichever comes first. May return early and may return with nothing to
    /// do, so a caller re-checks its sources on return rather than assuming
    /// work is waiting.
    ///
    /// This is not a simulation convenience. On Teensy it is __WFI(): stop the
    /// core until an interrupt wakes it. It is also the primitive the duty
    /// cycling planned for milestone 3 will build on.
    virtual void wait_for_event(uint32_t max_ms) = 0;
};

}  // namespace floodnet

#endif  // FLOODNET_HAL_CLOCK_HPP
