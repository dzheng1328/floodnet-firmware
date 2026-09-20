#ifndef FLOODNET_HAL_CLOCK_HPP
#define FLOODNET_HAL_CLOCK_HPP

#include <stdint.h>

namespace floodnet {

class IClock {
  public:
    virtual ~IClock() {}
    virtual uint32_t now_ms() = 0;
    virtual void delay_ms(uint32_t ms) = 0;
};

}  // namespace floodnet

#endif  // FLOODNET_HAL_CLOCK_HPP
