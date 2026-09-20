#ifndef FLOODNET_TEENSY_CLOCK_HPP
#define FLOODNET_TEENSY_CLOCK_HPP

#include <Arduino.h>

#include <floodnet/hal/clock.hpp>

namespace floodnet {

class TeensyClock : public IClock {
  public:
    uint32_t now_ms() override { return millis(); }
    void delay_ms(uint32_t ms) override { delay(ms); }
};

}  // namespace floodnet

#endif  // FLOODNET_TEENSY_CLOCK_HPP
