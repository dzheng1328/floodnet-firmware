#ifndef FLOODNET_TEENSY_CLOCK_HPP
#define FLOODNET_TEENSY_CLOCK_HPP

#include <Arduino.h>

#include <floodnet/hal/clock.hpp>

namespace floodnet {

class TeensyClock : public IClock {
  public:
    uint32_t now_ms() override { return millis(); }
    void delay_ms(uint32_t ms) override { delay(ms); }

    void wait_for_event(uint32_t) override {
        // Sleep the core until any interrupt fires. The systick driving
        // millis() runs at 1 kHz, so this returns within a millisecond even
        // when no peripheral is active. That makes max_ms advisory here
        // rather than something this implementation enforces with a timer,
        // which is why the parameter is unused: the caller's loop re-checks
        // its sources and calls again.
        //
        // Raw instruction rather than CMSIS __WFI(): that macro lives in
        // core_cmInstr.h, which nothing in Arduino.h's include chain reaches
        // on this platform (imxrt.h includes only <stdint.h>). The Teensy 4
        // core itself does exactly this, in avr/sleep.h's sleep_cpu().
        //
        // The memory clobber matters. An interrupt handler is what wakes this,
        // and it is what writes the flags the caller checks on return, so the
        // compiler must not hoist those loads above the sleep.
        __asm__ volatile("wfi" ::: "memory");
    }
};

}  // namespace floodnet

#endif  // FLOODNET_TEENSY_CLOCK_HPP
