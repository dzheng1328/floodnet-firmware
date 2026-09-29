#ifndef FLOODNET_HAL_POWER_HPP
#define FLOODNET_HAL_POWER_HPP

#include <stdint.h>

namespace floodnet {

enum class Peripheral : uint8_t { Gps, Imu, Radio };

enum class PowerState : uint8_t { On, Low };

class IPower {
  public:
    virtual ~IPower() {}

    /// `Low` is each part's own low-power command, not a load switch: backup
    /// for the GPS (keeps ephemeris, so the next wake is a hot start), suspend
    /// for the IMU, sleep for the radio.
    virtual void set_power(Peripheral p, PowerState s) = 0;

    /// Low, then On, then re-initialise the driver: for a part that has
    /// stopped answering. Re-initialisation is driver-specific, which is why
    /// it lives here. False if re-initialisation failed.
    virtual bool power_cycle(Peripheral p) = 0;

    /// Puts the MCU in its lowest RAM-retaining mode until `wake_ms` on the
    /// IClock timeline. Returns at once if `wake_ms` has passed (wrap-safe).
    /// May return early; callers re-check the time.
    virtual void sleep_until(uint32_t wake_ms) = 0;

    virtual uint16_t battery_mv() = 0;
};

}  // namespace floodnet

#endif  // FLOODNET_HAL_POWER_HPP
