#ifndef FLOODNET_HAL_GPS_HPP
#define FLOODNET_HAL_GPS_HPP

#include <stdint.h>

namespace floodnet {

class IGpsSource {
  public:
    virtual ~IGpsSource() {}

    /// Returns the next buffered byte, or -1 when none is available.
    virtual int read_byte() = 0;

    /// Receive-buffer overflow indications. The unit is implementation-defined
    /// and deliberately so: the simulation counts one per byte it discards,
    /// because it knows exactly how many it discarded, while the Teensy UART
    /// counts one per poll that found the buffer saturated, because the
    /// hardware does not expose a lost-byte count and cannot know. Compare
    /// values only between runs of the SAME implementation. A simulated figure
    /// and a hardware figure are not the same quantity.
    virtual uint16_t rx_overflows() const = 0;
};

}  // namespace floodnet

#endif  // FLOODNET_HAL_GPS_HPP
