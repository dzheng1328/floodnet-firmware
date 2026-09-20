#ifndef FLOODNET_HAL_GPS_HPP
#define FLOODNET_HAL_GPS_HPP

#include <stdint.h>

namespace floodnet {

class IGpsSource {
  public:
    virtual ~IGpsSource() {}

    /// Returns the next buffered byte, or -1 when none is available.
    virtual int read_byte() = 0;

    /// Bytes the receive hardware discarded because its FIFO was full.
    virtual uint16_t bytes_dropped() const = 0;
};

}  // namespace floodnet

#endif  // FLOODNET_HAL_GPS_HPP
