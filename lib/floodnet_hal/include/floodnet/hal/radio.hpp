#ifndef FLOODNET_HAL_RADIO_HPP
#define FLOODNET_HAL_RADIO_HPP

#include <stddef.h>
#include <stdint.h>

namespace floodnet {

class IRadio {
  public:
    virtual ~IRadio() {}

    /// Blocking transmit. Returns false if the radio reported a failure.
    virtual bool transmit(const uint8_t *data, size_t len) = 0;

    /// Non-blocking receive. Returns bytes written to `buf`, or -1 if nothing waited.
    virtual int receive(uint8_t *buf, size_t len) = 0;
};

}  // namespace floodnet

#endif  // FLOODNET_HAL_RADIO_HPP
