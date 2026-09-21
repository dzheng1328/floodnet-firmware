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

/// A radio that can start a transmission and hand control straight back.
///
/// IRadio keeps its blocking transmit() unchanged and PollingSampler keeps
/// taking IRadio&, so milestone 1's build is untouched by milestone 2. That is
/// deliberate: it makes the published baseline valid by construction instead
/// of something a re-measurement has to establish.
class IAsyncRadio : public IRadio {
  public:
    /// Starts a transmission and returns immediately. False if the radio
    /// refused, or if a transmission is already in flight.
    virtual bool begin_transmit(const uint8_t *data, size_t len) = 0;

    /// True while a transmission started by begin_transmit() is still on the
    /// air. Not const: RadioHead's RHGenericDriver::mode() is a non-const
    /// virtual, and wrapping that in a const method would require a cast that
    /// buys nothing.
    virtual bool tx_busy() = 0;
};

}  // namespace floodnet

#endif  // FLOODNET_HAL_RADIO_HPP
