#ifndef FLOODNET_HAL_TX_LISTENER_HPP
#define FLOODNET_HAL_TX_LISTENER_HPP

#include <stdint.h>

namespace floodnet {

/// Told when a transmit ends. `completed` is true when the radio confirmed
/// the frame and false when the sampler abandoned it after its timeout; an
/// abandoned frame may or may not have reached the air.
class ITxListener {
  public:
    virtual ~ITxListener() {}
    virtual void on_tx_end(uint16_t node_id, uint16_t boot_count, uint32_t seq, uint32_t now_ms,
                           bool completed) = 0;
};

}  // namespace floodnet

#endif  // FLOODNET_HAL_TX_LISTENER_HPP
