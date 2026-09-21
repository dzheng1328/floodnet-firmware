#ifndef FLOODNET_RADIO_CONFIG_HPP
#define FLOODNET_RADIO_CONFIG_HPP

#include <stdint.h>

namespace floodnet {

// Shared between src/main.cpp and src/gateway_main.cpp. A node and the
// gateway that must hear it need identical frequency and pin wiring; two
// independent copies of these constants would let one entry point's value
// drift from the other's with no build error, and the failure would only
// show up as silence in the field.
const float RADIO_FREQUENCY_MHZ = 915.0f;
const int8_t RADIO_TX_POWER_DBM = 20;

const uint8_t RADIO_CS_PIN = 10;
const uint8_t RADIO_RESET_PIN = 9;
const uint8_t RADIO_DIO0_PIN = 2;

}  // namespace floodnet

#endif  // FLOODNET_RADIO_CONFIG_HPP
