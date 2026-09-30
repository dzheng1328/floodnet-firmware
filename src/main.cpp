#include <Arduino.h>

#include "hal/teensy_clock.hpp"
#include "hal/teensy_gps.hpp"
#include "hal/teensy_imu.hpp"
#include "hal/teensy_radio.hpp"
#include "radio_config.hpp"

#if defined(FLOODNET_NODE_DUTY_CYCLED)
#include "duty_cycled_node.hpp"
#include "hal/teensy_persistent_store.hpp"
#include "hal/teensy_power.hpp"
#include "hal/teensy_watchdog.hpp"
#elif defined(FLOODNET_SAMPLER_INTERRUPT)
#include "sampler_interrupt.hpp"
#elif defined(FLOODNET_SAMPLER_POLLING)
#include "sampler_polling.hpp"
#else
#error "Define exactly one of FLOODNET_SAMPLER_POLLING, FLOODNET_SAMPLER_INTERRUPT or FLOODNET_NODE_DUTY_CYCLED"
#endif

namespace {

const uint16_t NODE_ID = 1;
const uint8_t PACKET_TTL = 3;
const uint32_t GPS_BAUD = 9600;

using floodnet::RADIO_CS_PIN;
using floodnet::RADIO_DIO0_PIN;
using floodnet::RADIO_FREQUENCY_MHZ;
using floodnet::RADIO_RESET_PIN;
using floodnet::RADIO_TX_POWER_DBM;

floodnet::TeensyClock g_clock;
floodnet::TeensyGps g_gps(Serial1);
floodnet::TeensyImu g_imu;
floodnet::TeensyRadio g_radio(RADIO_CS_PIN, RADIO_DIO0_PIN, RADIO_RESET_PIN);

#if defined(FLOODNET_NODE_DUTY_CYCLED)
floodnet::TeensyPower g_power(g_gps, g_imu, g_radio, RADIO_FREQUENCY_MHZ, RADIO_TX_POWER_DBM);
floodnet::TeensyWatchdog g_watchdog;
floodnet::TeensyPersistentStore g_store;
floodnet::DutyCycledNode g_node(g_gps, g_imu, g_radio, g_clock, g_power, g_watchdog, g_store,
                                floodnet::DutyCycledNodeConfig{NODE_ID, PACKET_TTL});
const char *const SAMPLER_NAME = "duty-cycled";
#elif defined(FLOODNET_SAMPLER_INTERRUPT)
floodnet::InterruptSampler g_sampler(g_gps, g_imu, g_radio, g_clock, NODE_ID, PACKET_TTL);
const char *const SAMPLER_NAME = "interrupt";
#else
floodnet::PollingSampler g_sampler(g_gps, g_imu, g_radio, g_clock, NODE_ID, PACKET_TTL);
const char *const SAMPLER_NAME = "polling";
#endif

}  // namespace

void setup() {
    Serial.begin(115200);
    g_gps.begin(GPS_BAUD);

    if (!g_imu.begin()) {
        Serial.println("imu: init failed");
    }
    if (!g_radio.begin(RADIO_FREQUENCY_MHZ, RADIO_TX_POWER_DBM)) {
        Serial.println("radio: init failed");
    }

    Serial.print("floodnet node: ");
    Serial.print(SAMPLER_NAME);
    Serial.println(" sampler");

#if defined(FLOODNET_NODE_DUTY_CYCLED)
    g_node.begin();
#endif
}

void loop() {
    g_gps.note_buffer_state();
#if defined(FLOODNET_NODE_DUTY_CYCLED)
    g_node.step();
#else
    g_sampler.step();
#endif
}
