#include <Arduino.h>

#include "hal/teensy_clock.hpp"
#include "hal/teensy_gps.hpp"
#include "hal/teensy_imu.hpp"
#include "hal/teensy_radio.hpp"
#include "sampler_polling.hpp"

namespace {

const uint16_t NODE_ID = 1;
const uint8_t PACKET_TTL = 3;
const uint32_t GPS_BAUD = 9600;
const float RADIO_FREQUENCY_MHZ = 915.0f;
const int8_t RADIO_TX_POWER_DBM = 20;

const uint8_t RADIO_CS_PIN = 10;
const uint8_t RADIO_RESET_PIN = 9;
const uint8_t RADIO_DIO0_PIN = 2;

floodnet::TeensyClock g_clock;
floodnet::TeensyGps g_gps(Serial1);
floodnet::TeensyImu g_imu;
floodnet::TeensyRadio g_radio(RADIO_CS_PIN, RADIO_DIO0_PIN, RADIO_RESET_PIN);

floodnet::PollingSampler g_sampler(g_gps, g_imu, g_radio, g_clock, NODE_ID, PACKET_TTL);

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

    Serial.println("floodnet node: polling sampler");
}

void loop() {
    g_gps.note_buffer_state();
    g_sampler.step();
}
