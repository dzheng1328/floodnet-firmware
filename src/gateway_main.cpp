#include <Arduino.h>

#include <floodnet/mesh.hpp>
#include <floodnet/packet.hpp>

#include "hal/teensy_clock.hpp"
#include "hal/teensy_radio.hpp"

namespace {

const float RADIO_FREQUENCY_MHZ = 915.0f;
const int8_t RADIO_TX_POWER_DBM = 20;

const uint8_t RADIO_CS_PIN = 10;
const uint8_t RADIO_RESET_PIN = 9;
const uint8_t RADIO_DIO0_PIN = 2;

floodnet::TeensyClock g_clock;
floodnet::TeensyRadio g_radio(RADIO_CS_PIN, RADIO_DIO0_PIN, RADIO_RESET_PIN);
floodnet::DedupTable g_dedup;
uint16_t g_crc_errors = 0;

/// One line per packet, comma separated, so the host pipeline can read it
/// without a framing layer.
void print_record(const floodnet::Packet &p, int16_t rssi) {
    Serial.print("REC,");
    Serial.print(p.node_id);
    Serial.print(',');
    Serial.print(p.seq);
    Serial.print(',');
    Serial.print(p.record.gps.time_ms);
    Serial.print(',');
    Serial.print(p.record.gps.lat_1e7);
    Serial.print(',');
    Serial.print(p.record.gps.lon_1e7);
    Serial.print(',');
    Serial.print(p.record.gps.alt_mm);
    Serial.print(',');
    Serial.print(p.record.gps.satellites);
    Serial.print(',');
    Serial.print(p.record.imu.yaw_cd);
    Serial.print(',');
    Serial.print(p.record.imu.pitch_cd);
    Serial.print(',');
    Serial.print(p.record.imu.roll_cd);
    Serial.print(',');
    Serial.print(p.record.diag.drops);
    Serial.print(',');
    Serial.print(p.record.diag.crc_errors);
    Serial.print(',');
    Serial.println(rssi);
}

}  // namespace

void setup() {
    Serial.begin(115200);
    if (!g_radio.begin(RADIO_FREQUENCY_MHZ, RADIO_TX_POWER_DBM)) {
        Serial.println("radio: init failed");
    }
    Serial.println("floodnet gateway");
}

void loop() {
    uint8_t buffer[floodnet::PACKET_SIZE];
    const int received = g_radio.receive(buffer, sizeof(buffer));
    if (received != static_cast<int>(floodnet::PACKET_SIZE)) {
        return;
    }

    floodnet::Packet packet;
    if (!floodnet::decode_packet(buffer, floodnet::PACKET_SIZE, &packet)) {
        if (g_crc_errors < 0xFFFF) {
            ++g_crc_errors;
        }
        Serial.print("ERR,crc,");
        Serial.println(g_crc_errors);
        return;
    }

    if (g_dedup.seen(packet.node_id, packet.seq)) {
        return;
    }

    print_record(packet, g_radio.last_rssi());
}
