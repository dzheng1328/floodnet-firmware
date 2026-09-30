#include <Arduino.h>

#include <floodnet/gateway_format.hpp>
#include <floodnet/mesh.hpp>
#include <floodnet/packet.hpp>

#include "hal/teensy_radio.hpp"
#include "radio_config.hpp"

namespace {

using floodnet::RADIO_CS_PIN;
using floodnet::RADIO_DIO0_PIN;
using floodnet::RADIO_FREQUENCY_MHZ;
using floodnet::RADIO_RESET_PIN;
using floodnet::RADIO_TX_POWER_DBM;

floodnet::TeensyRadio g_radio(RADIO_CS_PIN, RADIO_DIO0_PIN, RADIO_RESET_PIN);
floodnet::DedupTable g_dedup;
// decode_packet() rejects on bad magic, bad version, or bad CRC and returns a
// single bool, so the three causes are not distinguishable through its
// current return type. Naming this "decode" rather than "crc" avoids
// claiming a cause we cannot actually identify. Separating the three is a
// later milestone's job if it proves necessary.
uint16_t g_decode_errors = 0;
uint16_t g_short_reads = 0;

/// One line per packet, comma separated, so the host pipeline can read it
/// without a framing layer.
void print_record(const floodnet::Packet &p, int16_t rssi) {
    // Formatted in floodnet_core so the simulated gateway writes the same bytes.
    char line[floodnet::REC_LINE_MAX];
    if (floodnet::format_rec_line(p, rssi, line, sizeof(line)) > 0) {
        Serial.println(line);
    }
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
        if (received < 0) {
            return;  // Nothing waiting. Not an error.
        }
        if (g_short_reads < 0xFFFF) {
            ++g_short_reads;
        }
        Serial.print("ERR,short,");
        Serial.print(received);
        Serial.print(',');
        Serial.println(g_short_reads);
        return;
    }

    floodnet::Packet packet;
    if (!floodnet::decode_packet(buffer, floodnet::PACKET_SIZE, &packet)) {
        if (g_decode_errors < 0xFFFF) {
            ++g_decode_errors;
        }
        char line[32];
        if (floodnet::format_decode_error_line(g_decode_errors, line, sizeof(line)) > 0) {
            Serial.println(line);
        }
        return;
    }

    if (g_dedup.seen(packet.node_id, packet.boot_count, packet.seq)) {
        return;
    }

    print_record(packet, g_radio.last_rssi());
}
