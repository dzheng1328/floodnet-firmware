#ifndef FLOODNET_TEENSY_RADIO_HPP
#define FLOODNET_TEENSY_RADIO_HPP

#include <Arduino.h>
#include <RH_RF95.h>
#include <SPI.h>

#include <floodnet/hal/radio.hpp>

namespace floodnet {

class TeensyRadio : public IRadio {
  public:
    TeensyRadio(uint8_t cs_pin, uint8_t interrupt_pin, uint8_t reset_pin)
        : driver_(cs_pin, interrupt_pin), reset_pin_(reset_pin), ready_(false) {}

    bool begin(float frequency_mhz, int8_t tx_power_dbm) {
        pinMode(reset_pin_, OUTPUT);
        digitalWrite(reset_pin_, HIGH);
        digitalWrite(reset_pin_, LOW);
        delay(10);
        digitalWrite(reset_pin_, HIGH);
        delay(10);

        ready_ = driver_.init();
        if (!ready_) {
            return false;
        }
        if (!driver_.setFrequency(frequency_mhz)) {
            ready_ = false;
            return false;
        }
        // Long-range configuration: slower, more robust, less throughput.
        driver_.setModemConfig(RH_RF95::Bw125Cr48Sf4096);
        driver_.setTxPower(tx_power_dbm, false);
        return true;
    }

    bool transmit(const uint8_t *data, size_t len) override {
        if (!ready_) {
            return false;
        }
        if (!driver_.send(data, static_cast<uint8_t>(len))) {
            return false;
        }
        return driver_.waitPacketSent();
    }

    int receive(uint8_t *buf, size_t len) override {
        if (!ready_ || !driver_.available()) {
            return -1;
        }
        uint8_t length = static_cast<uint8_t>(len);
        if (!driver_.recv(buf, &length)) {
            return -1;
        }
        return static_cast<int>(length);
    }

    int16_t last_rssi() { return driver_.lastRssi(); }

  private:
    RH_RF95 driver_;
    uint8_t reset_pin_;
    bool ready_;
};

}  // namespace floodnet

#endif  // FLOODNET_TEENSY_RADIO_HPP
