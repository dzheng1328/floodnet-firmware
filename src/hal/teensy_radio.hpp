#ifndef FLOODNET_TEENSY_RADIO_HPP
#define FLOODNET_TEENSY_RADIO_HPP

#include <Arduino.h>
#include <RH_RF95.h>
#include <SPI.h>

#include <floodnet/hal/radio.hpp>

namespace floodnet {

class TeensyRadio : public IAsyncRadio {
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
        // Radio operations must carry timeouts (spec, Error handling). Size
        // generously against the worst case this firmware can program: SF12
        // airtime for a 45-byte packet is about 3.0 s (see
        // test/test_polling/test_main.cpp), so 5000 ms leaves headroom without
        // blocking indefinitely on a wedged modem.
        return driver_.waitPacketSent(5000);
    }

    bool begin_transmit(const uint8_t *data, size_t len) override {
        if (!ready_ || tx_busy()) {
            return false;
        }
        // send() loads the FIFO, switches the modem to transmit and returns.
        // The DIO0 interrupt RadioHead attached during init() clears the mode
        // once the packet is on the air. Milestone 1 spun inside
        // waitPacketSent() waiting for precisely that; here the loop goes back
        // to other work and checks tx_busy() on a later pass.
        return driver_.send(data, static_cast<uint8_t>(len));
    }

    bool tx_busy() override {
        if (!ready_) {
            return false;
        }
        return driver_.mode() == RHGenericDriver::RHModeTx;
    }

    void abort_transmit() override {
        if (!ready_) {
            return;
        }
        // Forces the driver out of RHModeTx whether or not the modem finished,
        // which is the state a lost DIO0 edge leaves behind.
        driver_.setModeIdle();
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
