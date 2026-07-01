#pragma once
#include <Wire.h>
#include "pins.h"
#include "serial_protocol.h" // Per avere AS5600_TICKS_PER_REV

#define AS5600_I2C_ADDR 0x36 // Indirizzo fisso dell'AS5600

class AS5600Encoder {
public:
    AS5600Encoder(TwoWire& wire) : wire_(wire), last_raw_angle_(0),
                                   turns_(0), initialized_(false), ok_(false) {}

    bool begin(int sda_pin, int scl_pin) {
        wire_.begin(sda_pin, scl_pin, 400000);  // 400kHz I2C fast mode
        wire_.beginTransmission(AS5600_I2C_ADDR);
        ok_ = (wire_.endTransmission() == 0);
        return ok_;
    }

    void update() {
        uint16_t raw = readRawAngle();
        if (raw == 0xFFFF) {
            ok_ = false;
            return;
        }
        ok_ = true;

        if (!initialized_) {
            last_raw_angle_ = raw;
            initialized_ = true;
            return;
        }

        int16_t delta = (int16_t)raw - (int16_t)last_raw_angle_;
        if (delta > (AS5600_TICKS_PER_REV / 2)) {
            turns_--; 
        } else if (delta < -(AS5600_TICKS_PER_REV / 2)) {
            turns_++; 
        }
        last_raw_angle_ = raw;
    }

    int32_t getPositionTicks() const {
        return (int32_t)turns_ * AS5600_TICKS_PER_REV + (int32_t)last_raw_angle_;
    }

    bool isOk() const { return ok_; }

private:
    uint16_t readRawAngle() {
        wire_.beginTransmission(AS5600_I2C_ADDR);
        wire_.write(0x0C); 
        if (wire_.endTransmission(false) != 0) return 0xFFFF;
        
        wire_.requestFrom((uint8_t)AS5600_I2C_ADDR, (uint8_t)2);
        if (wire_.available() < 2) return 0xFFFF;
        
        uint8_t high = wire_.read();
        uint8_t low = wire_.read();
        return ((uint16_t)(high & 0x0F) << 8) | low;
    }

    TwoWire& wire_;
    uint16_t last_raw_angle_;
    int32_t turns_;
    bool initialized_;
    bool ok_;
};