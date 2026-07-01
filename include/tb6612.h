#pragma once
#include <Arduino.h>

// Wrapper per un singolo canale del driver TB6612FNG ottimizzato per ESP32
class TB6612Motor {
public:
    TB6612Motor(uint8_t pwm_pin, uint8_t in1_pin, uint8_t in2_pin)
        : pwm_pin_(pwm_pin), in1_pin_(in1_pin), in2_pin_(in2_pin) {}

    void begin() {
        pinMode(in1_pin_, OUTPUT);
        pinMode(in2_pin_, OUTPUT);

        // API aggiornata per ESP32 Core 3.x+ (PlatformIO recente)
        // 20kHz = frequenza inudibile. 10 bit = 1024 step di risoluzione.
        #if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 0, 0)
            ledcAttach(pwm_pin_, 20000, 10);
        #else
            // Fallback per vecchie versioni del core ESP32
            ledcSetup(pwm_channel_, 20000, 10);
            ledcAttachPin(pwm_pin_, pwm_channel_);
        #endif

        stop();
    }

    // command: -1.0 (indietro pieno) ... 0.0 (fermo) ... 1.0 (avanti pieno)
    void setCommand(float command) {
        if (command > 1.0f) command = 1.0f;
        if (command < -1.0f) command = -1.0f;

        if (command > 0.001f) {
            digitalWrite(in1_pin_, HIGH);
            digitalWrite(in2_pin_, LOW);
        } else if (command < -0.001f) {
            digitalWrite(in1_pin_, LOW);
            digitalWrite(in2_pin_, HIGH);
        } else {
            // Short brake / Coasting
            digitalWrite(in1_pin_, LOW);
            digitalWrite(in2_pin_, LOW);
        }

        uint32_t duty = (uint32_t)(fabs(command) * 1023.0f);
        
        #if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 0, 0)
            ledcWrite(pwm_pin_, duty);
        #else
            ledcWrite(pwm_channel_, duty);
        #endif
    }

    void stop() { setCommand(0.0f); }

private:
    uint8_t pwm_pin_, in1_pin_, in2_pin_;
    // Solo per vecchi core
    uint8_t pwm_channel_ = 0; 
};