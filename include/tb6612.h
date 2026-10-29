#pragma once
#include <Arduino.h>

// Wrapper per un singolo canale del driver TB6612FNG ottimizzato per ESP32
class TB6612Motor {
public:
    TB6612Motor(uint8_t pwm_pin, uint8_t in1_pin, uint8_t in2_pin, uint8_t pwm_channel = 0)
        : pwm_pin_(pwm_pin), in1_pin_(in1_pin), in2_pin_(in2_pin), pwm_channel_(pwm_channel) {}

    // 9.5 kHz, 12 bit = 4096 step. Il clock LEDC è 40 MHz: con 12 bit la frequenza massima è
    // 40 MHz / 4096 = 9.77 kHz. A 10 kHz / 12 bit ledcSetup() falliva e il PWM restava a 0.
    static constexpr uint32_t PWM_FREQ_HZ = 9500;
    static constexpr uint8_t PWM_RES_BITS = 12;
    static constexpr uint32_t PWM_MAX_DUTY = (1u << PWM_RES_BITS) - 1;

    // Restituisce false se il PWM non si può configurare (motore fermo)
    bool begin() {
        pinMode(in1_pin_, OUTPUT);
        pinMode(in2_pin_, OUTPUT);

        // API aggiornata per ESP32 Core 3.x+ (PlatformIO recente)
        #if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 0, 0)
            bool ok = ledcAttach(pwm_pin_, PWM_FREQ_HZ, PWM_RES_BITS);
        #else
            // Fallback per vecchie versioni del core ESP32
            bool ok = ledcSetup(pwm_channel_, PWM_FREQ_HZ, PWM_RES_BITS) != 0;
            ledcAttachPin(pwm_pin_, pwm_channel_);
        #endif

        stop();
        return ok;
    }

    // command: -1.0 (indietro pieno) ... 0.0 (fermo) ... 1.0 (avanti pieno)
    void setCommand(float command) {
        if (command > 1.0f) command = 1.0f;
        if (command < -1.0f) command = -1.0f;

        if (command > 0.001f) {
            digitalWrite(in1_pin_, LOW);
            digitalWrite(in2_pin_, HIGH);
        } else if (command < -0.001f) {
            digitalWrite(in1_pin_, HIGH);
            digitalWrite(in2_pin_, LOW);
        } else {
            // Short brake / Coasting
            digitalWrite(in1_pin_, LOW);
            digitalWrite(in2_pin_, LOW);
        }

        uint32_t duty = (uint32_t)(fabs(command) * (float)PWM_MAX_DUTY);
        
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
    uint8_t pwm_channel_; 
};