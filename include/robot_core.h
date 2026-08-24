#pragma once
#include <Arduino.h>

#include "pins.h"
#include "serial_protocol.h"
#include "wheel_pid.h"
#include "tb6612.h"
#include "as5600_encoder.h"
#include "mpu6500.h" // Bolder Flight
#include <Adafruit_NeoPixel.h>

class LidarinoRobot {
public:
    LidarinoRobot() 
        : encoderL(Wire), encoderR(Wire1),
          motorL(PIN_MOTOR_L_PWM, PIN_MOTOR_L_IN1, PIN_MOTOR_L_IN2),
          motorR(PIN_MOTOR_R_PWM, PIN_MOTOR_R_IN1, PIN_MOTOR_R_IN2),
          pidL(0.5f, 5.0f, 0.01f, 1.0f),
          pidR(0.5f, 5.0f, 0.01f, 1.0f),
          statusLed(1, PIN_WATCHDOG_LED, NEO_GRB + NEO_KHZ800) {}

    void begin() {
        Serial.begin(115200);

        // Motori ed emergenza
        motorL.begin();
        motorR.begin();
        pinMode(PIN_MOTOR_STBY, OUTPUT);
        digitalWrite(PIN_MOTOR_STBY, HIGH);
        pinMode(PIN_ESTOP, INPUT_PULLUP);

        // Encoder
        encoderL.begin(PIN_I2C0_SDA, PIN_I2C0_SCL);
        encoderR.begin(PIN_I2C1_SDA, PIN_I2C1_SCL);

        // LED di stato
        statusLed.begin();
        statusLed.setBrightness(30); // Usa una luminosità bassa, i neopixel accecano!
        statusLed.setPixelColor(0, statusLed.Color(0, 0, 255)); // Blu durante il setup
        statusLed.show();

        // IMU
        mpu.Config(&Wire, bfs::Mpu6500::I2C_ADDR_PRIM);
        imu_ok_ = mpu.Begin();
        if (!imu_ok_) {
            Serial.println("Error initializing IMU!");
            while (1) { delay(1000); }
            // mpu.setAccelerometerRange(MPU6050_RANGE_2_G);
            // mpu.setGyroRange(MPU6050_RANGE_250_DEG);
            // mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
        } else {
            mpu.ConfigSrd(19); // Output data rate = 1000 / (1 + Srd) = 50 Hz
        }

        uint32_t now = millis();
        last_cmd_time = now; last_pid_time = now; last_tlm_time = now;
    }

    void update() {
        uint32_t now = millis();

        // 1. Seriale asincrona
        processSerial();

        // 2. Sicurezza
        if (now - last_cmd_time > COMMAND_TIMEOUT_MS || digitalRead(PIN_ESTOP) == LOW) {
            target_vel_l = 0.0f;
            target_vel_r = 0.0f;
        }

        // 3. Loop di controllo a 100 Hz
        if (now - last_pid_time >= 10) {
            float dt_sec = (now - last_pid_time) / 1000.0f;
            last_pid_time = now;

            // Aggiorna posizioni interne
            encoderL.update();
            encoderR.update();

            // Legge posizioni cumulative
            int32_t new_pos_l = encoderL.getPositionTicks();
            int32_t new_pos_r = encoderR.getPositionTicks();

            // Calcola velocità istantanea per il PID
            meas_vel_l = ((float)(new_pos_l - pos_left) / AS5600_TICKS_PER_REV) * TWO_PI / dt_sec;
            meas_vel_r = ((float)(new_pos_r - pos_right) / AS5600_TICKS_PER_REV) * TWO_PI / dt_sec;

            pos_left = new_pos_l;
            pos_right = new_pos_r;

            // Applica PID ai driver
            motorL.setCommand(pidL.compute(target_vel_l, meas_vel_l, dt_sec));
            motorR.setCommand(pidR.compute(target_vel_r, meas_vel_r, dt_sec));
        }

        // 4. Telemetria a 50 Hz
        if (now - last_tlm_time >= (1000 / TELEMETRY_RATE_HZ)) {
            last_tlm_time = now;
            sendTelemetry();
        }
        updateLedStatus(now);
    }

private:
    Adafruit_NeoPixel statusLed;
    uint32_t last_led_blink = 0;
    bool led_state = false;
    uint32_t blink_interval = 1000; // ms
    AS5600Encoder encoderL;
    AS5600Encoder encoderR;
    TB6612Motor motorL;
    TB6612Motor motorR;
    WheelPID pidL;
    WheelPID pidR;
    bfs::Mpu6500 mpu;

    bool imu_ok_ = false;

    float target_vel_l = 0.0f, target_vel_r = 0.0f;
    float meas_vel_l = 0.0f, meas_vel_r = 0.0f;
    int32_t pos_left = 0, pos_right = 0;
    float acc_x, acc_y, acc_z;
    float gyro_x, gyro_y, gyro_z;

    uint32_t last_cmd_time = 0, last_pid_time = 0, last_tlm_time = 0;

    uint8_t serial_buf[COMMAND_PACKET_SIZE];
    uint8_t serial_idx = 0;

    void updateLedStatus(uint32_t now) {
        if (now - last_led_blink >= blink_interval) {
            last_led_blink = now;
            led_state = !led_state;

            if (digitalRead(PIN_ESTOP) == LOW) {
                // Emergenza premuta: Rosso
                statusLed.setPixelColor(0, led_state ? statusLed.Color(255, 0, 0) : statusLed.Color(50, 0, 0));
            }
            else if (!encoderL.isOk() || !encoderR.isOk()) {
                // Encoder non ok: Giallo lampeggiante
                statusLed.setPixelColor(0, led_state ? statusLed.Color(255, 255, 0) : statusLed.Color(0, 0, 0));
            }
            else if (!imu_ok_) {
                // IMU non ok: Viola lampeggiante
                statusLed.setPixelColor(0, led_state ? statusLed.Color(128, 0, 128) : statusLed.Color(0, 0, 0));
            }
            else if (now - last_cmd_time > COMMAND_TIMEOUT_MS) {
                // Timeout Seriale: Arancione lampeggiante
                statusLed.setPixelColor(0, led_state ? statusLed.Color(255, 100, 0) : statusLed.Color(0, 0, 0));
            }
            else {
                // Tutto ok: Verde
                statusLed.setPixelColor(0, led_state ? statusLed.Color(0, 255, 0) : statusLed.Color(0, 50, 0));
            }
            statusLed.show();
        }
    }

    void processSerial() {
        while (Serial.available() > 0) {
            serial_buf[serial_idx++] = Serial.read();

            if (serial_idx == COMMAND_PACKET_SIZE) {
                if (serial_buf[0] == PROTO_SYNC_0 && serial_buf[1] == PROTO_SYNC_1 && serial_buf[COMMAND_PACKET_SIZE - 1] == 0x0D) {
                    if (compute_checksum(serial_buf, COMMAND_PACKET_SIZE - 2) == serial_buf[COMMAND_PACKET_SIZE - 2]) {
                        CommandPacket* cmd = (CommandPacket*)serial_buf;
                        target_vel_l = cmd->vel_left / 1000.0f;
                        target_vel_r = cmd->vel_right / 1000.0f;
                        last_cmd_time = millis(); 
                        serial_idx = 0;
                        continue;
                    }
                }
                memmove(serial_buf, serial_buf + 1, COMMAND_PACKET_SIZE - 1);
                serial_idx--;
            }
        }
    }

    void sendTelemetry() {
        TelemetryPacket tp;
        tp.sync0 = PROTO_SYNC_0;
        tp.sync1 = PROTO_SYNC_1;
        tp.pos_left = pos_left;
        tp.pos_right = pos_right;
        
        tp.status_flags = 0;
        if (digitalRead(PIN_ESTOP) == LOW) tp.status_flags |= STATUS_BIT_ESTOP;
        if (encoderL.isOk()) tp.status_flags |= STATUS_BIT_ENCODER_L_OK;
        if (encoderR.isOk()) tp.status_flags |= STATUS_BIT_ENCODER_R_OK;
        if (imu_ok_) tp.status_flags |= STATUS_BIT_IMU_OK;

        updateIMU();
        tp.accel_x = (int16_t)(acc_x * 1000.0f);
        tp.accel_y = (int16_t)(acc_y * 1000.0f);
        tp.accel_z = (int16_t)(acc_z * 1000.0f);
        tp.gyro_x = (int16_t)(gyro_x * 1000.0f);
        tp.gyro_y = (int16_t)(gyro_y * 1000.0f);
        tp.gyro_z = (int16_t)(gyro_z * 1000.0f);

        // sensor_event_t a, g, temp;
        // mpu.getEvent(&a, &g, &temp);
        // tp.accel_x = (int16_t)(a.acceleration.x * 1000);
        // tp.accel_y = (int16_t)(a.acceleration.y * 1000);
        // tp.accel_z = (int16_t)(a.acceleration.z * 1000);
        // tp.gyro_x = (int16_t)(g.gyro.x * 1000);
        // tp.gyro_y = (int16_t)(g.gyro.y * 1000);
        // tp.gyro_z = (int16_t)(g.gyro.z * 1000);

        tp.checksum = compute_checksum((uint8_t*)&tp, TELEMETRY_PACKET_SIZE - 2);
        tp.terminator = 0x0D;

        Serial.write((uint8_t*)&tp, TELEMETRY_PACKET_SIZE);
    }

    void updateIMU() {
        if (mpu.Read()) {
            // Estrazione dati accelerometro (in m/s^2)
            acc_x = mpu.accel_x_mps2();
            acc_y = mpu.accel_y_mps2();
            acc_z = mpu.accel_z_mps2();
            
            // Estrazione dati giroscopio (in rad/s)
            gyro_x = mpu.gyro_x_radps();
            gyro_y = mpu.gyro_y_radps();
            gyro_z = mpu.gyro_z_radps();
        }
    }
};