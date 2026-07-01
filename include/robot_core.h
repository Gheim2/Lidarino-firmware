#pragma once
#include <Arduino.h>

#include "pins.h"
#include "serial_protocol.h"
#include "wheel_pid.h"
#include "tb6612.h"
#include "as5600_encoder.h"
#include <Adafruit_MPU6050.h>

class LidarinoRobot {
public:
    LidarinoRobot() 
        : encoderL(Wire), encoderR(Wire1),
          motorL(PIN_MOTOR_L_PWM, PIN_MOTOR_L_IN1, PIN_MOTOR_L_IN2),
          motorR(PIN_MOTOR_R_PWM, PIN_MOTOR_R_IN1, PIN_MOTOR_R_IN2),
          pidL(0.5f, 5.0f, 0.01f, 1.0f),
          pidR(0.5f, 5.0f, 0.01f, 1.0f) {}

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

        // IMU
        imu_ok_ = mpu.begin(MPU6050_I2C_ADDR, &Wire, 0);
        if (imu_ok_) {
            mpu.setAccelerometerRange(MPU6050_RANGE_2_G);
            mpu.setGyroRange(MPU6050_RANGE_250_DEG);
            mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
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
    }

private:
    AS5600Encoder encoderL;
    AS5600Encoder encoderR;
    TB6612Motor motorL;
    TB6612Motor motorR;
    WheelPID pidL;
    WheelPID pidR;
    Adafruit_MPU6050 mpu;

    bool imu_ok_ = false;

    float target_vel_l = 0.0f, target_vel_r = 0.0f;
    float meas_vel_l = 0.0f, meas_vel_r = 0.0f;
    int32_t pos_left = 0, pos_right = 0;

    uint32_t last_cmd_time = 0, last_pid_time = 0, last_tlm_time = 0;

    uint8_t serial_buf[COMMAND_PACKET_SIZE];
    uint8_t serial_idx = 0;

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

        sensor_event_t a, g, temp;
        mpu.getEvent(&a, &g, &temp);
        tp.accel_x = (int16_t)(a.acceleration.x * 1000);
        tp.accel_y = (int16_t)(a.acceleration.y * 1000);
        tp.accel_z = (int16_t)(a.acceleration.z * 1000);
        tp.gyro_x = (int16_t)(g.gyro.x * 1000);
        tp.gyro_y = (int16_t)(g.gyro.y * 1000);
        tp.gyro_z = (int16_t)(g.gyro.z * 1000);

        tp.checksum = compute_checksum((uint8_t*)&tp, TELEMETRY_PACKET_SIZE - 2);
        tp.terminator = 0x0D;

        Serial.write((uint8_t*)&tp, TELEMETRY_PACKET_SIZE);
    }
};