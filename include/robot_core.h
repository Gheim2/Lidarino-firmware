#pragma once
#include <Arduino.h>

#include "pins.h"
#include "serial_protocol.h"
#include "wheel_pid.h"
#include "tb6612.h"
// #include <TB6612_ESP32.h> // TB6612FNG driver per ESP32
// #include "as5600_encoder.h"
#include "user_config.h"
#include <AS5600.h>
#include "mpu6500.h" // Bolder Flight
#include <Adafruit_NeoPixel.h>

class LidarinoRobot
{
public:
    LidarinoRobot()
        : encoderL(&Wire), encoderR(&Wire1),
          motorL(PIN_MOTOR_L_PWM, PIN_MOTOR_L_IN1, PIN_MOTOR_L_IN2, 0),
          motorR(PIN_MOTOR_R_PWM, PIN_MOTOR_R_IN1, PIN_MOTOR_R_IN2, 1),
          // motorL(PIN_MOTOR_L_IN1, PIN_MOTOR_L_IN2, PIN_MOTOR_L_PWM, 1, PIN_MOTOR_STBY, 5000, 10, 1),
          // motorR(PIN_MOTOR_R_IN1, PIN_MOTOR_R_IN2, PIN_MOTOR_R_PWM, 1, PIN_MOTOR_STBY, 5000, 10, 2),
          pidL(0.02f, 0.1f, 0.0f, 0.5f),
          pidR(0.02f, 0.1f, 0.0f, 0.5f),
          statusLed(1, PIN_WATCHDOG_LED, NEO_GRB + NEO_KHZ800)
    {
    }

    void begin()
    {
        Serial.begin(115200);
        delay(1000);
        Wire.begin(PIN_I2C0_SDA, PIN_I2C0_SCL, 400000);  // Per IMU ed Encoder R
        Wire.setTimeOut(50);
        Wire1.begin(PIN_I2C1_SDA, PIN_I2C1_SCL, 400000); // Per Encoder L
        Wire1.setTimeOut(50);


        // Motori ed emergenza
        motorL.begin();
        motorR.begin();
        pinMode(PIN_MOTOR_STBY, OUTPUT);
        digitalWrite(PIN_MOTOR_STBY, HIGH);
        pinMode(PIN_ESTOP, INPUT_PULLUP);

        // Encoder
        if (!encoderL.begin())
        {
            while (1)
            {
                delay(1000);
                Serial.println("Error initializing left encoder!");
                checkEncoderDiagnostics();
            }
        }
        if (!encoderR.begin())
        {
            while (1)
            {
                delay(1000);
                Serial.println("Error initializing right encoder!");
                checkEncoderDiagnostics();
            }
        }
        encoderL.setDirection(AS5600_CLOCK_WISE);
        encoderR.setDirection(AS5600_CLOCK_WISE);

        // Resetta la posizione cumulativa iniziale a 0
        encoderL.resetCumulativePosition(0);
        encoderR.resetCumulativePosition(0);

        // LED di stato
        statusLed.begin();
        statusLed.setBrightness(30);                            // Usa una luminosità bassa, i neopixel accecano!
        statusLed.setPixelColor(0, statusLed.Color(0, 0, 255)); // Blu durante il setup
        statusLed.show();

        // // IMU
        // Serial.println("[DEBUG] Inizio test IMU passo-passo...");

        // // 1. Test di presenza sul bus (WHO_AM_I)
        // Wire.beginTransmission(0x68); // Indirizzo I2C standard MPU6500
        // byte error = Wire.endTransmission();
        // if (error != 0) {
        //     Serial.print("[DEBUG ERRORE] IMU non trovata sul bus! Errore I2C: ");
        //     Serial.println(error);
        // } else {
        //     Serial.println("[DEBUG OK] IMU risponde all'indirizzo I2C.");
        // }

        // // 2. Lettura del registro WHO_AM_I (dovrebbe restituire 0x70 o simile per MPU6500)
        // Wire.beginTransmission(0x68);
        // Wire.write(0x75); // Indirizzo del registro WHO_AM_I
        // Wire.endTransmission(false); // Repeated start
        // Wire.requestFrom(0x68, 1);
        // if (Wire.available()) {
        //     uint8_t whoami = Wire.read();
        //     Serial.print("[DEBUG] Registro WHO_AM_I letto: 0x");
        //     Serial.println(whoami, HEX);
        // } else {
        //     Serial.println("[DEBUG ERRORE] Impossibile leggere il registro WHO_AM_I dell'IMU!");
        // }

        // 3. Ora proviamo l'avvio ufficiale della libreria
        mpu.Config(&Wire, bfs::Mpu6500::I2C_ADDR_PRIM);
        imu_ok_ = mpu.Begin();
        // mpu.Config(&Wire, bfs::Mpu6500::I2C_ADDR_PRIM);
        if (!imu_ok_)
        {
            while (1)
            {
                Serial.println("Error initializing IMU!");
                delay(1000);
            }
        }
        else
        {
            mpu.ConfigSrd(19); // Output data rate = 1000 / (1 + Srd) = 50 Hz
        }

        uint32_t now = millis();
        last_cmd_time = now;
        last_pid_time = now;
        last_tlm_time = now;
    }

    void update()
    {
        uint32_t now = millis();

        // 1. Seriale asincrona
        processSerial();

        // 2. Sicurezza
        if (now - last_cmd_time > COMMAND_TIMEOUT_MS || digitalRead(PIN_ESTOP) == LOW)
        {
            target_vel_l = 0.0f;
            target_vel_r = 0.0f;
        }

        // 3. Loop di controllo a 100 Hz
        if (now - last_pid_time >= 10)
        {

            float dt_sec = (now - last_pid_time) / 1000.0f;
            last_pid_time = now;

            // Aggiorna posizioni interne
            // encoderL.update();
            // encoderR.update();

            // Legge posizioni cumulative
            // int32_t new_pos_l = encoderL.getPositionTicks();
            // int32_t new_pos_r = encoderR.getPositionTicks();
            // Legge le posizioni cumulative native della libreria
            int32_t new_pos_l = encoderL.getCumulativePosition();
            int32_t new_pos_r = encoderR.getCumulativePosition();

            // Calcola velocità istantanea per il PID
            meas_vel_l = ((float)(new_pos_l - pos_left) / AS5600_TICKS_PER_REV) * TWO_PI / dt_sec;
            meas_vel_r = ((float)(new_pos_r - pos_right) / AS5600_TICKS_PER_REV) * TWO_PI / dt_sec;

            pos_left = new_pos_l;
            pos_right = new_pos_r;

            // Applica PID ai driver
            float outL = pidL.compute(target_vel_l, meas_vel_l, dt_sec);
            float outR = pidR.compute(target_vel_r, meas_vel_r, dt_sec);
            motorL.setCommand(outL);
            motorR.setCommand(outR);
            // Serial.print("TargetL: ");
            // Serial.print(target_vel_l);
            // Serial.print(" | MeasL: ");
            // Serial.print(meas_vel_l);
            // Serial.print(" | OutL: ");
            // Serial.println(outL);
            // Serial.print("TargetR: ");
            // Serial.print(target_vel_r);
            // Serial.print(" | MeasR: ");
            // Serial.print(meas_vel_r);
            // Serial.print(" | OutR: ");
            // Serial.println(outR);
            // Serial.println("----");
            // motorL.drive(0.5f * 1023.0f, 1000);
            // motorR.drive(0.5f * 1023.0f, 2000);
            // motorL.setCommand(0.0f);
            // motorR.setCommand(0.0f);
        }

        // 4. Telemetria a 50 Hz
        if (now - last_tlm_time >= (1000 / TELEMETRY_RATE_HZ))
        {
            last_tlm_time = now;
            sendTelemetry();
        }

        // 5. Lettura batteria a 1 Hz
        if (now - last_battery_time >= 1000)
        {
            last_battery_time = now;
            uint32_t raw_mv = analogReadMilliVolts(PIN_MEASURE_VIN);
            uint16_t calc_mv = raw_mv * (8.8 / 2.0);

            // Filtro passabasso esponenziale per stabilizzare la lettura della batteria
            if (current_battery_mv == 0)
                current_battery_mv = calc_mv;
            else
                current_battery_mv = (current_battery_mv * 0.8) + (calc_mv * 0.2);
        }
        updateLedStatus(now);
    }

private:
    Adafruit_NeoPixel statusLed;
    uint32_t last_led_blink = 0;
    bool led_state = false;
    uint32_t blink_interval = 1000; // ms
    // AS5600Encoder encoderL;
    // AS5600Encoder encoderR;
    AS5600 encoderL;
    AS5600 encoderR;
    TB6612Motor motorL;
    TB6612Motor motorR;
    // Motor motorL;
    // Motor motorR;
    WheelPID pidL;
    WheelPID pidR;
    bfs::Mpu6500 mpu;
    uint16_t current_battery_mv = 0;
    uint32_t last_battery_time = 0;

    bool imu_ok_ = false;

    float target_vel_l = 0.0f, target_vel_r = 0.0f;
    float meas_vel_l = 0.0f, meas_vel_r = 0.0f;
    int32_t pos_left = 0, pos_right = 0;
    float acc_x, acc_y, acc_z;
    float gyro_x, gyro_y, gyro_z;

    uint32_t last_cmd_time = 0, last_pid_time = 0, last_tlm_time = 0;

    uint8_t serial_buf[COMMAND_PACKET_SIZE];
    uint8_t serial_idx = 0;

    void updateLedStatus(uint32_t now)
    {
        if (now - last_led_blink >= blink_interval)
        {
            last_led_blink = now;
            led_state = !led_state;
            // Restituisce true se l'encoder risponde e rileva un magnete valido
            bool encoder_l_ok = encoderL.isConnected() && (encoderL.magnetDetected() == 1);
            bool encoder_r_ok = encoderR.isConnected() && (encoderR.magnetDetected() == 1);

            if (digitalRead(PIN_ESTOP) == LOW)
            {
                // Emergenza premuta: Rosso
                statusLed.setPixelColor(0, led_state ? statusLed.Color(255, 0, 0) : statusLed.Color(50, 0, 0));
            }
            else if (current_battery_mv <= 9800)
            {
                // Batteria scarica (<= 9.8V): Magenta lampeggiante veloce
                statusLed.setPixelColor(0, led_state ? statusLed.Color(255, 0, 255) : statusLed.Color(0, 0, 0));
                blink_interval = 250;
            }
            else if (!encoder_l_ok || !encoder_r_ok)
            {
                // Encoder non ok: Giallo lampeggiante
                #if DEBUG_MODE
                    checkEncoderDiagnostics();
                #endif
                statusLed.setPixelColor(0, led_state ? statusLed.Color(255, 255, 0) : statusLed.Color(0, 0, 0));
            }
            else if (!imu_ok_)
            {
                // IMU non ok: Viola lampeggiante
                #if DEBUG_MODE
                    checkIMUDiagnostics();
                #endif
                statusLed.setPixelColor(0, led_state ? statusLed.Color(128, 0, 128) : statusLed.Color(0, 0, 0));
            }
            else if (now - last_cmd_time > COMMAND_TIMEOUT_MS)
            {
                // Timeout Seriale: Arancione lampeggiante
                statusLed.setPixelColor(0, led_state ? statusLed.Color(255, 100, 0) : statusLed.Color(0, 0, 0));
            }
            else
            {
                // Tutto ok: Verde
                statusLed.setPixelColor(0, led_state ? statusLed.Color(0, 255, 0) : statusLed.Color(0, 50, 0));
                blink_interval = 1000; // Reset blink interval to default
            }
            statusLed.show();
        }
    }

    void processSerial()
    {
        while (Serial.available() > 0)
        {
            serial_buf[serial_idx++] = Serial.read();

            if (serial_idx == COMMAND_PACKET_SIZE)
            {
                if (serial_buf[0] == PROTO_SYNC_0 && serial_buf[1] == PROTO_SYNC_1 && serial_buf[COMMAND_PACKET_SIZE - 1] == 0x0D)
                {
                    if (compute_checksum(serial_buf, COMMAND_PACKET_SIZE - 2) == serial_buf[COMMAND_PACKET_SIZE - 2])
                    {
                        CommandPacket *cmd = (CommandPacket *)serial_buf;
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

    void sendTelemetry()
    {
        TelemetryPacket tp;
        tp.sync0 = PROTO_SYNC_0;
        tp.sync1 = PROTO_SYNC_1;
        tp.pos_left = pos_left;
        tp.pos_right = pos_right;

        tp.status_flags = 0;
        // Restituisce true se l'encoder risponde e rileva un magnete valido
        bool encoder_l_ok = encoderL.isConnected() && (encoderL.magnetDetected() == 1);
        bool encoder_r_ok = encoderR.isConnected() && (encoderR.magnetDetected() == 1);
        if (digitalRead(PIN_ESTOP) == LOW)
            tp.status_flags |= STATUS_BIT_ESTOP;
        if (encoder_l_ok)
            tp.status_flags |= STATUS_BIT_ENCODER_L_OK;
        if (encoder_r_ok)
            tp.status_flags |= STATUS_BIT_ENCODER_R_OK;
        if (imu_ok_)
            tp.status_flags |= STATUS_BIT_IMU_OK;

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

        tp.battery_mv = current_battery_mv;
        tp.checksum = compute_checksum((uint8_t *)&tp, TELEMETRY_PACKET_SIZE - 2);
        tp.terminator = 0x0D;
        #if !DEBUG_MODE
            Serial.write((uint8_t *)&tp, TELEMETRY_PACKET_SIZE);
        #endif
    }

    void updateIMU()
    {
        if (mpu.Read())
        {
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

    void checkEncoderDiagnostics()
    {
        // 1. Verifica la presenza e la distanza del magnete
        if (!encoderL.magnetDetected())
        {
            Serial.println("[WARN] Encoder L: Nessun magnete rilevato!");
        }
        else
        {
            if (encoderL.magnetTooStrong())
            {
                Serial.println("[WARN] Encoder L: Magnete TROPPO VICINO (troppo forte)");
            }
            if (encoderL.magnetTooWeak())
            {
                Serial.println("[WARN] Encoder L: Magnete TROPPO LONTANO (troppo debole)");
            }
        }

        // 2. Controlla eventuali errori di comunicazione I2C
        int err = encoderL.lastError();
        if (err != 0)
        {
            Serial.print("[ERROR] Encoder L errore I2C codice: ");
            Serial.println(err);
        }
    }

    void checkIMUDiagnostics()
    {
        // 1. Verifica la presenza dell'IMU sul bus I2C
        Wire.beginTransmission(bfs::Mpu6500::I2C_ADDR_PRIM);
        byte error = Wire.endTransmission();
        if (error != 0)
        {
            Serial.print("[ERROR] IMU non trovata sul bus! Errore I2C: ");
            Serial.println(error);
        }
        else
        {
            Serial.println("[INFO] IMU risponde all'indirizzo I2C.");
        }

        // 2. Lettura del registro WHO_AM_I
        Wire.beginTransmission(bfs::Mpu6500::I2C_ADDR_PRIM);
        Wire.write(0x75); // Indirizzo del registro WHO_AM_I
        Wire.endTransmission(false); // Repeated start
        Wire.requestFrom(bfs::Mpu6500::I2C_ADDR_PRIM, 1);
        if (Wire.available())
        {
            uint8_t whoami = Wire.read();
            Serial.print("[INFO] Registro WHO_AM_I letto: 0x");
            Serial.println(whoami, HEX);
        }
        else
        {
            Serial.println("[ERROR] Impossibile leggere il registro WHO_AM_I dell'IMU!");
        }
    }
};