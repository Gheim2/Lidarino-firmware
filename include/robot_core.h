#pragma once
#include <Arduino.h>

#include "pins.h"
#include "serial_protocol.h"
#include "wheel_pid.h"
#include "tb6612.h"
#include "user_config.h"
#include "imu_calibration.h"
#include <AS5600.h>
#include "mpu6500.h" // Bolder Flight
#include <Adafruit_NeoPixel.h>
#include <esp_task_wdt.h>
#include <esp_attr.h>
#include <esp_system.h>

constexpr uint32_t UPDATE_WATCHDOG_TIMEOUT_S = 3;

// Ultima fase del loop raggiunta. Sta in RTC_NOINIT: sopravvive ai reset da watchdog/panic
// (non al power-on), così al boot successivo si sa dove si era bloccato il firmware.
enum LoopStage : uint32_t
{
    STAGE_NONE = 0,
    STAGE_SERIAL,
    STAGE_ENCODERS,
    STAGE_MOTORS,
    STAGE_IMU,
    STAGE_ENCODER_STATUS,
    STAGE_TELEMETRY_WRITE,
    STAGE_BATTERY,
    STAGE_LED,
    STAGE_COUNT
};
RTC_NOINIT_ATTR static uint32_t g_last_stage;

inline const char *stageName(uint32_t stage)
{
    static const char *const names[STAGE_COUNT] = {
        "none", "serial", "encoders", "motors", "imu", "encoder_status", "telemetry_write", "battery", "led"};
    return stage < STAGE_COUNT ? names[stage] : "invalid";
}

inline const char *resetReasonName(esp_reset_reason_t reason)
{
    switch (reason)
    {
    case ESP_RST_POWERON:   return "POWERON";
    case ESP_RST_SW:        return "SOFTWARE";
    case ESP_RST_PANIC:     return "PANIC";
    case ESP_RST_INT_WDT:   return "INT_WDT";
    case ESP_RST_TASK_WDT:  return "TASK_WDT";
    case ESP_RST_WDT:       return "OTHER_WDT";
    case ESP_RST_BROWNOUT:  return "BROWNOUT";
    case ESP_RST_DEEPSLEEP: return "DEEPSLEEP";
    case ESP_RST_EXT:       return "EXTERNAL";
    case ESP_RST_SDIO:      return "SDIO";
    default:                return "UNKNOWN";
    }
}

class LidarinoRobot
{
public:
    LidarinoRobot()
        : encoderR(&Wire), encoderL(&Wire1),
          motorL(PIN_MOTOR_L_PWM, PIN_MOTOR_L_IN2, PIN_MOTOR_L_IN1, 0),
          motorR(PIN_MOTOR_R_PWM, PIN_MOTOR_R_IN1, PIN_MOTOR_R_IN2, 1),
          pidL(0.11f, 0.05f, 0.0f, 0.8f),
          pidR(0.11f, 0.05f, 0.0f, 0.8f),
          statusLed(1, PIN_WATCHDOG_LED, NEO_GRB + NEO_KHZ800)
    {
    }

    void begin()
    {
        Serial.begin(115200);
        delay(1000);

        // Causa dell'ultimo reset e fase del loop in cui si trovava (solo se il reset non è un power-on,
        // altrimenti la memoria RTC contiene valori casuali)
        reset_reason_ = esp_reset_reason();
        crash_stage_ = (reset_reason_ == ESP_RST_POWERON || reset_reason_ == ESP_RST_BROWNOUT) ? STAGE_NONE : g_last_stage;
        g_last_stage = STAGE_NONE;
        printBootReport();

        Wire.begin(PIN_I2C0_SDA, PIN_I2C0_SCL, 400000);  // Per IMU ed Encoder R
        Wire.setTimeOut(50);
        Wire1.begin(PIN_I2C1_SDA, PIN_I2C1_SCL, 400000); // Per Encoder L
        Wire1.setTimeOut(50);


        // Motori ed emergenza
        if (!motorL.begin())
            Serial.println("Error initializing left motor PWM!");
        if (!motorR.begin())
            Serial.println("Error initializing right motor PWM!");
        pinMode(PIN_MOTOR_STBY, OUTPUT);
        digitalWrite(PIN_MOTOR_STBY, HIGH);
        pinMode(PIN_ESTOP, INPUT_PULLUP);

        // LED di stato
        statusLed.begin();
        statusLed.setBrightness(30);                            // Usa una luminosità bassa, i neopixel accecano!
        statusLed.setPixelColor(0, statusLed.Color(0, 0, 255)); // Blu durante il setup
        statusLed.show();

        // Encoder
        while (!encoderR.begin())
        {
            delay(1000);
            Serial.println("Error initializing right encoder!");
            checkEncoderDiagnostics();
        }
        while (!encoderL.begin())
        {
            delay(1000);
            Serial.println("Error initializing left encoder!");
            checkEncoderDiagnostics();
        }
        encoderL.setDirection(AS5600_COUNTERCLOCK_WISE);
        encoderR.setDirection(AS5600_CLOCK_WISE);

        // Resetta la posizione cumulativa iniziale a 0
        encoderL.resetCumulativePosition(0);
        encoderR.resetCumulativePosition(0);

        // L'IMU non blocca l'avvio: senza IMU motori e odometria da encoder funzionano lo stesso
        // (LED viola, bit IMU_OK a 0 in telemetria) e il loop ritenta la configurazione
        mpu.Config(&Wire, bfs::Mpu6500::I2C_ADDR_PRIM);
        for (uint8_t attempt = 0; attempt < IMU_SETUP_ATTEMPTS && !imu_ok_; attempt++)
        {
            imu_ok_ = configureIMU();
            if (!imu_ok_)
            {
                Serial.println("Error initializing IMU!");
                delay(200);
            }
        }
        if (imu_ok_)
            calibrateGyroBias();
        else
            Serial.println("IMU not responding: starting without it, retrying in the loop");

        esp_task_wdt_init(UPDATE_WATCHDOG_TIMEOUT_S, true);
        esp_task_wdt_add(NULL);

        uint32_t now = millis();
        last_cmd_time = now;
        last_pid_time = now;
        last_tlm_time = now;
        boot_time_ = now;
    }

    void update()
    {
        esp_task_wdt_reset();

        // 1. Seriale asincrona
        g_last_stage = STAGE_SERIAL;
        processSerial();

        // Letto DOPO processSerial(): se un comando appena ricevuto aggiorna
        // last_cmd_time a un millis() successivo, now - last_cmd_time andrebbe
        // in underflow e il timeout azzererebbe subito i target
        uint32_t now = millis();

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

            g_last_stage = STAGE_ENCODERS;
            int32_t new_pos_l = encoderL.getCumulativePosition();
            int32_t new_pos_r = encoderR.getCumulativePosition();

            const float alpha = 0.15f; // Fattore di smoothing per il filtro passa basso

            // Calcola velocità istantanea per il PID
            float raw_vel_l = ((float)(new_pos_l - pos_left) / AS5600_TICKS_PER_REV) * TWO_PI / dt_sec;
            float raw_vel_r = ((float)(new_pos_r - pos_right) / AS5600_TICKS_PER_REV) * TWO_PI / dt_sec;

            // Applica filtro passa basso
            filtered_vel_l = alpha * raw_vel_l + (1.0f - alpha) * filtered_vel_l;
            filtered_vel_r = alpha * raw_vel_r + (1.0f - alpha) * filtered_vel_r;

            meas_vel_l = filtered_vel_l;
            meas_vel_r = filtered_vel_r;

            #if DEBUG_MODE
            if (target_vel_l != 0.0f || target_vel_r != 0.0f) {
                // Formato per Serial Plotter: "Etichetta:Valore, Etichetta:Valore"
                Serial.print("Target_L:"); Serial.print(target_vel_l);
                Serial.print(", Meas_L:"); Serial.print(meas_vel_l);
                Serial.print(", Target_R:"); Serial.print(target_vel_r);
                Serial.print(", Meas_R:"); Serial.println(meas_vel_r);
            }
            #endif

            pos_left = new_pos_l;
            pos_right = new_pos_r;

            // Applica PID ai driver solo se c'è un target di velocità
            g_last_stage = STAGE_MOTORS;
            float outL = 0.0f;
            float outR = 0.0f;
            if (!manual_pwm) {
                if (target_vel_l != 0.0f) {
                    outL = pidL.compute(target_vel_l, meas_vel_l, dt_sec);
                } else {
                    pidL.reset(); // Reset del PID se il target è 0, per evitare accumulo di errore integrale    
                    outL = 0.0f; // Assicura che il comando al motore sia 0
                }
    
                if (target_vel_r != 0.0f) {
                    outR = pidR.compute(target_vel_r, meas_vel_r, dt_sec);
                } else {
                    pidR.reset(); // Reset del PID se il target è 0, per evitare accumulo di errore integrale    
                    outR = 0.0f; // Assicura che il comando al motore sia 0
                }
    
                motorL.setCommand(outL);
                motorR.setCommand(outR);
            }
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
            g_last_stage = STAGE_BATTERY;
            uint32_t raw_mv = analogReadMilliVolts(PIN_MEASURE_VIN);
            uint16_t calc_mv = raw_mv * (8.8 / 2.0);

            // Filtro passabasso esponenziale per stabilizzare la lettura della batteria
            if (current_battery_mv == 0)
                current_battery_mv = calc_mv;
            else
                current_battery_mv = (current_battery_mv * 0.8) + (calc_mv * 0.2);
        }
        g_last_stage = STAGE_LED;
        updateLedStatus(now);

        // Ripete il report di boot una volta dopo 3 s: la prima stampa può arrivare
        // prima che il nodo ROS abbia riaperto la porta dopo un reset
        if (!boot_report_repeated_ && now - boot_time_ >= 3000)
        {
            boot_report_repeated_ = true;
            printBootReport();
        }
        flushBootText();
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
    uint8_t imu_fail_count_ = 0;
    uint32_t last_imu_retry_ = 0;
    esp_reset_reason_t reset_reason_ = ESP_RST_UNKNOWN;
    uint32_t crash_stage_ = STAGE_NONE;
    uint32_t boot_time_ = 0;
    bool boot_report_repeated_ = false;
    char boot_text_[160] = {0};
    int boot_text_len_ = 0;
    int boot_text_pos_ = 0;
    static constexpr uint8_t IMU_MAX_FAILS = 5; // 5 letture fallite consecutive a 50 Hz = 100 ms
    static constexpr uint8_t IMU_SETUP_ATTEMPTS = 3;
    bool manual_pwm = false;

    float target_vel_l = 0.0f, target_vel_r = 0.0f;
    float meas_vel_l = 0.0f, meas_vel_r = 0.0f;
    float filtered_vel_l = 0.0f, filtered_vel_r = 0.0f;
    int32_t pos_left = 0, pos_right = 0;
    float acc_x = 0.0f, acc_y = 0.0f, acc_z = 0.0f;
    float gyro_x = 0.0f, gyro_y = 0.0f, gyro_z = 0.0f;
    float gyro_bias_[3] = {imu_calib::GYRO_BIAS_DEFAULT[0], imu_calib::GYRO_BIAS_DEFAULT[1], imu_calib::GYRO_BIAS_DEFAULT[2]};
    bool gyro_bias_from_boot_ = false;

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
            else if (current_battery_mv <= BATTERY_MV_THRESHOLD)
            {
                // Batteria scarica (<= BATTERY_MV_THRESHOLD): Magenta lampeggiante veloce
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
                // Timeout Seriale: Arancione lampeggiante alternato a Rosso
                DEBUG_PRINTLN("[WARN] Timeout seriale: nessun comando ricevuto da " + String(COMMAND_TIMEOUT_MS) + " ms!");
                statusLed.setPixelColor(0, led_state ? statusLed.Color(255, 100, 0) : statusLed.Color(255, 0, 0));
                
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
        #if DEBUG_MODE
        // --- MODALITÀ DEBUG: MENU INTERATTIVO ASCII ---
        while (Serial.available() > 0)
        {
            char c = Serial.read();
            if (c == '\n' || c == '\r') continue; // Ignora gli a capo

            switch (c)
            {
                case 'w': // Avanti entrambi (PID)
                    manual_pwm = false;
                    target_vel_l = 10.0f; target_vel_r = 10.0f;
                    DEBUG_PRINTLN("[DEBUG] Comando: AVANTI");
                    break;
                case 's': // Stop
                    manual_pwm = false;
                    target_vel_l = 0.0f; target_vel_r = 0.0f;
                    DEBUG_PRINTLN("[DEBUG] Comando: STOP");
                    break;
                case '1': // Solo motore sinistro (PID)
                    manual_pwm = false;
                    target_vel_l = 10.0f; target_vel_r = 0.0f;
                    DEBUG_PRINTLN("[DEBUG] Comando: TEST MOTORE SINISTRO");
                    break;
                case '2': // Solo motore destro (PID)
                    manual_pwm = false;
                    target_vel_l = 0.0f; target_vel_r = 10.0f;
                    DEBUG_PRINTLN("[DEBUG] Comando: TEST MOTORE DESTRO");
                    break;
                case 'q': // Test PWM grezzo Sinistro (Bypassa PID)
                    manual_pwm = true;
                    target_vel_l = 0.0f; target_vel_r = 0.0f;
                    motorL.setCommand(0.5f); // 50% duty cycle
                    motorR.setCommand(0.0f);
                    DEBUG_PRINTLN("[DEBUG] Comando: RAW PWM SINISTRO 50%");
                    break;
                case 'e': // Test PWM grezzo Destro (Bypassa PID)
                    manual_pwm = true;
                    target_vel_l = 0.0f; target_vel_r = 0.0f;
                    motorL.setCommand(0.0f);
                    motorR.setCommand(0.5f); // 50% duty cycle
                    DEBUG_PRINTLN("[DEBUG] Comando: RAW PWM DESTRO 50%");
                    break;
                case 'u': // Aumenta Kp di 0.01
                    pidL.setTunings(pidL.getKp() + 0.01f, pidL.getKi(), 0.0f);
                    pidR.setTunings(pidR.getKp() + 0.01f, pidR.getKi(), 0.0f);
                    DEBUG_PRINT("[DEBUG] Nuovo Kp: "); DEBUG_PRINTLN(pidL.getKp());
                    break;
                case 'j': // Riduci Kp di 0.01
                    if (pidL.getKp() > 0.01f) pidL.setTunings(pidL.getKp() - 0.01f, pidL.getKi(), 0.0f);
                    if (pidR.getKp() > 0.01f) pidR.setTunings(pidR.getKp() - 0.01f, pidR.getKi(), 0.0f);
                    DEBUG_PRINT("[DEBUG] Nuovo Kp: "); DEBUG_PRINTLN(pidL.getKp());
                    break;
                case 'i': // Aumenta Ki di 0.01
                    pidL.setTunings(pidL.getKp(), pidL.getKi() + 0.01f, 0.0f);
                    pidR.setTunings(pidR.getKp(), pidR.getKi() + 0.01f, 0.0f);
                    DEBUG_PRINT("[DEBUG] Nuovo Ki: "); DEBUG_PRINTLN(pidL.getKi());
                    break;
                case 'k': // Riduci Ki di 0.01
                    if (pidL.getKi() > 0.0f) pidL.setTunings(pidL.getKp(), pidL.getKi() - 0.01f, 0.0f);
                    if (pidR.getKi() > 0.0f) pidR.setTunings(pidR.getKp(), pidR.getKi() - 0.01f, 0.0f);
                    DEBUG_PRINT("[DEBUG] Nuovo Ki: "); DEBUG_PRINTLN(pidL.getKi());
                    break;
                default:
                    DEBUG_PRINTLN("Menu: [w] Avanti, [s] Stop, [1] Solo SX, [2] Solo DX, [q] Raw PWM SX, [e] Raw PWM DX");
                    break;
            }
            last_cmd_time = millis(); // Aggiorna il timeout di sicurezza
        }
        #else
        // --- MODALITÀ OPERATIVA: PROTOCOLLO BINARIO ---
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
        #endif
    }

    void sendTelemetry()
    {
        TelemetryPacket tp;
        tp.sync0 = PROTO_SYNC_0;
        tp.sync1 = PROTO_SYNC_1;
        tp.pos_left = pos_left;
        tp.pos_right = pos_right;

        // Lettura IMU prima dei flag, così il bit IMU_OK riflette questa lettura
        g_last_stage = STAGE_IMU;
        updateIMU();

        tp.status_flags = 0;
        // Restituisce true se l'encoder risponde e rileva un magnete valido
        g_last_stage = STAGE_ENCODER_STATUS;
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

        tp.accel_x = toMilli(acc_x);
        tp.accel_y = toMilli(acc_y);
        tp.accel_z = toMilli(acc_z);
        tp.gyro_x = toMilli(gyro_x);
        tp.gyro_y = toMilli(gyro_y);
        tp.gyro_z = toMilli(gyro_z);

        tp.battery_mv = current_battery_mv;
        tp.checksum = compute_checksum((uint8_t *)&tp, TELEMETRY_PACKET_SIZE - 2);
        tp.terminator = 0x0D;
        #if !DEBUG_MODE
            g_last_stage = STAGE_TELEMETRY_WRITE;
            // Serial.write() su USB CDC si blocca finché c'è spazio: se il Pi smette di leggere
            // (porta chiusa, nodo ROS fermo) il loop si fermava e il watchdog resettava l'ESP32.
            // Meglio scartare il pacchetto: il prossimo arriva 20 ms dopo.
            if (!bootTextPending() && Serial.availableForWrite() >= (int)TELEMETRY_PACKET_SIZE)
                Serial.write((uint8_t *)&tp, TELEMETRY_PACKET_SIZE);
        #endif
    }

    // Riga di testo nel flusso seriale: il parser ROS la scarta come byte non validi e la riporta nel log
    void printBootReport()
    {
        int len = snprintf(boot_text_, sizeof(boot_text_),
                           "\n[BOOT] reset_reason=%s last_stage=%s gyro_bias=%s(%.4f,%.4f,%.4f)\n",
                           resetReasonName(reset_reason_), stageName(crash_stage_),
                           gyro_bias_from_boot_ ? "boot" : "default",
                           gyro_bias_[0], gyro_bias_[1], gyro_bias_[2]);
        boot_text_len_ = (len > 0) ? min(len, (int)sizeof(boot_text_) - 1) : 0;
        boot_text_pos_ = 0;
        flushBootText();
    }

    // La riga è più lunga del buffer TX USB (64 byte): si invia a pezzi, quanto entra a ogni giro
    // del loop, senza mai bloccare. Finché non è finita la telemetria aspetta, così un pacchetto
    // binario non spezza la riga (il parser ROS la scarterebbe).
    void flushBootText()
    {
        if (boot_text_pos_ >= boot_text_len_) return;
        int space = Serial.availableForWrite();
        if (space <= 0) return;
        int chunk = min(space, boot_text_len_ - boot_text_pos_);
        Serial.write((const uint8_t *)boot_text_ + boot_text_pos_, chunk);
        boot_text_pos_ += chunk;
    }

    bool bootTextPending() const { return boot_text_pos_ < boot_text_len_; }

    // Configura l'IMU. Con ODR a 200 Hz la telemetria a 50 Hz trova sempre un
    // campione nuovo, e il DLPF a 41 Hz evita l'aliasing delle vibrazioni dei motori.
    bool configureIMU()
    {
        return mpu.Begin() &&
               // ±4G: la telemetria (int16 * 1000) arriva fino a ±32.7 m/s^2 (~3.3 g), oltre satura
               mpu.ConfigAccelRange(bfs::Mpu6500::ACCEL_RANGE_4G) &&
               mpu.ConfigDlpfBandwidth(bfs::Mpu6500::DLPF_BANDWIDTH_41HZ) &&
               mpu.ConfigSrd(4) && // Output data rate = 1000 / (1 + Srd) = 200 Hz
               mpu.EnableDrdyInt(); // Garantisce che il bit RAW_DATA_RDY usato da Read() venga settato (pin INT non collegato)
    }

    // Converte in unità * 1000 per la telemetria, saturando al range di int16
    // (il cast di un float fuori range a int16 è undefined behavior)
    static int16_t toMilli(float value)
    {
        float scaled = value * 1000.0f;
        if (scaled > 32767.0f) return INT16_MAX;
        if (scaled < -32768.0f) return INT16_MIN;
        return (int16_t)scaled;
    }

    void updateIMU()
    {
        // IMU guasta: un tentativo al secondo invece di 50, per non occupare il bus I2C
        // (condiviso con l'encoder destro) con transazioni che vanno in timeout
        if (!imu_ok_)
        {
            uint32_t now = millis();
            if (now - last_imu_retry_ < 1000) return;
            last_imu_retry_ = now;
        }

        // Read() restituisce false sia per errore I2C sia per assenza di dati nuovi;
        // con ODR a 200 Hz e lettura a 50 Hz il secondo caso non dovrebbe capitare,
        // quindi più fallimenti consecutivi indicano un'IMU non funzionante.
        if (!mpu.Read())
        {
            if (imu_fail_count_ < IMU_MAX_FAILS) imu_fail_count_++;
            if (imu_fail_count_ >= IMU_MAX_FAILS) imu_ok_ = false;
            // Un'IMU mai configurata (assente all'avvio) o resettata da un calo di tensione
            // non produce più dati: Read() fallirebbe per sempre senza riconfigurarla
            if (!imu_ok_) recoverIMU();
            return;
        }
        imu_fail_count_ = 0;
        imu_ok_ = true;

        // Assi robot REP 103 (x avanti, y sinistra, z su) e calibrazione, vedi imu_calibration.h
        float accel[3], gyro[3];
        readImuRobotFrame(accel, gyro);
        float accel_corr[3], gyro_corr[3];
        for (int i = 0; i < 3; i++)
        {
            accel_corr[i] = imu_calib::ACCEL_SCALE[i] * (accel[i] - imu_calib::ACCEL_OFFSET[i]);
            gyro_corr[i] = gyro[i] - gyro_bias_[i];
        }
        imu_calib::align(accel_corr, accel);
        imu_calib::align(gyro_corr, gyro);

        acc_x = accel[0];   // m/s^2
        acc_y = accel[1];
        acc_z = accel[2];
        gyro_x = gyro[0];   // rad/s
        gyro_y = gyro[1];
        gyro_z = gyro[2];
    }

    // Riconfigura l'IMU se è tornata a rispondere. configureIMU() blocca ~100 ms (la libreria fa
    // delay(10) dopo ogni scrittura di registro), quindi solo a robot fermo, mai durante il controllo.
    // imu_ok_ torna true alla prossima Read() riuscita.
    void recoverIMU()
    {
        if (target_vel_l != 0.0f || target_vel_r != 0.0f) return;
        // Probe rapido (una transazione, senza delay): se l'IMU non risponde non si prova a configurarla
        Wire.beginTransmission(bfs::Mpu6500::I2C_ADDR_PRIM);
        if (Wire.endTransmission() != 0) return;
        if (configureIMU()) imu_fail_count_ = 0;
    }

    // Ultimo campione della libreria convertito negli assi del robot (senza calibrazione)
    void readImuRobotFrame(float accel[3], float gyro[3])
    {
        const float accel_lib[3] = {mpu.accel_x_mps2(), mpu.accel_y_mps2(), mpu.accel_z_mps2()};
        const float gyro_lib[3] = {mpu.gyro_x_radps(), mpu.gyro_y_radps(), mpu.gyro_z_radps()};
        imu_calib::remap(accel_lib, accel);
        imu_calib::remap(gyro_lib, gyro);
    }

    // Bias del giroscopio misurato all'avvio (robot fermo, ~1 s). Se il robot si muove o l'IMU
    // non risponde si usano i valori di imu_calibration.h. Solo al boot, prima del watchdog.
    void calibrateGyroBias()
    {
        float sum[3] = {0, 0, 0}, sum_sq[3] = {0, 0, 0};
        uint16_t samples = 0;
        const uint32_t start = millis();
        while (samples < imu_calib::GYRO_CALIB_SAMPLES && millis() - start < imu_calib::GYRO_CALIB_TIMEOUT_MS)
        {
            if (!mpu.Read())
            {
                delay(1);
                continue;
            }
            float accel[3], gyro[3];
            readImuRobotFrame(accel, gyro);
            for (int i = 0; i < 3; i++)
            {
                sum[i] += gyro[i];
                sum_sq[i] += gyro[i] * gyro[i];
            }
            samples++;
        }

        bool still = samples == imu_calib::GYRO_CALIB_SAMPLES;
        float mean[3];
        for (int i = 0; i < 3 && still; i++)
        {
            mean[i] = sum[i] / samples;
            const float variance = sum_sq[i] / samples - mean[i] * mean[i];
            if (variance > imu_calib::GYRO_STILL_MAX_STD * imu_calib::GYRO_STILL_MAX_STD)
                still = false;
        }
        for (int i = 0; i < 3; i++)
            gyro_bias_[i] = still ? mean[i] : imu_calib::GYRO_BIAS_DEFAULT[i];
        gyro_bias_from_boot_ = still;
    }

    void checkEncoderDiagnostics()
    {
        // 1. Verifica la presenza e la distanza del magnete
        if (!encoderL.magnetDetected())
        {
            DEBUG_PRINTLN("[WARN] Encoder L: Nessun magnete rilevato!");
        }
        else
        {
            if (encoderL.magnetTooStrong())
            {
                DEBUG_PRINTLN("[WARN] Encoder L: Magnete TROPPO VICINO (troppo forte)");
            }
            if (encoderL.magnetTooWeak())
            {
                DEBUG_PRINTLN("[WARN] Encoder L: Magnete TROPPO LONTANO (troppo debole)");
            }
        }

        // 2. Controlla eventuali errori di comunicazione I2C
        int err = encoderL.lastError();
        if (err != 0)
        {
            DEBUG_PRINT("[ERROR] Encoder L errore I2C codice: ");
            DEBUG_PRINTLN(err);
        }
    }

    void checkIMUDiagnostics()
    {
        // 1. Verifica la presenza dell'IMU sul bus I2C
        Wire.beginTransmission(bfs::Mpu6500::I2C_ADDR_PRIM);
        byte error = Wire.endTransmission();
        if (error != 0)
        {
            DEBUG_PRINT("[ERROR] IMU non trovata sul bus! Errore I2C: ");
            DEBUG_PRINTLN(error);
        }
        else
        {
            DEBUG_PRINTLN("[INFO] IMU risponde all'indirizzo I2C.");
        }

        // 2. Lettura del registro WHO_AM_I
        Wire.beginTransmission(bfs::Mpu6500::I2C_ADDR_PRIM);
        Wire.write(0x75); // Indirizzo del registro WHO_AM_I
        Wire.endTransmission(false); // Repeated start
        Wire.requestFrom(bfs::Mpu6500::I2C_ADDR_PRIM, 1);
        if (Wire.available())
        {
            uint8_t whoami = Wire.read();
            DEBUG_PRINT("[INFO] Registro WHO_AM_I letto: 0x");
            Serial.println(whoami, HEX);
        }
        else
        {
            DEBUG_PRINTLN("[ERROR] Impossibile leggere il registro WHO_AM_I dell'IMU!");
        }
    }
};