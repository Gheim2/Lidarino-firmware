#pragma once

// ─────────────────────────────────────────────────────────────
// PINOUT — vedi diagramma di riferimento nella documentazione.
// GPIO0, 19, 20, 45, 46 evitati: strapping pins / USB nativo riservati.
// ─────────────────────────────────────────────────────────────

// Motore destro (TB6612FNG canale A)
#define PIN_MOTOR_R_PWM   6
#define PIN_MOTOR_R_IN1   4
#define PIN_MOTOR_R_IN2   5

// Motore sinistro (TB6612FNG canale B)
#define PIN_MOTOR_L_PWM   15
#define PIN_MOTOR_L_IN1   17
#define PIN_MOTOR_L_IN2   16

// Standby condiviso TB6612FNG (LOW = motori disabilitati, sicurezza)
#define PIN_MOTOR_STBY    7

// I2C bus 0 — encoder AS5600 destro
#define PIN_I2C0_SDA      9
#define PIN_I2C0_SCL      8

// I2C bus 1 — encoder AS5600 sinistro
#define PIN_I2C1_SDA      11
#define PIN_I2C1_SCL      10

// UART verso Orange Pi (UART0 hardware)
#define PIN_UART_TX       43
#define PIN_UART_RX       44

// Sicurezza
#define PIN_WATCHDOG_LED  48     // lampeggia mentre il loop gira correttamente
#define PIN_ESTOP         13    // letto come stato (NON è la sicurezza primaria,
                                // quella è il taglio elettrico fisico sul pulsante)
#define PIN_MEASURE_VIN   12     // lettura tensione batteria (tramite partitore resistivo)

// Indirizzo I2C fisso degli AS5600 (entrambi uguali, per questo servono
// due bus separati invece di uno condiviso)
#define AS5600_I2C_ADDR   0x36
#define MPU6050_I2C_ADDR  0x68