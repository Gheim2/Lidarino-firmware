#pragma once
#include <stdint.h>

// ─────────────────────────────────────────────────────────────
// PROTOCOLLO SERIALE BINARIO — Orange Pi <-> ESP32-S3
//
// Questo header va condiviso (copiato) anche nel pacchetto
// lidarino_hardware lato ROS 2, così entrambi i lati parlano
// esattamente lo stesso formato.
//
// Tutti i pacchetti sono little-endian (nativo su ESP32 e ARM64).
// ─────────────────────────────────────────────────────────────

// Byte di sincronizzazione: ogni pacchetto inizia con questi due byte.
// Servono per ri-sincronizzarsi dopo un eventuale errore di framing
// (es. all'avvio, o se un byte viene perso/corrotto in transito).
#define PROTO_SYNC_0   0xAA
#define PROTO_SYNC_1   0x55

// ───────────── Comando: Orange Pi → ESP32 ─────────────
// Velocità target delle due ruote, in rad/s, scalate per
// trasmissione come intero (rad/s * 1000), per evitare floating
// point sulla seriale (più robusto, meno problemi di endianness sui float).
//
// Dimensione totale pacchetto: 10 byte
#pragma pack(push, 1)
struct CommandPacket {
    uint8_t  sync0;       // 0xAA
    uint8_t  sync1;       // 0x55
    int16_t  vel_left;    // rad/s * 1000
    int16_t  vel_right;   // rad/s * 1000
    uint16_t reserved;    // riservato per usi futuri (es. flag), sempre 0
    uint8_t  checksum;    // XOR di tutti i byte precedenti
    uint8_t  terminator;  // 0x0D, per validazione extra
};
#pragma pack(pop)
#define COMMAND_PACKET_SIZE sizeof(CommandPacket)  // 10 byte

// ───────────── Telemetria: ESP32 → Orange Pi ─────────────
// Posizione assoluta cumulativa dei due encoder, in "tick" (12 bit
// AS5600 = 4096 tick/giro, ma qui è già un contatore esteso oltre
// i 4096 per gestire giri multipli — calcolato lato firmware).
//
// Dimensione totale pacchetto: 25 byte
#pragma pack(push, 1)
struct TelemetryPacket {
    uint8_t  sync0;        // 0xAA
    uint8_t  sync1;        // 0x55
    int32_t  pos_left;     // tick assoluti cumulativi, ruota sinistra
    int32_t  pos_right;    // tick assoluti cumulativi, ruota destra

    int16_t  accel_x;
    int16_t  accel_y;
    int16_t  accel_z;
    int16_t  gyro_x;
    int16_t  gyro_y;
    int16_t  gyro_z;

    uint8_t  status_flags; // bit0: estop attivo, bit1: encoder_L ok, bit2: encoder_R ok, bit3: IMU ok
    uint8_t  checksum;     // XOR di tutti i byte precedenti
    uint8_t  terminator;   // 0x0D
};
#pragma pack(pop)
#define TELEMETRY_PACKET_SIZE sizeof(TelemetryPacket)  // 25 byte

// Bit del campo status_flags
#define STATUS_BIT_ESTOP        (1 << 0)
#define STATUS_BIT_ENCODER_L_OK (1 << 1)
#define STATUS_BIT_ENCODER_R_OK (1 << 2)
#define STATUS_BIT_IMU_OK       (1 << 3)

// Risoluzione AS5600: 12 bit -> 4096 tick per giro meccanico del sensore
#define AS5600_TICKS_PER_REV  4096

// Timeout di sicurezza: se l'ESP32 non riceve un CommandPacket valido
// entro questo intervallo, ferma i motori autonomamente.
#define COMMAND_TIMEOUT_MS    300

// Frequenza di invio telemetria (Hz)
#define TELEMETRY_RATE_HZ     50

// Calcola lo XOR checksum di un buffer di byte (esclude il checksum stesso
// e il terminator, che sono gli ultimi 2 byte della struct)
inline uint8_t compute_checksum(const uint8_t* data, size_t len_excluding_checksum_and_terminator) {
    uint8_t cs = 0;
    for (size_t i = 0; i < len_excluding_checksum_and_terminator; i++) {
        cs ^= data[i];
    }
    return cs;
}