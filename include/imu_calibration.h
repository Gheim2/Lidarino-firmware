#pragma once
#include <stdint.h>

// ─────────────────────────────────────────────────────────────
// Calibrazione IMU (MPU6500, libreria Bolder Flight).
// Da rifare quando si passa al LSM6DS3: cambiano assi, offset e scale.
//
// Misurata il 2026-10-28 con tools/imu_calibrate.py, robot fermo in 4 posizioni:
// in piano, sul fianco destro, sul fianco sinistro, sul retro.
//
// Correzione applicata (assi robot REP 103: x avanti, y sinistra, z su):
//   accel = ALIGN * (ACCEL_SCALE * (remap(accel_lib) - ACCEL_OFFSET))
//   gyro  = ALIGN * (remap(gyro_lib) - bias)      bias misurato a ogni avvio
// ─────────────────────────────────────────────────────────────
namespace imu_calib
{
    // Assi della libreria -> assi robot: robot[i] = AXIS_SIGN[i] * libreria[AXIS_INDEX[i]]
    // (la libreria usa assi stile NED: x avanti, y destra, z giù)
    constexpr uint8_t AXIS_INDEX[3] = {0, 1, 2};
    constexpr float AXIS_SIGN[3] = {+1.0f, -1.0f, -1.0f};

    // Offset e scala dell'accelerometro, in assi robot (m/s^2 e adimensionale)
    constexpr float ACCEL_OFFSET[3] = {-0.1671f, -0.1334f, 0.0044f};
    constexpr float ACCEL_SCALE[3] = {0.99077f, 0.99930f, 0.95948f};

    // Allineamento: rotazione che porta la gravità misurata con il robot in piano su +z
    // (IMU montata inclinata di roll -1.64 deg, pitch +4.20 deg rispetto al pavimento)
    constexpr float ALIGN[3][3] = {
        {+0.997314f, -0.002093f, +0.073217f},
        {+0.000000f, +0.999592f, +0.028568f},
        {-0.073247f, -0.028491f, +0.996907f},
    };

    // Bias del giroscopio usato se la calibrazione all'avvio fallisce (robot in movimento), rad/s in assi robot
    constexpr float GYRO_BIAS_DEFAULT[3] = {0.1699f, 0.1318f, 0.0212f};

    // Calibrazione del giroscopio all'avvio: 1 s di campioni a 200 Hz, robot fermo.
    // Deviazione standard misurata da fermo: ~0.001 rad/s.
    constexpr uint16_t GYRO_CALIB_SAMPLES = 200;
    constexpr uint32_t GYRO_CALIB_TIMEOUT_MS = 2000;
    constexpr float GYRO_STILL_MAX_STD = 0.01f; // oltre questa soglia il robot si stava muovendo

    inline void remap(const float lib[3], float robot[3])
    {
        for (int i = 0; i < 3; i++)
            robot[i] = AXIS_SIGN[i] * lib[AXIS_INDEX[i]];
    }

    inline void align(const float in[3], float out[3])
    {
        for (int i = 0; i < 3; i++)
            out[i] = ALIGN[i][0] * in[0] + ALIGN[i][1] * in[1] + ALIGN[i][2] * in[2];
    }
}
