#pragma once

// PID semplice per il controllo di velocità di una singola ruota.
// Lavora in rad/s: riceve target e misura (entrambe in rad/s),
// restituisce un comando PWM normalizzato in [-1.0, 1.0].
class WheelPID {
public:
    WheelPID(float kp, float ki, float kd, float output_limit = 1.0f)
        : kp_(kp), ki_(ki), kd_(kd), output_limit_(output_limit),
          integral_(0.0f), prev_error_(0.0f), first_run_(true) {}

    // dt in secondi
    float compute(float target_rad_s, float measured_rad_s, float dt) {
        float error = target_rad_s - measured_rad_s;

        // Termine integrale con anti-windup: limitiamo l'integrale
        // stesso, non solo l'output, altrimenti il PID "memorizza"
        // errore in eccesso e poi overshoot quando il target cambia.
        integral_ += error * dt;
        float integral_limit = output_limit_ / (ki_ > 0.0001f ? ki_ : 1.0f);
        if (integral_ > integral_limit) integral_ = integral_limit;
        if (integral_ < -integral_limit) integral_ = -integral_limit;

        // Termine derivativo: al primo giro non abbiamo un prev_error
        // valido, quindi lo saltiamo per evitare un picco spurio.
        float derivative = 0.0f;
        if (!first_run_ && dt > 0.0001f) {
            derivative = (error - prev_error_) / dt;
        }
        first_run_ = false;

        float output = kp_ * error + ki_ * integral_ + kd_ * derivative;

        if (output > output_limit_) output = output_limit_;
        if (output < -output_limit_) output = -output_limit_;

        prev_error_ = error;
        return output;
    }

    void reset() {
        integral_ = 0.0f;
        prev_error_ = 0.0f;
        first_run_ = true;
    }

    void setTunings(float kp, float ki, float kd) {
        kp_ = kp;
        ki_ = ki;
        kd_ = kd;
    }

    float getKp() { return kp_; }
    float getKi() { return ki_; }
    float getKd() { return kd_; }

private:
    float kp_, ki_, kd_;
    float output_limit_;
    float integral_;
    float prev_error_;
    bool first_run_;
};