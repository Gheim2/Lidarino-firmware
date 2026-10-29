// Test sulla scheda del PWM dei motori TB6612: pio test -e bpi_leaf_s3 -f test_motor_pwm
// (carica il test sull'ESP32 al posto del firmware del robot).
//
// IN1/IN2 restano sempre LOW: il driver è in freno, i motori non possono muoversi.
// Verifica che frequenza e risoluzione di tb6612.h siano ottenibili dal clock LEDC
// (con 10 kHz / 12 bit ledcSetup() falliva e i motori non ricevevano PWM).
// Dopo i test il PWM resta al 50% su GPIO 6 e 15, per controllarlo con un analizzatore logico.
#include <Arduino.h>
#include <unity.h>
#include "pins.h"
#include "tb6612.h"

static const uint8_t PWM_PINS[2] = {PIN_MOTOR_R_PWM, PIN_MOTOR_L_PWM};
static const uint8_t IN_PINS[4] = {PIN_MOTOR_R_IN1, PIN_MOTOR_R_IN2, PIN_MOTOR_L_IN1, PIN_MOTOR_L_IN2};
static const uint32_t HALF_DUTY = (TB6612Motor::PWM_MAX_DUTY + 1) / 2;

static void brakeMotors()
{
    for (uint8_t pin : IN_PINS)
    {
        pinMode(pin, OUTPUT);
        digitalWrite(pin, LOW);
    }
}

void test_motor_begin_reports_success()
{
    // Stessi pin e canali del firmware; begin() lascia IN1/IN2 LOW e duty 0
    TB6612Motor motor_r(PIN_MOTOR_R_PWM, PIN_MOTOR_R_IN1, PIN_MOTOR_R_IN2, 0);
    TB6612Motor motor_l(PIN_MOTOR_L_PWM, PIN_MOTOR_L_IN2, PIN_MOTOR_L_IN1, 1);
    TEST_ASSERT_TRUE(motor_r.begin());
    TEST_ASSERT_TRUE(motor_l.begin());
}

void test_ledc_setup_reaches_configured_frequency()
{
    for (uint8_t ch = 0; ch < 2; ch++)
    {
        uint32_t freq = ledcSetup(ch, TB6612Motor::PWM_FREQ_HZ, TB6612Motor::PWM_RES_BITS);
        // 0 = configurazione impossibile; altrimenti entro l'1% della frequenza richiesta
        TEST_ASSERT_UINT32_WITHIN(TB6612Motor::PWM_FREQ_HZ / 100, TB6612Motor::PWM_FREQ_HZ, freq);
    }
}

void test_pwm_duty_is_applied()
{
    for (uint8_t ch = 0; ch < 2; ch++)
    {
        ledcAttachPin(PWM_PINS[ch], ch);
        ledcWrite(ch, HALF_DUTY);
        TEST_ASSERT_EQUAL_UINT32(HALF_DUTY, ledcRead(ch));
    }
}

void setup()
{
    brakeMotors();
    pinMode(PIN_MOTOR_STBY, OUTPUT);
    digitalWrite(PIN_MOTOR_STBY, HIGH);
    delay(2000);  // tempo al test runner per aprire la seriale

    UNITY_BEGIN();
    RUN_TEST(test_motor_begin_reports_success);
    RUN_TEST(test_ledc_setup_reaches_configured_frequency);
    RUN_TEST(test_pwm_duty_is_applied);  // per ultimo: lascia il PWM al 50% con i motori in freno
    UNITY_END();
    brakeMotors();
}

void loop()
{
}
