#include <Arduino.h>

#include "temperature.h"
#include "config.h"

static const float roll_avg_factor = 0.9;
static const float temp_pid_p = 0.5;
static const float temp_pid_i = 8.0;

static float temp_pid_i_term = 0;

static float temp_roll_avg = 0.0;
static float temp_setpoint = 0.0;

int temperature_init(void){
    pinMode(TEMP_SENSOR_PIN, INPUT);
    pinMode(HEATER_PIN, OUTPUT);
    analogWrite(HEATER_PIN, 0);

    return 0;
}

int temperature_poll(uint32_t dt_us){
    // calc temperature
    float V0c = 0.4; // 0C output is 400mV
    float TC = 0.0195; // 19.5mV per degree C
    float raw_temp = (analogRead(TEMP_SENSOR_PIN) / 4095.0 * 3.3 - V0c) / TC;
    

    temp_roll_avg = temp_roll_avg * roll_avg_factor + raw_temp * (1 - roll_avg_factor);

    // PID
    float diff_temp = temp_setpoint - temp_roll_avg; // target temperature is 60C

    temp_pid_i_term += diff_temp * dt_us / 1000000.0 * temp_pid_i;
    temp_pid_i_term = constrain(temp_pid_i_term, -0.1, 0.9); // anti-windup
    float heater_power = diff_temp * temp_pid_p + temp_pid_i_term; // P control only for heating, no cooling

    analogWrite(HEATER_PIN, constrain(heater_power * PWM_MAX_VALUE, 0, PWM_MAX_VALUE-1));

    return 0;
}

void temperature_set(float temp){
    temp_setpoint = temp;
}
float temperature_get(void){
    return temp_roll_avg;
}

float temperature_target_get(void){
    return temp_setpoint;
}