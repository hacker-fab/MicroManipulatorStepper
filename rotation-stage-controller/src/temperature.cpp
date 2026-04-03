#include <Arduino.h>

#include "temperature.h"
#include "config.h"

static const float roll_avg_factor = 0.95;
static const float temp_pid_p = 1.0;

static float temp_roll_avg = 0.0;
static float temp_setpoint = 0.0;

int temperature_init(void){
    pinMode(TEMP_SENSOR_PIN, INPUT);
    pinMode(HEATER_PIN, OUTPUT);
    analogWrite(HEATER_PIN, 0);

    return 0;
}

int temperature_poll(void){
    // calc temperature
    float V0c = 0.4; // 0C output is 400mV
    float TC = 0.0195; // 19.5mV per degree C
    float faw_temp = (analogRead(TEMP_SENSOR_PIN) / 4095.0 * 3.3 - V0c) / TC;
    

    temp_roll_avg = temp_roll_avg * roll_avg_factor + faw_temp * (1 - roll_avg_factor);

    // PID
    float diff_temp = temp_setpoint - temp_roll_avg; // target temperature is 60C
    float heater_power = diff_temp * temp_pid_p; // P control only for heating, no cooling

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