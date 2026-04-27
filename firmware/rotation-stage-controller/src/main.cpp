#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>

#include "config.h"

#include "rotation.h"
#include "temperature.h"
#include "i2c_parser.h"
#include "vacuum.h"




void setup()
{
    Serial.begin(115200);
    analogReadResolution(12);
    analogWriteFrequency(PWM_FREQ);
    analogWriteResolution(12);

    i2c_parser_init(
        (i2c_parser_callbacks){
            .home_rotation = rotation_home,
            .set_rotation_target = rotation_set,
            .get_rotation_target = rotation_target_get,
            .get_rotation = rotation_get,

            .set_temperature_setpoint = temperature_set,
            .get_temperature_setpoint = temperature_target_get,
            .get_temperature = temperature_get,

            .set_vacuum = vacuum_set,
            .get_vacuum = vacuum_get,
    });

    rotation_init();
    temperature_init();
    vacuum_init();

    rotation_home();

    // buildAngleLookupTable();
    // printAngleLookupTable();
}


int i = 0;
uint32_t last_time = 0;
char input_buffer[32] = {0};
uint16_t input_index = 0;

uint32_t parse_uint(char *str) {
    uint32_t result = 0;
    while (*str >= '0' && *str <= '9') {
        result = result * 10 + (*str - '0');
        str++;
    }
    return result;
}

static char invalid_command_hint[] = "Invalid command!\n"
" * Format:\n"
" * ROT <angle>\n"
" * where angle is in 1/100 degrees, so 90 deg is 9000\n"
" * \n"
" * VAC <PWM percent>\n"
" * where 0 is off and 100 is fully on\n"
" * \n"
" * TEM <temp>\n"
" * where temp is in 1/100 degrees C, so 60C is 6000\n"
" * \n"
" * Put 'R' in the value to get current sensor value\n"
" * Example: TEM R\n"
" * \n"
" * HOME\n"
" * Home the rotation sensor\n"
" * \n"
" * CALI\n"
" * Calibrate the rotation sensor. Build the angle lookup table and print it out\n";

/**
 * Debug command
 */
void parse_uart_input(){
    
    if (Serial.available() > 0)
    {
        char c = Serial.read();
        if (c == '\n')
        {
            input_buffer[input_index] = '\0';

            // Minimal length
            if (input_index < 4) {
                // invalid line
                Serial.println(invalid_command_hint);
                input_index = 0;
                return;
            }

            char cmd[5] = {0};
            memcpy(cmd, input_buffer, 4);

            if (strcmp(cmd, "ROT ") == 0) {
                
                // input in degree
                // manual parse to avoid large size
                if (input_buffer[4] == 'R') {
                    Serial.println(rotation_get() * 180 / PI);
                    input_index = 0;
                    return;
                }
                int input_raw = parse_uint(input_buffer + 4);
                Serial.println(input_raw);
                float input_angle = input_raw / 100.0;
                float rad_angle = input_angle / 180.0 * PI;
                while (rad_angle < 0)
                {
                    rad_angle += 2 * PI;
                }
                rotation_set(rad_angle);

            } else if (strcmp(cmd, "TEM ") == 0) {
                if (input_buffer[4] == 'R') {
                    Serial.println(temperature_get());
                    input_index = 0;
                    return;
                }

                int input_raw = parse_uint(input_buffer + 4);
                temperature_set(input_raw / 100.0);
                
            } else if (strcmp(cmd, "VAC ") == 0) {
                if (input_buffer[4] == 'R') {
                    Serial.println(vacuum_get());
                    input_index = 0;
                    return;
                }
                vacuum_set(parse_uint(input_buffer + 4));
            } else if (strcmp(cmd, "CALI") == 0) {
                rotation_home();
                buildAngleLookupTable();
                printAngleLookupTable();
            } else if (strcmp(cmd, "HOME") == 0) {
                rotation_home();
            } else {
                Serial.println(invalid_command_hint);
                input_index = 0;
                return;
            }
            input_index = 0;
        }
        else if (input_index < 31)
        {
            input_buffer[input_index] = c;
            input_index++;
        }
    }
}


void loop()
{
    // calculate dt

    uint32_t current_time = micros();
    uint32_t dt = 0;
    if (current_time < last_time) {
        // handle micros() overflow
        dt = (uint32_t)(current_time + (1<<32 - last_time));
    } else {
        dt = current_time - last_time;
    }
    last_time = current_time;

    // poll sensor and get angle
    rotation_poll(dt);

    parse_uart_input();

    i += 1;
    if (i >= 50)
    {
        i = 0;
        // Serial.println(dt);
        // Serial.println(torque);
        // Serial.println((float)diff / MT6835_CPR * 360, 5);

        // Serial.println(rotation_get() * 180 / PI, 2);

        temperature_poll(dt);
        // Serial.println(temperature_get(), 2);
    }
}
