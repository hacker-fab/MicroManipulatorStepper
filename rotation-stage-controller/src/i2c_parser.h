#if !defined(I2C_PARSER_H)
#define I2C_PARSER_H

#include <Arduino.h>

struct i2c_parser_callbacks
{
    uint8_t (*home_rotation)(void);
    void (*set_rotation_target)(float angle_rad);
    float (*get_rotation_target)(void);
    float (*get_rotation)(void);

    void (*set_temperature_setpoint)(float temp);
    float (*get_temperature_setpoint)(void);
    float (*get_temperature)(void);

    void (*set_vacuum)(uint16_t duty);
    uint16_t (*get_vacuum)(void);
};


void i2c_parser_init(i2c_parser_callbacks callbacks);



#endif // I2C_PARSER_H
