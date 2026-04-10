#if !defined(CONFIG_H)
#define CONFIG_H


// uncomment this to print debug messages through serial
#define DEBUG_PRINTS



/**
 * MT6835 Sensor
 */
#define SENSOR_MOSI PB5
#define SENSOR_MISO PB4
#define SENSOR_SCK PB3
#define SENSOR_SS PA15

#define MT6835_CPR 2097152



/**
 * PWM
 */

// The 72MHz clock cannot support 40kHz with 12-bit resolution, so the actual frequency will be lower
#define PWM_FREQ 40000
#define PWM_RES RESOLUTION_12B_COMPARE_FORMAT
#define PWM_MAX_VALUE 4096

#define MOT_A_POS PA_0
#define MOT_A_NEG PA_1
#define MOT_B_POS PA_2
#define MOT_B_NEG PA_3




/**
 * I2C
 */
#define I2C_SDA PB7
#define I2C_SCL PB6
#define I2C_SLAVE_ADDRESS 0x10


/**
 * Endstop & homing
 */
#define ROTATION_ENDSTOP PA12


/**
 * Heating
 */
#define TEMP_SENSOR_PIN PB0
#define HEATER_PIN PB1

/**
 * Control
 */

#define STEPPER_STEPS 50
#define LOOKUP_TABLE_SIZE (STEPPER_STEPS * 4 + 10) // 10 extra for safety margin



#define VAC_PWM_PIN  PB8          
#define VAC_PWM_FREQ 25000       
#define VAC_PWM_RES  RESOLUTION_12B_COMPARE_FORMAT
#define VAC_PWM_MAX  PWM_MAX_VALUE



#endif // CONFIG_H
