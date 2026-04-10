#include <Arduino.h>
#include <Wire.h>

#include "config.h"
#include "i2c_parser.h"




/**
 * I2C Communication
 */
TwoWire i2c_inst(I2C_SDA, I2C_SCL);

uint8_t i2c_cursor = 0;

uint8_t i2c_virtual_reg[256] = {0};
uint8_t i2c_virtual_reg_cursor = 0;

enum I2CStatus {
    MODULE_UNINITIALIZED = 0,
    MODULE_HOMING = 1,
    MODULE_READY = 2,
    MODULE_EMERGENCY_STOPPED = 3,
    MODULE_ERROR = 4,
};


static struct i2c_parser_callbacks parser_callbacks;



/**
 * I2C protocol:
 * 
 * The first byte written is the cursor address.
 * Any bytes after will write the register in increasing order.
 * Example: 01 0A 0B 0C
 * will write 0A to address 1, 0B to 2, 0C to 3.
 * The cursor now stays at address 4.
 * 
 * Write one byte to place the cursor before read,
 * same increasing order applies for reading.
 */
static void on_i2c_receive(int num_bytes){
    
    #ifdef DEBUG_PRINTS
    Serial.print("I2C Receive");
    #endif
    
    int i = 0;
    while (i2c_inst.available()) {
        char c = i2c_inst.read();

        #ifdef DEBUG_PRINTS
        Serial.print(c, HEX);
        Serial.print(" ");
        #endif

        if (i == 0) {
            // first byte is cursor address
            i2c_virtual_reg_cursor = c;
        } else {
            // write to virtual register
            i2c_virtual_reg[i2c_virtual_reg_cursor] = c;
            i2c_virtual_reg_cursor += 1;
        }
        i += 1;
    }

    #ifdef DEBUG_PRINTS
    Serial.println();
    #endif


    // check trigger registers
    if (i2c_virtual_reg[0xFF] != 0)
    {

        uint8_t trigger = i2c_virtual_reg[0xFF];
        if (trigger & 0b0001) {
            // apply rotation
            uint32_t new_rot_target;
            memcpy(&new_rot_target, &i2c_virtual_reg[0x10], sizeof(uint32_t));

            #ifdef DEBUG_PRINTS
            Serial.print("Applying rotation ");
            Serial.println(new_rot_target);
            #endif

            parser_callbacks.set_rotation_target(new_rot_target / 360000.0f * 2 * PI);
        }
        if (trigger & 0b0010) {
            // apply temperature
            uint32_t new_temp_setpoint;
            memcpy(&new_temp_setpoint, &i2c_virtual_reg[0x20], sizeof(uint32_t));
            
            #if defined(DEBUG_PRINTS)
            Serial.print("Applying temperature ");
            Serial.println(new_temp_setpoint);
            #endif

            parser_callbacks.set_temperature_setpoint(new_temp_setpoint / 50000.0f);
        }
        if (trigger & 0b0100) {
            // apply vacuum

            #ifdef DEBUG_PRINTS
            Serial.print("Applying vacuum ");
            Serial.println(i2c_virtual_reg[0x30]);
            #endif
            parser_callbacks.set_vacuum(i2c_virtual_reg[0x30]);
        }
        // clear trigger register
        i2c_virtual_reg[0xFF] = 0;
    }
}

static void on_i2c_request(){
    #ifdef DEBUG_PRINTS
    Serial.println("I2C Request");
    #endif

    i2c_virtual_reg[0] = 1;

    // update virtual register before read
    float rot_target = parser_callbacks.get_rotation_target();
    uint32_t rot_target_int = (uint32_t)(rot_target / (2 * PI) * 360000);
    memcpy(&i2c_virtual_reg[0x10], &rot_target_int, sizeof(uint32_t));

    float rot_current = parser_callbacks.get_rotation();
    while (rot_current < 0) {
        rot_current += 2 * PI;
    }
    uint32_t rot_current_int = (uint32_t)(rot_current / (2.0 * PI) * 360000);
    memcpy(&i2c_virtual_reg[0x14], &rot_current_int, sizeof(uint32_t));


    float temp_setpoint = parser_callbacks.get_temperature_setpoint();
    uint32_t temp_setpoint_int = (uint32_t)(temp_setpoint * 50000);
    memcpy(&i2c_virtual_reg[0x20], &temp_setpoint_int, sizeof(uint32_t));

    float temp_current = parser_callbacks.get_temperature();
    uint32_t temp_current_int = (uint32_t)(temp_current * 50000);
    memcpy(&i2c_virtual_reg[0x24], &temp_current_int, sizeof(uint32_t));

    uint16_t vacuum = parser_callbacks.get_vacuum();
    i2c_virtual_reg[0x30] = vacuum;

    // write buffer
    i2c_inst.write(i2c_virtual_reg + i2c_virtual_reg_cursor, 8);

    #ifdef DEBUG_PRINTS
    Serial.print("Cursor: 0x");
    Serial.print(i2c_virtual_reg_cursor, HEX);
    Serial.print(" Data: 0x");
    Serial.println(i2c_virtual_reg[i2c_virtual_reg_cursor], HEX);
    #endif
}


void i2c_parser_init(i2c_parser_callbacks callbacks){
    i2c_inst.begin(I2C_SLAVE_ADDRESS);
    parser_callbacks = callbacks;
    i2c_inst.onReceive(on_i2c_receive);
    i2c_inst.onRequest(on_i2c_request);
}

