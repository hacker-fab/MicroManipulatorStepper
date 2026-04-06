#pragma once
#include "utilities/degrees.h"
#include <Wire.h>

//*** Classes *************************************************************************

class Peripheral {
    public:
        Peripheral(TwoWire* wire);
        void begin(bool wait);
        bool is_ready();


        void set_rot(RotDegree deg);
        RotDegree get_desired_rot();
        RotDegree get_cur_rot();

        void set_temp(TempDegree temp);
        TempDegree get_desired_temp();
        TempDegree get_cur_temp();

        void set_vac(bool status);
        bool get_vac();
        
        bool home();
        void estop();
    
    private:
        void write_to_register(uint8_t reg, size_t size, uint8_t* buffer);
        void read_from_register(uint8_t reg, size_t size, uint8_t* buffer);

        void execute(bool rot, bool temp, bool vac);

        TwoWire* wire;
};

//*** Static *************************************************************************

#define STATUS_REG 0x00
#define ESTOP_REG 0xFD
#define HOME_REG 0xFE
#define EXEC_REG 0xFF

#define ROT_TARGET_00_REG 0x10
#define ROT_TARGET_01_REG 0x11
#define ROT_TARGET_02_REG 0x12
#define ROT_TARGET_03_REG 0x13
#define ROT_POS_00_REG 0x14
#define ROT_POS_01_REG 0x15
#define ROT_POS_02_REG 0x16
#define ROT_POS_03_REG 0x17

#define TEMP_TARGET_00_REG 0x20
#define TEMP_TARGET_01_REG 0x21
#define TEMP_TARGET_02_REG 0x22
#define TEMP_TARGET_03_REG 0x23
#define TEMP_POS_00_REG 0x24
#define TEMP_POS_01_REG 0x25
#define TEMP_POS_02_REG 0x26
#define TEMP_POS_03_REG 0x27

#define VAC_REG 0x30
