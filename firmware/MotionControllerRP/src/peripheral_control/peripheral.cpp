#include <hw_config.h>
#include "peripheral.h"
#include <utilities/logging.h>

Peripheral::Peripheral(TwoWire* wire_ptr) {
  // initialize the two wire system
  wire = wire_ptr;
}

void Peripheral::begin(bool wait) {
  wire->begin();

  // if wait is enable then wait for the status register to return any non 0 value
  while (wait){
    if(is_ready()){
      break;
    }
    delay(100);
  }
}

bool Peripheral::is_ready() {
  uint8_t buffer = 0;
  read_from_register(STATUS_REG, 1, &buffer);
  return buffer!=0;
}


void Peripheral::set_rot(RotDegree deg) {
  FourByteArray trans_val = deg.get_trans_value();
  write_to_register(ROT_TARGET_00_REG, 1, &trans_val.bytes[0]);
  write_to_register(ROT_TARGET_01_REG, 1, &trans_val.bytes[1]);
  write_to_register(ROT_TARGET_02_REG, 1, &trans_val.bytes[2]);
  write_to_register(ROT_TARGET_03_REG, 1, &trans_val.bytes[3]);
  execute(true, false, false);
}

RotDegree Peripheral::get_desired_rot() {
  FourByteArray trans_val = FourByteArray{0};
  read_from_register(ROT_TARGET_00_REG, 1, &trans_val.bytes[0]);
  read_from_register(ROT_TARGET_01_REG, 1, &trans_val.bytes[1]);
  read_from_register(ROT_TARGET_02_REG, 1, &trans_val.bytes[2]);
  read_from_register(ROT_TARGET_03_REG, 1, &trans_val.bytes[3]);

  return RotDegree(trans_val.integer);
}

RotDegree Peripheral::get_cur_rot() {
  FourByteArray trans_val = FourByteArray{0};
  read_from_register(ROT_POS_00_REG, 1, &trans_val.bytes[0]);
  read_from_register(ROT_POS_01_REG, 1, &trans_val.bytes[1]);
  read_from_register(ROT_POS_02_REG, 1, &trans_val.bytes[2]);
  read_from_register(ROT_POS_03_REG, 1, &trans_val.bytes[3]);

  return RotDegree(trans_val.integer);
}

void Peripheral::set_temp(TempDegree deg) {
  FourByteArray trans_val = deg.get_trans_value();
  write_to_register(TEMP_TARGET_00_REG, 1, &trans_val.bytes[0]);
  write_to_register(TEMP_TARGET_01_REG, 1, &trans_val.bytes[1]);
  write_to_register(TEMP_TARGET_02_REG, 1, &trans_val.bytes[2]);
  write_to_register(TEMP_TARGET_03_REG, 1, &trans_val.bytes[3]);
  execute(false, true, false);
}

TempDegree Peripheral::get_desired_temp() {
  FourByteArray trans_val = FourByteArray{0};
  read_from_register(TEMP_TARGET_00_REG, 1, &trans_val.bytes[0]);
  read_from_register(TEMP_TARGET_01_REG, 1, &trans_val.bytes[1]);
  read_from_register(TEMP_TARGET_02_REG, 1, &trans_val.bytes[2]);
  read_from_register(TEMP_TARGET_03_REG, 1, &trans_val.bytes[3]);

  return TempDegree(trans_val.integer);
}

TempDegree Peripheral::get_cur_temp() {
  FourByteArray trans_val = FourByteArray{0};
  read_from_register(TEMP_POS_00_REG, 1, &trans_val.bytes[0]);
  read_from_register(TEMP_POS_01_REG, 1, &trans_val.bytes[1]);
  read_from_register(TEMP_POS_02_REG, 1, &trans_val.bytes[2]);
  read_from_register(TEMP_POS_03_REG, 1, &trans_val.bytes[3]);

  return TempDegree(trans_val.integer);
}


void Peripheral::set_vac(bool status) {
  write_to_register(VAC_REG, 1, (uint8_t*)&status);
  execute(false, false, true);
}

bool Peripheral::get_vac() {
  bool status;
  read_from_register(VAC_REG, 1, (uint8_t*)&status);
  return status;
}

bool Peripheral::home() {
  LOG_INFO("homing peripheral...");
  write_to_register(HOME_REG, 0, NULL);
  LOG_INFO("homed peripheral");
  return true;
}
void Peripheral::estop() {
  write_to_register(ESTOP_REG, 0, NULL);
}

void Peripheral::execute(bool rot, bool temp, bool vac) {
  uint8_t base = 0x00;
  if(rot){
    base+=1;
  }
  if(temp){
    base+=2;
  }
  if(vac){
    base+=4;
  }
  write_to_register(EXEC_REG, 1, &base);
}


// TwoWire Helpers

void Peripheral::write_to_register(uint8_t reg, size_t size, uint8_t* buffer){
    wire->beginTransmission(PERIPHERAL_I2C_ADDRESS);
    wire->write(reg);
    for(size_t i=0;i<size;i++){
      wire->endTransmission(false);
      wire->write(buffer[i]);
    }
    wire->endTransmission(true);
}

void Peripheral::read_from_register(uint8_t reg, size_t size, uint8_t* buffer){
    wire->beginTransmission(PERIPHERAL_I2C_ADDRESS);
    wire->write(reg);
    wire->endTransmission(false);
    wire->requestFrom(PERIPHERAL_I2C_ADDRESS, (uint8_t)size);
    for(size_t i=0;i<size;i++){
      buffer[i] = wire->read();
    }
    wire->endTransmission(true);
}
