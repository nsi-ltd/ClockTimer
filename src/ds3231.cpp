#include "ds3231.h"

DS3231::DS3231() {

}

/**************************************************************************/
/*!
    @brief  Read a byte from an I2C register
    @param addr I2C address
    @param reg Register address
    @return Register value
*/
/**************************************************************************/
uint8_t DS3231::read_i2c_register(TwoWire* i2c_port, uint8_t addr, uint8_t reg) {
  i2c_port->beginTransmission(addr);
  i2c_port->write((byte)reg);
  i2c_port->endTransmission();

  i2c_port->requestFrom(addr, (byte)1);
  return i2c_port->read();
}

/**************************************************************************/
/*!
    @brief  Write a byte to an I2C register
    @param addr I2C address
    @param reg Register address
    @param val Value to write
*/
/**************************************************************************/
void DS3231::write_i2c_register(TwoWire* i2c_port, uint8_t addr, uint8_t reg, uint8_t val) {
  i2c_port->beginTransmission(addr);
  i2c_port->write((byte)reg);
  i2c_port->write((byte)val);
  i2c_port->endTransmission();
}

/**************************************************************************/
/*!
    @brief  Set the SQW pin mode
    @param mode Desired mode, see Ds3231SqwPinMode enum
*/
/**************************************************************************/
void DS3231::writeSqwPinMode(TwoWire* i2c_port, Ds3231SqwPinMode mode) {
  uint8_t ctrl;
  ctrl = read_i2c_register(i2c_port, DS3231_ADDRESS, DS3231_CONTROL);

  ctrl &= ~0x04; // turn off INTCON
  ctrl &= ~0x18; // set freq bits to 0

  ctrl |= mode;
  write_i2c_register(i2c_port, DS3231_ADDRESS, DS3231_CONTROL, ctrl);

  // Serial.println( read_i2c_register(DS3231_ADDRESS, DS3231_CONTROL), HEX);
}
