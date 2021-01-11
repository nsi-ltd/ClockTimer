#ifndef DS3231_H
#define DS3231_H

#include <Arduino.h>
#include <Wire.h>

#define DS3231_ADDRESS 0x68   ///< I2C address for DS3231
#define DS3231_TIME 0x00      ///< Time register
#define DS3231_ALARM1 0x07    ///< Alarm 1 register
#define DS3231_ALARM2 0x0B    ///< Alarm 2 register
#define DS3231_CONTROL 0x0E   ///< Control register
#define DS3231_STATUSREG 0x0F ///< Status register
#define DS3231_TEMPERATUREREG                                                  \
  0x11 ///< Temperature register (high byte - low byte is at 0x12), 10-bit
       ///< temperature value

/** DS3231 SQW pin mode settings */
enum Ds3231SqwPinMode {
  DS3231_OFF = 0x1C,            /**< Off */
  DS3231_SquareWave1Hz = 0x00,  /**<  1Hz square wave */
  DS3231_SquareWave1kHz = 0x08, /**<  1kHz square wave */
  DS3231_SquareWave4kHz = 0x10, /**<  4kHz square wave */
  DS3231_SquareWave8kHz = 0x18  /**<  8kHz square wave */
};

class DS3231 {
public:
    DS3231();
    static uint8_t read_i2c_register(TwoWire* i2c_port, uint8_t addr, uint8_t reg);
    static void write_i2c_register(TwoWire* i2c_port, uint8_t addr, uint8_t reg, uint8_t val);
    static void writeSqwPinMode(TwoWire* i2c_port, Ds3231SqwPinMode mode);
};

#endif
