#ifndef __CONFIG_H__
#define __CONFIG_H__

/* i2c commands */
#define I2C_GET_CAPACITANCE     0x00
#define I2C_SET_ADDRESS         0x01
#define I2C_GET_ADDRESS         0x02
#define I2C_MEASURE_LIGHT       0x03
#define I2C_GET_LIGHT           0x04
#define I2C_GET_TEMPERATURE     0x05
#define I2C_RESET               0x06
#define I2C_GET_VERSION         0x07
#define I2C_GET_DRY_CAPACITANCE 0x08
#define I2C_SET_DRY_CAPACITANCE 0x09
#define I2C_CHIRP               0x0A
#define I2C_FLUSH_I2C_BUFFERS   0x0B
#define I2C_MEASURE_CAPACITANCE 0x0C
#define I2C_GOTO_SLEEP          0x0D
#define I2C_ENABLE_SLEEP        0x0E
#define I2C_GET_FREE_RAM        0x0F

/* eeprom addresses */
#define I2C_ADDRESS_EEPROM  0x01
#define DRY_CAP_HIGH_EEPROM 0x02
#define DRY_CAP_LOW_EEPROM  0x03

#endif
