#ifndef LCD_CFG_H_
#define LCD_CFG_H_

#include "IfxI2c_I2c.h"

// I2C_LCD_CfgType struct
typedef struct {
    uint8 I2C_LCD_InstanceIndex;
    IfxI2c_I2c_Device *I2C_Handle;
    uint8 I2C_LCD_Address;
    uint8 I2C_LCD_nColumn;
    uint8 I2C_LCD_nRow;
} I2C_LCD_CfgType;

// I2C handle
extern IfxI2c_I2c_Device i2cDevice;

// I2C dev conifg
extern IfxI2c_I2c_deviceConfig i2cDeviceConfig;

extern const I2C_LCD_CfgType I2C_LCD_CfgParam[I2C_LCD_MAX];

#endif /* LCD_CFG_H_ */
