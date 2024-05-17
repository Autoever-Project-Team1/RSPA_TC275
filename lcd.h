#ifndef I2C_LCD_H_
#define I2C_LCD_H_

#include "stdint.h"
#include "IfxI2c_I2c.h"

#define I2C_LCD_MAX 1   // Maximum Number of I2C_LCD Modules in Your Project
#define I2C_LCD_1   0   // I2C_LCD Instance Number 1 (Add more if you need)

// GUIDELINE SEQUENCE
#define SEQ_0       0
#define SEQ_1       1
#define SEQ_2       2
#define SEQ_3       3

//-----[ Prototypes For All User External Functions ]-----

void I2C_LCD_Init(uint8 I2C_LCD_InstanceIndex);
void I2C_LCD_Clear(uint8 I2C_LCD_InstanceIndex);
void I2C_LCD_Home(uint8 I2C_LCD_InstanceIndex);
void I2C_LCD_SetCursor(uint8 I2C_LCD_InstanceIndex, uint8 Col, uint8 Row);
void I2C_LCD_WriteChar(uint8 I2C_LCD_InstanceIndex, char Ch);
void I2C_LCD_WriteString(uint8 I2C_LCD_InstanceIndex, char* Str);

void I2C_LCD_ShiftLeft(uint8 I2C_LCD_InstanceIndex);
void I2C_LCD_ShiftRight(uint8 I2C_LCD_InstanceIndex);

void I2C_LCD_Backlight(uint8 I2C_LCD_InstanceIndex);
void I2C_LCD_NoBacklight(uint8 I2C_LCD_InstanceIndex);
void I2C_LCD_Display(uint8 I2C_LCD_InstanceIndex);
void I2C_LCD_NoDisplay(uint8 I2C_LCD_InstanceIndex);
void I2C_LCD_Cursor(uint8 I2C_LCD_InstanceIndex);
void I2C_LCD_NoCursor(uint8 I2C_LCD_InstanceIndex);
void I2C_LCD_Blink(uint8 I2C_LCD_InstanceIndex);
void I2C_LCD_NoBlink(uint8 I2C_LCD_InstanceIndex);

void I2C_LCD_CreateCustomChar(uint8 I2C_LCD_InstanceIndex, uint8 CharIndex, const uint8* CharMap);
void I2C_LCD_PrintCustomChar(uint8 I2C_LCD_InstanceIndex, uint8 CharIndex);

#endif /* I2C_LCD_H_ */
