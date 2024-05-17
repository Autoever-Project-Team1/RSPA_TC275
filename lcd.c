#include <lcd.h>
#include <lcd_cfg.h>
#include "IfxStm.h"
#include "IfxI2c_I2c.h"
#include "IfxI2c_PinMap.h"
#include "I2c/Std/IfxI2c.h"
#include "Bsp.h"
#include "asclin.h"

#define I2C_SCL_PIN IfxI2c0_SCL_P15_4_INOUT  // SCL 핀
#define I2C_SDA_PIN IfxI2c0_SDA_P15_5_INOUT  // SDA 핀

IfxI2c_I2c i2cHandle;            // I2C handler

//i2c device handle
IfxI2c_I2c_Device i2cDevice;

IfxI2c_I2c_deviceConfig i2cDeviceConfig;

const I2C_LCD_CfgType I2C_LCD_CfgParam[I2C_LCD_MAX] =
{
    {
        I2C_LCD_1, // Index of I2C_LCD Instance #1
        &i2cDevice, // Hardware I2C Module's Handle
        0x27 , // Hardware I2C_LCD Device Address //
        16,   // LCD Columns Count
        2     // LCD Rows Count
    }
};

// CMD
#define LCD_CLEARDISPLAY        0x01
#define LCD_RETURNHOME          0x02
#define LCD_ENTRYMODESET        0x04
#define LCD_DISPLAYCONTROL      0x08
#define LCD_CURSORSHIFT         0x10
#define LCD_FUNCTIONSET         0x20
#define LCD_SETCGRAMADDR        0x40
#define LCD_SETDDRAMADDR        0x80
// DISPLAY ENTRY
#define LCD_ENTRYRIGHT          0x00
#define LCD_ENTRYLEFT           0x02
#define LCD_ENTRYSHIFTINCREMENT 0x01
#define LCD_ENTRYSHIFTDECREMENT 0x00
// DISPLAY CONTROL
#define LCD_DISPLAYON           0x04
#define LCD_DISPLAYOFF          0x00
#define LCD_CURSORON            0x02
#define LCD_CURSOROFF           0x00
#define LCD_BLINKON             0x01
#define LCD_BLINKOFF            0x00
// CURSOR MOTION
#define LCD_DISPLAYMOVE         0x08
#define LCD_CURSORMOVE          0x00
#define LCD_MOVERIGHT           0x04
#define LCD_MOVELEFT            0x00
// FUNCTION SET
#define LCD_8BITMODE            0x10
#define LCD_4BITMODE            0x00
#define LCD_2LINE               0x08
#define LCD_1LINE               0x00
#define LCD_5x10DOTS            0x04
#define LCD_5x8DOTS             0x00
// BACKLIGHT CONTROL
#define LCD_BACKLIGHT           0x08
#define LCD_NOBACKLIGHT         0x00
#define EN                      0x04  // Enable bit
#define RW                      0x02  // Read/Write bit
#define RS                      0x01  // Register select bit

typedef struct I2C_LCD_InfoParam_s
{
    uint8 DisplayCtrl;
    uint8 BacklightVal;
} I2C_LCD_InfoParam_t;

static I2C_LCD_InfoParam_t I2C_LCD_InfoParam_g[I2C_LCD_MAX];

static void I2C_LCD_ExpanderWrite(uint8 I2C_LCD_InstanceIndex, uint8 DATA)
{
    uint8 TxData = (DATA) | I2C_LCD_InfoParam_g[I2C_LCD_InstanceIndex].BacklightVal;
    IfxI2c_I2c_initDeviceConfig(&i2cDeviceConfig, &i2cHandle);
     i2cDeviceConfig.deviceAddress = (0x27 << 1);
     IfxI2c_I2c_initDevice(&i2cDevice, &i2cDeviceConfig);
     IfxI2c_I2c_Status status = IfxI2c_I2c_write(I2C_LCD_CfgParam[I2C_LCD_InstanceIndex].I2C_Handle, &TxData, 1);


     if (status != IfxI2c_I2c_Status_ok) {
           // Handle error appropriately
         //_out_uart3('1');
     }
     else{
         //_out_uart3('2');
     }
}

static void I2C_LCD_EnPulse(uint8 I2C_LCD_InstanceIndex, uint8 DATA)
{
    I2C_LCD_ExpanderWrite(I2C_LCD_InstanceIndex, (DATA | EN)); // En high
    waitTime(IfxStm_getTicksFromMicroseconds(&MODULE_STM0, 2)); // enable pulse must be >450ns

    I2C_LCD_ExpanderWrite(I2C_LCD_InstanceIndex, (DATA & ~EN)); // En low
    waitTime(IfxStm_getTicksFromMicroseconds(&MODULE_STM0, 50)); // commands need > 37us to settle
}

static void I2C_LCD_Write4Bits(uint8 I2C_LCD_InstanceIndex, uint8 Val)
{
    I2C_LCD_ExpanderWrite(I2C_LCD_InstanceIndex, Val);
    I2C_LCD_EnPulse(I2C_LCD_InstanceIndex, Val);
}

static void I2C_LCD_Send(uint8 I2C_LCD_InstanceIndex, uint8 Val, uint8 Mode)
{
    uint8 HighNib = Val & 0xF0;
    uint8 LowNib = (Val << 4) & 0xF0;
    I2C_LCD_Write4Bits(I2C_LCD_InstanceIndex, (HighNib) | Mode);
    I2C_LCD_Write4Bits(I2C_LCD_InstanceIndex, (LowNib) | Mode);
}

static void I2C_LCD_Cmd(uint8 I2C_LCD_InstanceIndex, uint8 CMD)
{
    I2C_LCD_Send(I2C_LCD_InstanceIndex, CMD, 0);
}

static void I2C_LCD_Data(uint8 I2C_LCD_InstanceIndex, uint8 DATA)
{
    I2C_LCD_Send(I2C_LCD_InstanceIndex, DATA, RS);
}



void I2C_Init(void) {
    // I2C moudule config
    IfxI2c_I2c_Config i2cConfig;
    IfxI2c_I2c_initConfig(&i2cConfig, &MODULE_I2C0);

    // I2C pin
    const IfxI2c_Pins i2cPins = {
        .scl = &I2C_SCL_PIN,
        .sda = &I2C_SDA_PIN,
        .padDriver = IfxPort_PadDriver_cmosAutomotiveSpeed1
    };
    i2cConfig.pins = &i2cPins;
    i2cConfig.baudrate = 400000;

    // I2C module init
    IfxI2c_I2c_initModule(&i2cHandle, &i2cConfig);

    // I2C device init
    IfxI2c_I2c_initDeviceConfig(&i2cDeviceConfig, &i2cHandle);

    i2cDeviceConfig.deviceAddress = I2C_LCD_CfgParam[I2C_LCD_1].I2C_LCD_Address;
    IfxI2c_I2c_initDevice(&i2cDevice, &i2cDeviceConfig);
    //error check


}


void I2C_LCD_Init(uint8 I2C_LCD_InstanceIndex)
{
    waitTime(IfxStm_getTicksFromMilliseconds(&MODULE_STM0, 50)); // Wait at least 40ms after power up
    I2C_LCD_Cmd(I2C_LCD_InstanceIndex, 0x30);
    waitTime(IfxStm_getTicksFromMilliseconds(&MODULE_STM0, 5));  // Delay > 4.1ms
    I2C_LCD_Cmd(I2C_LCD_InstanceIndex, 0x30);
    waitTime(IfxStm_getTicksFromMilliseconds(&MODULE_STM0, 5));  // Delay > 4.1ms
    I2C_LCD_Cmd(I2C_LCD_InstanceIndex, 0x30);
    waitTime(IfxStm_getTicksFromMicroseconds(&MODULE_STM0, 150)); // Delay > 100μs
    I2C_LCD_Cmd(I2C_LCD_InstanceIndex, 0x02);
    // Configure the LCD
    I2C_LCD_Cmd(I2C_LCD_InstanceIndex, LCD_FUNCTIONSET | LCD_4BITMODE | LCD_2LINE | LCD_5x8DOTS);
    I2C_LCD_Cmd(I2C_LCD_InstanceIndex, LCD_DISPLAYCONTROL | LCD_DISPLAYON | LCD_CURSOROFF | LCD_BLINKOFF);
    I2C_LCD_Cmd(I2C_LCD_InstanceIndex, LCD_ENTRYMODESET | LCD_ENTRYLEFT | LCD_ENTRYSHIFTDECREMENT);
    I2C_LCD_InfoParam_g[I2C_LCD_InstanceIndex].DisplayCtrl = LCD_DISPLAYON | LCD_CURSOROFF | LCD_BLINKOFF;
    I2C_LCD_InfoParam_g[I2C_LCD_InstanceIndex].BacklightVal = LCD_BACKLIGHT;
    // Clear the LCD
    I2C_LCD_Clear(I2C_LCD_InstanceIndex);

    //error check

}

void I2C_LCD_Clear(uint8 I2C_LCD_InstanceIndex)
{
    I2C_LCD_Cmd(I2C_LCD_InstanceIndex, LCD_CLEARDISPLAY);
    waitTime(IfxStm_getTicksFromMilliseconds(&MODULE_STM0, 2));
}

void I2C_LCD_Home(uint8 I2C_LCD_InstanceIndex)
{
    I2C_LCD_Cmd(I2C_LCD_InstanceIndex, LCD_RETURNHOME);
    waitTime(IfxStm_getTicksFromMilliseconds(&MODULE_STM0, 2));
}

void I2C_LCD_SetCursor(uint8 I2C_LCD_InstanceIndex, uint8 Col, uint8 Row)
{
    int Row_Offsets[] = {0x00, 0x40, 0x14, 0x54};
    if (Row > I2C_LCD_CfgParam[I2C_LCD_InstanceIndex].I2C_LCD_nRow)
    {
        Row = I2C_LCD_CfgParam[I2C_LCD_InstanceIndex].I2C_LCD_nRow - 1;
    }
    I2C_LCD_Cmd(I2C_LCD_InstanceIndex, LCD_SETDDRAMADDR | (Col + Row_Offsets[Row]));
}

void I2C_LCD_WriteChar(uint8 I2C_LCD_InstanceIndex, char Ch)
{
    I2C_LCD_Data(I2C_LCD_InstanceIndex, Ch);
}

void I2C_LCD_WriteString(uint8 I2C_LCD_InstanceIndex, char *Str)
{
    while (*Str)
    {
        I2C_LCD_Data(I2C_LCD_InstanceIndex, *Str++);
    }
}

void I2C_LCD_ShiftLeft(uint8 I2C_LCD_InstanceIndex)
{
    I2C_LCD_Cmd(I2C_LCD_InstanceIndex, LCD_CURSORSHIFT | LCD_DISPLAYMOVE | LCD_MOVELEFT);
}

void I2C_LCD_ShiftRight(uint8 I2C_LCD_InstanceIndex)
{
    I2C_LCD_Cmd(I2C_LCD_InstanceIndex, LCD_CURSORSHIFT | LCD_DISPLAYMOVE | LCD_MOVERIGHT);
}

void I2C_LCD_Backlight(uint8 I2C_LCD_InstanceIndex)
{
    I2C_LCD_InfoParam_g[I2C_LCD_InstanceIndex].BacklightVal = LCD_BACKLIGHT;
    I2C_LCD_ExpanderWrite(I2C_LCD_InstanceIndex, 0);
}

void I2C_LCD_NoBacklight(uint8 I2C_LCD_InstanceIndex)
{
    I2C_LCD_InfoParam_g[I2C_LCD_InstanceIndex].BacklightVal = LCD_NOBACKLIGHT;
    I2C_LCD_ExpanderWrite(I2C_LCD_InstanceIndex, 0);
}

void I2C_LCD_Display(uint8 I2C_LCD_InstanceIndex)
{
    I2C_LCD_InfoParam_g[I2C_LCD_InstanceIndex].DisplayCtrl |= LCD_DISPLAYON;
    I2C_LCD_Cmd(I2C_LCD_InstanceIndex, LCD_DISPLAYCONTROL | I2C_LCD_InfoParam_g[I2C_LCD_InstanceIndex].DisplayCtrl);
}

void I2C_LCD_NoDisplay(uint8 I2C_LCD_InstanceIndex)
{
    I2C_LCD_InfoParam_g[I2C_LCD_InstanceIndex].DisplayCtrl &= ~LCD_DISPLAYON;
    I2C_LCD_Cmd(I2C_LCD_InstanceIndex, LCD_DISPLAYCONTROL | I2C_LCD_InfoParam_g[I2C_LCD_InstanceIndex].DisplayCtrl);
}

void I2C_LCD_Cursor(uint8 I2C_LCD_InstanceIndex)
{
    I2C_LCD_InfoParam_g[I2C_LCD_InstanceIndex].DisplayCtrl |= LCD_CURSORON;
    I2C_LCD_Cmd(I2C_LCD_InstanceIndex, LCD_DISPLAYCONTROL | I2C_LCD_InfoParam_g[I2C_LCD_InstanceIndex].DisplayCtrl);
}

void I2C_LCD_NoCursor(uint8 I2C_LCD_InstanceIndex)
{
    I2C_LCD_InfoParam_g[I2C_LCD_InstanceIndex].DisplayCtrl &= ~LCD_CURSORON;
    I2C_LCD_Cmd(I2C_LCD_InstanceIndex, LCD_DISPLAYCONTROL | I2C_LCD_InfoParam_g[I2C_LCD_InstanceIndex].DisplayCtrl);
}

void I2C_LCD_Blink(uint8 I2C_LCD_InstanceIndex)
{
    I2C_LCD_InfoParam_g[I2C_LCD_InstanceIndex].DisplayCtrl |= LCD_BLINKON;
    I2C_LCD_Cmd(I2C_LCD_InstanceIndex, LCD_DISPLAYCONTROL | I2C_LCD_InfoParam_g[I2C_LCD_InstanceIndex].DisplayCtrl);
}

void I2C_LCD_NoBlink(uint8 I2C_LCD_InstanceIndex)
{
    I2C_LCD_InfoParam_g[I2C_LCD_InstanceIndex].DisplayCtrl &= ~LCD_BLINKON;
    I2C_LCD_Cmd(I2C_LCD_InstanceIndex, LCD_DISPLAYCONTROL | I2C_LCD_InfoParam_g[I2C_LCD_InstanceIndex].DisplayCtrl);
}

void I2C_LCD_CreateCustomChar(uint8 I2C_LCD_InstanceIndex, uint8 CharIndex, const uint8* CharMap)
{
    CharIndex &= 0x07;
    I2C_LCD_Cmd(I2C_LCD_InstanceIndex, LCD_SETCGRAMADDR | (CharIndex << 3));
    for (int i = 0; i < 8; i++)
    {
        I2C_LCD_Send(I2C_LCD_InstanceIndex, CharMap[i], RS);
    }
}

void I2C_LCD_PrintCustomChar(uint8 I2C_LCD_InstanceIndex, uint8 CharIndex)
{
    I2C_LCD_Send(I2C_LCD_InstanceIndex, CharIndex, RS);
}
