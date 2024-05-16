#include "LcdDecision.h"
#include "AppScheduling.h"
#include "lcd.h"

extern int LCD_Mode_Parking_Go;
extern int LCD_Mode_Parking_Done;
extern int LCD_Mode_Exit_Go;
extern int LCD_Mode_Emergency_Stop;
extern int LCD_Mode_Parking_Back;
extern int LCD_Mode_Exit_Done;
extern int LCD_Mode_Emergency_Stop_F;

void LcdDisplay_Parking_Go()
{
    //Code that outputs "PARKING GO" on the LCD.
    if(LCD_Mode_Parking_Go ==0)
    {
        I2C_LCD_Clear(0);
        I2C_LCD_SetCursor(0, 4, 0);
        I2C_LCD_WriteString(0,"PARKING");
        I2C_LCD_SetCursor(0, 7, 1);
        I2C_LCD_WriteString(0,"GO");
        I2C_LCD_Backlight(0);
        LCD_Mode_Parking_Go =1;//parking go
    }
}
void LcdDisplay_Parking_Done()
{
    //Code that outputs "PARKING DONE" on the LCD.
    if(LCD_Mode_Parking_Done == 0)
    {
        I2C_LCD_Clear(0);
        I2C_LCD_SetCursor(0, 4, 0);
        I2C_LCD_WriteString(0,"PARKING");
        I2C_LCD_SetCursor(0, 6, 1);
        I2C_LCD_WriteString(0,"DONE");
        I2C_LCD_Backlight(0);
        LCD_Mode_Parking_Done =1;
    }
}
void LcdDisplay_Emeregency_Stop()
{
    //Code that outputs "EMERGENCY STOP" on the LCD.
    if(LCD_Mode_Emergency_Stop ==0)
    {
        I2C_LCD_Clear(0);
        I2C_LCD_SetCursor(0, 3, 0);
        I2C_LCD_WriteString(0,"EMERGENCY");
        I2C_LCD_SetCursor(0, 6, 1);
        I2C_LCD_WriteString(0,"STOP");
        I2C_LCD_Backlight(0);
        LCD_Mode_Emergency_Stop=1;
        LCD_Mode_Parking_Back = 0;
        LCD_Mode_Exit_Go = 0;
    }
}
void LcdDisplay_Exit_Go()
{
    //Code that outputs "EXIT GO" on the LCD.
    if(LCD_Mode_Exit_Go==0)
    {
        I2C_LCD_Clear(0);
        I2C_LCD_SetCursor(0, 6, 0);
        I2C_LCD_WriteString(0,"EXIT");
        I2C_LCD_SetCursor(0, 7, 1);
        I2C_LCD_WriteString(0,"GO");
        I2C_LCD_Backlight(0);
        LCD_Mode_Exit_Go =1;
    }
}
void LcdDisplay_Parking_Back()
{
    //Code that outputs "PARKING BACK" on the LCD.
    if(LCD_Mode_Parking_Back ==0)
    {
        I2C_LCD_Clear(0);
        I2C_LCD_SetCursor(0, 4, 0);
        I2C_LCD_WriteString(0,"PARKING");
        I2C_LCD_SetCursor(0, 6, 1);
        I2C_LCD_WriteString(0,"BACK");
        I2C_LCD_Backlight(0);
        LCD_Mode_Parking_Back =1;//
    }
}
void LcdDisplay_Exit_Done()
{
    //Code that outputs "EXIT DONE" on the LCD.
    if(LCD_Mode_Exit_Done ==0)
   {
       I2C_LCD_Clear(0);
       I2C_LCD_SetCursor(0, 6, 0);
       I2C_LCD_WriteString(0,"EXIT");
       I2C_LCD_SetCursor(0, 6, 1);
       I2C_LCD_WriteString(0,"DONE");
       I2C_LCD_Backlight(0);
       LCD_Mode_Exit_Done=1;
   }
}
void LcdDisplay_Emergency_Stop_F()
{
    //Code that outputs "EMERGENCY STOP" on the LCD.
    if(LCD_Mode_Emergency_Stop_F ==0)
    {
        I2C_LCD_Clear(0);
        I2C_LCD_SetCursor(0, 3, 0);
        I2C_LCD_WriteString(0,"EMERGENCY");
        I2C_LCD_SetCursor(0, 6, 1);
        I2C_LCD_WriteString(0,"STOP");
        I2C_LCD_Backlight(0);
        LCD_Mode_Emergency_Stop_F=1;
        LCD_Mode_Parking_Go = 0;
        LCD_Mode_Exit_Go = 0;

    }
}
