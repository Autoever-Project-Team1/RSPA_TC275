/*ETC*/
#include "Ifx_Types.h"
#include "IfxCpu.h"
#include "IfxScuWdt.h"
/*IO*/
#include "lcd.h"
#include <ToF.h>
/*DRIVER*/
#include "GPT12.h"
#include "GtmTomPwm.h"
#include "Driver_STM.h"
#include "Driver_ADC.h"
#include "asclin.h"
/*CONTROL*/
#include "PID_Controller.h"
/*DECISION*/
#include "AppScheduling.h"
/*PREDECISION*/

/*********************************************************************************************************************/
/*------------------------------------------------------Macros-------------------------------------------------------*/
/*********************************************************************************************************************/
#define BREAK_A     &MODULE_P02,7
#define BREAK_B     &MODULE_P02,6
/*********************************************************************************************************************/
/*-------------------------------------------------Global variables--------------------------------------------------*/
/*********************************************************************************************************************/

IfxCpu_syncEvent g_cpuSyncEvent = 0;

void core0_main(void)
{
    
    /* !!WATCHDOG0 AND SAFETY WATCHDOG ARE DISABLED HERE!!
     * Enable the watchdogs and service them periodically if it is required
     */
    IfxScuWdt_disableCpuWatchdog(IfxScuWdt_getCpuWatchdogPassword());
    IfxScuWdt_disableSafetyWatchdog(IfxScuWdt_getSafetyWatchdogPassword());
    
    /* Wait for CPU sync event */
    IfxCpu_emitEvent(&g_cpuSyncEvent);
    IfxCpu_waitEvent(&g_cpuSyncEvent, 1);


    /****************************INITIALIZE**********************************/
    initLED();
    _init_uart3();
    _init_uart2();
    unsigned char c;
    Driver_Stm_Init();
    DrvAdcInit();
    GtmTomPwmHl_init();
    Encoder_Init();
    Init_DCMotors();
    IfxPort_setPinLow(BREAK_A);    /* 모터 정지 (1: 정지, 0: PWM-B에 따라 동작)*/
    IfxPort_setPinLow(BREAK_B);    /* 모터 정지 (1: 정지, 0: PWM-B에 따라 동작)*/
    PID_Vel_Controller_Init();
    PID_Pos_Controller_Init();
    PID_App_Controller_Init();
    Init_Buzzer();
    Bluetooth_init();
    /*********************************************************************/
    IfxCpu_enableInterrupts();


    /*****LCD_INITIALIZE*****************/
    I2C_Init();
    I2C_LCD_Init(0);

    I2C_LCD_Clear(0);
    I2C_LCD_SetCursor(0, 2, 0);
    I2C_LCD_WriteString(0,"AUTOPARKING");
    I2C_LCD_SetCursor(0, 5, 1);
    I2C_LCD_WriteString(0,"SYSTEM");
    I2C_LCD_Backlight(0);
    /***********************************/

    while(1)
    {
        AppScheduling();
    }
}
