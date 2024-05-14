#include <stdio.h>
#include "AppScheduling.h"
#include "Driver_STM.h"

#include "Bluetooth.h"
#include "Button.h"

typedef struct
{
        uint32 u32nuCnt1ms;
        uint32 u32nuCnt10ms;
        uint32 u32nuCnt100ms;
        uint32 u32nuCnt50ms;
        uint32 u32nuCnt500ms;

}TestCnt;

static void AppNoTask(void);
static void AppTask1ms(void);
static void AppTask10ms(void);
static void AppTask50ms(void);
static void AppTask100ms(void);
static void AppTask500ms(void);
TestCnt stTestCnt;

static void AppNoTask(void){
    //getButtonState(); //read button state by polling
}

static void AppTask1ms(void){
    stTestCnt.u32nuCnt1ms++;
}
static void AppTask10ms(void){
    stTestCnt.u32nuCnt10ms++;
}
static void AppTask50ms(void){
    stTestCnt.u32nuCnt50ms++;
}
static void AppTask100ms(void){
    stTestCnt.u32nuCnt100ms++;
}
static void AppTask500ms(void){
    stTestCnt.u32nuCnt500ms++;

    SendBluetoothData_500ms_cycle(); //send button state, 500ms cycle by bluethooth
}

void AppScheduling(void)
{
    AppNoTask();

    if(stSchedulingInfo.u8nuScheduling1msFlag == 1u)
    {
        stSchedulingInfo.u8nuScheduling1msFlag = 0u;
        AppTask1ms();

        if(stSchedulingInfo.u8nuScheduling10msFlag == 1u)
        {
            stSchedulingInfo.u8nuScheduling10msFlag = 0u;
            AppTask10ms();
        }
        if(stSchedulingInfo.u8nuScheduling50msFlag == 1u)
        {
            stSchedulingInfo.u8nuScheduling50msFlag = 0u;
            AppTask50ms();
        }
        if(stSchedulingInfo.u8nuScheduling100msFlag == 1u)
        {
            stSchedulingInfo.u8nuScheduling100msFlag = 0u;
            AppTask100ms();
        }
        if(stSchedulingInfo.u8nuScheduling500msFlag == 1u)
        {
            stSchedulingInfo.u8nuScheduling500msFlag = 0u;
            AppTask500ms();
        }
    }
}
