

/*********************************************************************************************************************/
/*-----------------------------------------------------Includes------------------------------------------------------*/
/*********************************************************************************************************************/
#include "AppScheduling.h"
#include "Driver_STM.h"
#include "Blinky_LED.h"
#include "Driver_ADC.h"
#include "asclin.h"
#include "GtmTomPwm.h"
#include "Encoder.h"

#include "ToF.h"
#include "Observer.h"

#include "Ultrasonic.h"

#include <stdio.h>

char str[100];
char str2[100];
uint32 data = 0;

uint32 f_motor = 0;

uint32 sec = 0;




/*********************************************************************************************************************/
/*------------------------------------------------------Macros-------------------------------------------------------*/
/*********************************************************************************************************************/
typedef struct
{
        uint32 u32nuCnt1ms;
        uint32 u32nuCnt10ms;
        uint32 u32nuCnt100ms;
        uint32 u32nuCnt50ms;
        uint32 u32nuCnt500ms;

}TestCnt;
/*********************************************************************************************************************/
/*-------------------------------------------------Global variables--------------------------------------------------*/
/*********************************************************************************************************************/

/*********************************************************************************************************************/
/*--------------------------------------------Private Variables/Constants--------------------------------------------*/
/*********************************************************************************************************************/

/*********************************************************************************************************************/
/*------------------------------------------------Function Prototypes------------------------------------------------*/
/*********************************************************************************************************************/
static void AppNoTask(void);
static void AppTask1ms(void);
static void AppTask10ms(void);
static void AppTask50ms(void);
static void AppTask100ms(void);
static void AppTask500ms(void);
/*********************************************************************************************************************/
/*---------------------------------------------Function Implementations----------------------------------------------*/
/*********************************************************************************************************************/
TestCnt stTestCnt;

static void AppNoTask(void){
    readEncoderTick();
    EncoderPulsesToAngle(g_Encoder.AB_Cnt);
}

static void AppTask1ms(void){
    stTestCnt.u32nuCnt1ms++;
    //READ GROUP TOF!
    Nlink_getTofDistance_Tof(4);
    updateObserver(&observer,Tof_dist_mm[0], Tof_dist_mm[1]);

}
static void AppTask10ms(void){
    stTestCnt.u32nuCnt10ms++;
}
static void AppTask50ms(void){
    stTestCnt.u32nuCnt50ms++;
//    if(data > 4000)
//        //blinkLED();


}
static void AppTask100ms(void){
    stTestCnt.u32nuCnt100ms++;

    //double yaw = calculateYawAngle(Tof_dist_mm[0], Tof_dist_mm[1]);
    double yaw = getYawFromObserver(&observer);
    sprintf(str, "%d\t%d : %lf\t%lf\n", Tof_dist_mm[0], Tof_dist_mm[1], yaw, yaw/PI*180.0);
    //sprintf(str, "%d\t%d\t%d\t%d\t%d\n", Tof_dist_mm[0], Tof_dist_mm[1], Tof_dist_mm[2], Tof_dist_mm[3], Tof_dist_mm[4]);


    char idx = 0;
    while(1)
    {
        if(str[idx] == '\0')
        {
            break;
        }
        _out_uart3(str[idx++]);
    }
}
static void AppTask500ms(void){
    stTestCnt.u32nuCnt500ms++;



    if(stTestCnt.u32nuCnt500ms % 2 == 0){
        sec++;
    }
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
