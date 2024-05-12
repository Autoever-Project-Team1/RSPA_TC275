

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
#include "PID_Controller.h"
#include "Trajectory.h"
#include "Mobile_Kinematics.h"

#include <stdio.h>
#include "math.h"

#include "GPT12.h"
extern void setLeftMotorDuty(unsigned int duty);



char str[100];
uint32 data = 0;


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
        uint32 u32nuCnt1000ms;

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
static void AppTask1000ms(void);

/*********************************************************************************************************************/
/*---------------------------------------------Function Implementations----------------------------------------------*/
/*********************************************************************************************************************/
TestCnt stTestCnt;

uint32 W_RPM;
double w_ref;
double p_ref;

int second_timer = 0;
double Wd = 0;

double pos_target=0;
double random_angle=0;

double Left_Motor_Pos_Trajectory=0;
double Left_Motor_Vel_Trajectory=0;

double Right_Motor_Pos_Trajectory=0;
double Right_Motor_Vel_Trajectory=0;



extern double Target;
extern double TargetV;

extern int g_cnt;
extern unsigned int lMotorDuty;
extern unsigned int rMotorDuty;

Motor_Speed App_Motor_Speed;
Decision_Speed App_Decision_Speed;

static void AppNoTask(void)
{
    readEncoderTick();

}

static void AppTask1ms(void)
{
    /*count*/
    stTestCnt.u32nuCnt1ms++;
    Conv_rad_per_sec();

    ////POSITION_CONTROL/////
    //MAKE_TRAJECTORY
    //
    //POS_CON_START

//    //RIGHT
//    Right_Motor_Pos_Trajectory = MakeTrajectoryPos(360,0,5,(double)stTestCnt.u32nuCnt1ms);
//    Right_Motor_Vel_Trajectory = MakeTrajectoryVel(360,0,5,(double)stTestCnt.u32nuCnt1ms);
//    //LEFT
//    Left_Motor_Pos_Trajectory = MakeTrajectoryPos(360,0,5,(double)stTestCnt.u32nuCnt1ms);
//    Left_Motor_Vel_Trajectory = MakeTrajectoryVel(360,0,5,(double)stTestCnt.u32nuCnt1ms);
//
//    //LEFT
//    LeftMotor_Vel_PID_Controller(Left_Pos_PID.Win, Left_LPF_Encoder.LPF_rad_per_sec);
//    //RIGHT
//    RightMotor_Vel_PID_Controller(Right_Pos_PID.Win, Right_LPF_Encoder.LPF_rad_per_sec);


    //POS_CON_END

    //SPEED_CONTROL
    //
    //SPEED_CON_START

    //LEFT
    LeftMotor_Vel_PID_Controller(App_Motor_Speed.Left_Motor_Speed, Left_LPF_Encoder.LPF_rad_per_sec);
    //RIGHT
    RightMotor_Vel_PID_Controller(App_Motor_Speed.Right_Motor_Speed, Right_LPF_Encoder.LPF_rad_per_sec);


    //
    //SPEED_CON_END


    //DUTY/////////////////////////////////////
    setLeftMotorDuty(100.0*(Left_Vel_PID.Vin)/12);
    setRightMotorDuty(100.0*(Right_Vel_PID.Vin)/12);
    ///////////////////////////////////////////


    sprintf(str, "%lf\t%lf\t%lf\t\n",App_Motor_Speed.Left_Motor_Speed,App_Motor_Speed.Right_Motor_Speed,random_angle);
    //sprintf(str, "%lf\t%lf\t\n",Right_Pos_PID.Win,Right_LPF_Encoder.LPF_rad_per_sec);



}
static void AppTask10ms(void)
{
    /*count*/
    stTestCnt.u32nuCnt10ms++;

    //ADC
    DrvAdc_GetAdcRawGroup0();
    data = stSensorAdcRaw.sen1_Raw;


    //LEFT
    LeftMotor_Pos_PID_Controller(Left_Motor_Pos_Trajectory,Left_LPF_Encoder.LPF_Deg);
    //RIGHT
    RightMotor_Pos_PID_Controller(Right_Motor_Pos_Trajectory,Right_LPF_Encoder.LPF_Deg);


}
static void AppTask50ms(void)
{
    /*count*/
    stTestCnt.u32nuCnt50ms++;

}
static void AppTask100ms(void)
{
    /*count*/
    stTestCnt.u32nuCnt100ms++;

    /*UART*/
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
static void AppTask500ms(void)
{
    /*count*/
    stTestCnt.u32nuCnt500ms++;



}

static void AppTask1000ms(void)
{
    /*count*/
    stTestCnt.u32nuCnt1000ms++;

    //if now angle 20

    random_angle = 15*sin(stTestCnt.u32nuCnt100ms);


    App_Decision_Speed = decision(random_angle);
    App_Motor_Speed = kinematics(App_Decision_Speed.Reference_Vx,App_Decision_Speed.Reference_Vy,App_Decision_Speed.Reference_Wz);

    if(App_Motor_Speed.Left_Motor_Speed>10) App_Motor_Speed.Left_Motor_Speed = 10;
    if(App_Motor_Speed.Right_Motor_Speed>10) App_Motor_Speed.Right_Motor_Speed = 10;


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
        if(stSchedulingInfo.u8nuScheduling1000msFlag == 1u)
        {
            stSchedulingInfo.u8nuScheduling1000msFlag = 0u;
            AppTask1000ms();
        }
    }
}

