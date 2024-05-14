

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
#include "ToF.h"

#include <stdio.h>
#include <stdlib.h>
#include "math.h"

#include "GPT12.h"
extern void setLeftMotorDuty(unsigned int duty);
extern void setRightMotorDuty(unsigned int duty);


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

double yaw = 0;

extern double Target;
extern double TargetV;

extern int g_cnt;
extern unsigned int lMotorDuty;
extern unsigned int rMotorDuty;



//MODE
#define W_F_MODE 1

double Avg_Dist = 0;

int wall_mode_flag = 1;
int wall_mode_count = 0;

int Motion_mode_flag = 0;

int step=0;



extern int Wall_Follow_Distance_Control_Mode;
extern int Wall_Follow_Angle_Control_Mode;

//

//extern int Tof_dist_mm[10];

Motor_Speed App_Motor_Speed;
Decision_Speed App_Decision_Speed;
Wall_Vel App_Wall_Vel;

static void AppNoTask(void)
{
    readEncoderTick();

}

static void AppTask1ms(void)
{
    /*count*/
    stTestCnt.u32nuCnt1ms++;
    Conv_rad_per_sec();

    if(wall_mode_flag)
    {

    //LEFT
    LeftMotor_Vel_PID_Controller(App_Motor_Speed.Left_Motor_Speed, Left_LPF_Encoder.LPF_rad_per_sec);
    //RIGHT
    RightMotor_Vel_PID_Controller(App_Motor_Speed.Right_Motor_Speed, Right_LPF_Encoder.LPF_rad_per_sec);

    //DUTY/////////////////////////////////////
    setLeftMotorDuty(100.0*(Left_Vel_PID.Vin)/12);
    setRightMotorDuty(100.0*(Right_Vel_PID.Vin)/12);
    ///////////////////////////////////////////

    }
    else if(Motion_mode_flag)
    {


        //LEFT
        LeftMotor_Vel_PID_Controller(Left_Pos_PID.Win, Left_LPF_Encoder.LPF_rad_per_sec);
        //RIGHT
        RightMotor_Vel_PID_Controller(Right_Pos_PID.Win, Right_LPF_Encoder.LPF_rad_per_sec);

        setLeftMotorDuty(100.0*(Left_Vel_PID.Vin)/12);
        setRightMotorDuty(100.0*(Right_Vel_PID.Vin)/12);

    }
    //sprintf(str, "Dis:%d Ang:%lf Mode%d Mode%d\t\n",Tof_dist_mm[0],yaw,Wall_Follow_Distance_Control_Mode,Wall_Follow_Angle_Control_Mode);
    sprintf(str, "ML %lf, MR %lf ,mode: %d, Mode Dis %d T1:%d T2:%d",App_Motor_Speed.Left_Motor_Speed,App_Motor_Speed.Right_Motor_Speed,wall_mode_flag,Wall_Follow_Distance_Control_Mode,Tof_dist_mm[0],Tof_dist_mm[1]);
    //sprintf(str, "%lf\t%lf\t%lf\t\n",App_Motor_Speed.Left_Motor_Speed,App_Motor_Speed.Right_Motor_Speed,random_angle);

    //100gap = gogo


}
static void AppTask10ms(void)
{
    /*count*/
    stTestCnt.u32nuCnt10ms++;

    //ADC
    DrvAdc_GetAdcRawGroup0();
    data = stSensorAdcRaw.sen1_Raw;

    //TOF
    Nlink_getTofDistance_Tof(4);
    yaw = calculateYawAngle(Tof_dist_mm[0], Tof_dist_mm[1]);
    Avg_Dist = (Tof_dist_mm[0]+ Tof_dist_mm[1]+5)/2;


    if(Avg_Dist < 500 && Tof_dist_mm[0] != 0 &&Tof_dist_mm[1] != 0) //wall
    {
        if(Tof_dist_mm[0]-Tof_dist_mm[1]>70)
        {
            wall_mode_flag = 0;
            Motion_mode_flag = 1;
        }
        else if(Motion_mode_flag ==1)
        {
            wall_mode_flag = 0;
        }
    }
    else//no
    {

    }

    App_Wall_Vel = Wall_Follow(wall_mode_flag,150,Avg_Dist,0,yaw);

    if(wall_mode_flag)
    {
        App_Motor_Speed = Kinematics(App_Wall_Vel.Wall_Vx,App_Wall_Vel.Wall_Vy,-1.0*App_Wall_Vel.Wall_Wz);
        //위치제어를 위한..
        Left_LPF_Encoder.LPF_Deg = 0;
        Right_LPF_Encoder.LPF_Deg = 0;
        stTestCnt.u32nuCnt1ms=0;

        //speed limit
        if(App_Motor_Speed.Left_Motor_Speed>3) App_Motor_Speed.Left_Motor_Speed = 3;
        if(App_Motor_Speed.Right_Motor_Speed>3) App_Motor_Speed.Right_Motor_Speed = 3;

        if(App_Motor_Speed.Left_Motor_Speed<-3) App_Motor_Speed.Left_Motor_Speed = -3;
        if(App_Motor_Speed.Right_Motor_Speed<-3) App_Motor_Speed.Right_Motor_Speed = -3;
    }
    else if(Motion_mode_flag)
    {

        if(step ==0) //Forward_Motion
        {
            //RIGHT
            Right_Motor_Pos_Trajectory = MakeTrajectoryPos(360,0,5,(double)stTestCnt.u32nuCnt1ms);
            //LEFT
            Left_Motor_Pos_Trajectory = MakeTrajectoryPos(360,0,5,(double)stTestCnt.u32nuCnt1ms);

            if(Right_Motor_Pos_Trajectory == 360)
            {
                step = 1;
                Left_LPF_Encoder.LPF_Deg = 0;
                Right_LPF_Encoder.LPF_Deg = 0;
                stTestCnt.u32nuCnt1ms=0;
            }
        }
        else if(step ==1)//Rotate_Motion
        {
            //RIGHT
            Right_Motor_Pos_Trajectory = MakeTrajectoryPos(330,0,5,(double)stTestCnt.u32nuCnt1ms);
            //LEFT
            Left_Motor_Pos_Trajectory = MakeTrajectoryPos(-330,0,5,(double)stTestCnt.u32nuCnt1ms);

            if(Right_Motor_Pos_Trajectory == 330)
            {
                step = 2;
                Left_LPF_Encoder.LPF_Deg = 0;
                Right_LPF_Encoder.LPF_Deg = 0;
                stTestCnt.u32nuCnt1ms=0;
            }
        }
        else if(step ==2) //Back_Motion
        {
            //RIGHT
            Right_Motor_Pos_Trajectory = MakeTrajectoryPos(-720,0,5,(double)stTestCnt.u32nuCnt1ms);
            //LEFT
            Left_Motor_Pos_Trajectory = MakeTrajectoryPos(-720,0,5,(double)stTestCnt.u32nuCnt1ms);

        }

            App_Motor_Speed.Left_Motor_Speed = 0;
            App_Motor_Speed.Right_Motor_Speed = 0;


        //LEFT
        LeftMotor_Pos_PID_Controller(Left_Motor_Pos_Trajectory,Left_LPF_Encoder.LPF_Deg);
        //RIGHT
        RightMotor_Pos_PID_Controller(Right_Motor_Pos_Trajectory,Right_LPF_Encoder.LPF_Deg);

    }



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

