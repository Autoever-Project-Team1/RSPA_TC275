/*********************************************************************************************************************/
/*-----------------------------------------------------Includes------------------------------------------------------*/
/*********************************************************************************************************************/
/*IO*/
#include "ToF.h"
#include "Buzzer.h"
#include "Bluetooth.h";
#include "Encoder.h"
#include "Driver_ADC.h"
/*DRIVER*/
#include "Driver_STM.h"
#include "asclin.h"
#include "GtmTomPwm.h"
#include "GPT12.h"
/*CONTROL*/
#include "PID_Controller.h"
#include "Trajectory.h"
#include "Mobile_Kinematics.h"
/*PREDECISION*/
#include "ObstacleDetection.h"
#include "MotionDecision.h"
#include "LcdDecision.h"
/*DECISION*/
#include "AppScheduling.h"
/*ETC*/
#include <stdio.h>
#include <stdlib.h>
#include "math.h"

/*********************************************************************************************************************/
/*------------------------------------------------------Macros-------------------------------------------------------*/
/*********************************************************************************************************************/
#define BREAK_A     &MODULE_P02,7
#define BREAK_B     &MODULE_P02,6
#define STATE_NORMAL_WALL_FOLLOW 0
#define STATE_PARKING_WALL_FOLLOW 1
#define STATE_EXIT_WALL_FOLLOW 2
#define TOF_OFFSET 5
#define TOF_DETECT_DIS 500
#define TOF_DIF 50
#define FRONT_EMERGENCY_DIS 70
#define PARKING_FORWARD 0
#define PARKING_CWROTATE 1
#define PARKING_BACKWARD 2
#define PARKING_ING 3
#define EXIT_CCWROTATE 4
#define EXIT_DONE 5
#define FINISH 6
/*********************************************************************************************************************/
/*-------------------------------------------------Global variables--------------------------------------------------*/
/*********************************************************************************************************************/
extern void setLeftMotorDuty(unsigned int duty);
extern void setRightMotorDuty(unsigned int duty);
extern int g_cnt;
extern unsigned int lMotorDuty;
extern unsigned int rMotorDuty;
extern int Wall_Follow_Distance_Control_Mode;
extern int Wall_Follow_Angle_Control_Mode;

int LCD_Mode_Parking_Go = 0;
int LCD_Mode_Parking_Done = 0;
int LCD_Mode_Exit_Go = 0;
int LCD_Mode_Emergency_Stop = 0;
int LCD_Mode_Parking_Back = 0;
int LCD_Mode_Exit_Done = 0;
int LCD_Mode_Emergency_Stop_F = 0;

char str[100];
uint32 data = 0;
int Init_Clear = 0;
int emergency_stop;

double Left_Motor_Pos_Trajectory=0;
double Left_Motor_Vel_Trajectory=0;

double Right_Motor_Pos_Trajectory=0;
double Right_Motor_Vel_Trajectory=0;

double Yaw = 0;
double Avg_Dist = 0;
int Wall_Follow_Mode = 1;
int Wall_Motion_Mode = 0;
int step=0;
int Wall_Motion_Step = 0;
int STATE = 0;
int Emergency_Stop;

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

TestCnt stTestCnt;
Motor_Speed App_Motor_Speed;
Decision_Speed App_Decision_Speed;
Wall_Vel App_Wall_Vel;

static void AppNoTask(void)
{
    //Read Encoder
    //Read Blutooth
    readEncoderTick();
    readBlutooth();
}

static void AppTask1ms(void)
{
    /*A function that performs a 1ms cycle of work*/

    // 1ms counter increment
    stTestCnt.u32nuCnt1ms++;
    // Calling the Angle Conversion Function
    Conv_rad_per_sec();


    if(Wall_Follow_Mode)// When in Wall Follow mode
    {
        // Left&Right motor speed PID control
        LeftMotor_Vel_PID_Controller(App_Motor_Speed.Left_Motor_Speed, Left_LPF_Encoder.LPF_rad_per_sec);
        RightMotor_Vel_PID_Controller(App_Motor_Speed.Right_Motor_Speed, Right_LPF_Encoder.LPF_rad_per_sec);

        // Left&Right Motor Duty Setting
        setLeftMotorDuty(100.0*(Left_Vel_PID.Vin)/12);
        setRightMotorDuty(100.0*(Right_Vel_PID.Vin)/12);

    }
    else if(Wall_Motion_Mode)// When in Wall Motion mode
    {
        // Left&Right motor position PID control
        LeftMotor_Vel_PID_Controller(Left_Pos_PID.Win, Left_LPF_Encoder.LPF_rad_per_sec);
        RightMotor_Vel_PID_Controller(Right_Pos_PID.Win, Right_LPF_Encoder.LPF_rad_per_sec);

        // Left&Right Motor Duty Setting
        setLeftMotorDuty(100.0*(Left_Vel_PID.Vin)/12);
        setRightMotorDuty(100.0*(Right_Vel_PID.Vin)/12);
    }

    // Now_Data_Debugging
    sprintf(str, "%d %d %d %d", g_Moveflag.exitMove, g_Moveflag.exitStop, g_Moveflag.parkingMove, g_Moveflag.parkingStop);

}
static void AppTask10ms(void)
{
    /*A function that performs a 10ms cycle of work*/

    // 10ms counter increment
    stTestCnt.u32nuCnt10ms++;

    /* TOF Sensor distance and angle calculation */
    Nlink_getTofDistance_Tof(4);
    Yaw = calculateYawAngle(Tof_dist_mm[0], Tof_dist_mm[1]);
    Avg_Dist = (Tof_dist_mm[0]+ Tof_dist_mm[1]+TOF_OFFSET)/2;



    // Mode-specific behavioral functions
    // MODE : Wall_Follow_Mode
    // MODE : Wall_Motion_Mode

    // Determining the driving state mode step
    if(STATE ==STATE_PARKING_WALL_FOLLOW)
    {
        // Wall_Follow_Mode Active
        Wall_Follow_Mode = 1;
    }
    else if(Avg_Dist < TOF_DETECT_DIS && Tof_dist_mm[0] != 0 &&Tof_dist_mm[1] != 0 )
    {
        // Two TOF sensor differences (when parking space is recognized)
        if(Tof_dist_mm[0]-Tof_dist_mm[1]>TOF_DIF)
        {
            // Wall_Motion_Mode Active
            // Wall_Follow_Mode Disable
            Wall_Follow_Mode = 0;
            Wall_Motion_Mode = 1;
        }
        else if(Wall_Motion_Mode ==1)
        {
            // Disable Wall_Follow_Mode during Wall_Motion_Mode activation
            Wall_Follow_Mode = 0;
        }
    }
    // Mode-specific behavioral functions
    // MODE : Wall_Follow_Mode
    // MODE : Wall_Motion_Mode

    if(Wall_Follow_Mode)// MODE : Wall_Follow_Mode
    {
        if(STATE == STATE_NORMAL_WALL_FOLLOW && !Emergency_Stop && g_Moveflag.parkingMove)
        {
            //Action along the parking space relocation wall
            //Checking the display through LCD: PARKING GO
            Normal_Wall_Follow();
            LcdDisplay_Parking_Go();
        }
        else if(STATE ==STATE_PARKING_WALL_FOLLOW && g_Moveflag.parkingMove)
        {
            //The action of parking along the walls of a parking space
            PARKING_WALL_FOLLOW();
        }
        else if(STATE ==STATE_EXIT_WALL_FOLLOW && g_Moveflag.exitMove)
        {
            if(Emergency_Stop)
            {
                //Emergency stop motor stop
                //Check display through LCD: EMERGENCY_STOP
                MotorStop();
                LcdDisplay_Emeregency_Stop();
            }
            else
            {
                //Attracting along the wall in the parking space
                //Checking the display through LCD: PARKING GO
                EXIT_WALL_FOLLOW();
                LcdDisplay_Exit_Go();
            }
        }
        //Speed output limit
        Speed_Limit();
    }
    else if(Wall_Motion_Mode)//Wall -> Parking Space, Parking Space -> Execution Mode of Designated Action for Departure
    {
        if(Wall_Motion_Step == PARKING_FORWARD) //PARKING_FORWARD
        {
            Motion_Parking_Forward();//moving forward
        }
        else if(Wall_Motion_Step == PARKING_CWROTATE)//PARKING_CWROTATE
        {
            Motion_CWRotate();//clockwise rotation
        }
        else if(Wall_Motion_Step == PARKING_BACKWARD) //PARKING_BACKWARD
        {
            Motion_Backward();//moving backwards
            LcdDisplay_Parking_Back();//Display check via LCD: PARKING BACK
        }
        else if(Wall_Motion_Step ==EXIT_CCWROTATE)//EXIT_CCWROTATE
        {
            Motion_CCWRotate();//Counterclockwise rotation
        }
        else if(Wall_Motion_Step == EXIT_DONE )
        {
            //Exit Complete
            //Run the signature sound when parking is complete
            //Display check via LCD: EXIT DONE
            //Wall_motion_step exit processing
            Start_Signature_Sound();
            LcdDisplay_Exit_Done();
            Wall_Motion_Step = FINISH;
        }

        if(STATE ==STATE_EXIT_WALL_FOLLOW)
        {
            //In case of departure
            //Initialize motion-related data
            //Execute the outgoing motion
            Wall_Step_Motion_Init();
            Motion_Exit_Forward();
        }


        //Position Control
        // Left&Right motor position PID control
        LeftMotor_Pos_PID_Controller(Left_Motor_Pos_Trajectory,Left_LPF_Encoder.LPF_Deg);
        RightMotor_Pos_PID_Controller(Right_Motor_Pos_Trajectory,Right_LPF_Encoder.LPF_Deg);

    }

}
static void AppTask50ms(void)
{
    /*A function that performs a 50ms cycle of work*/

    // 50ms counter increment
    stTestCnt.u32nuCnt50ms++;

    //In all situations, not backwards
    if(STATE != STATE_PARKING_WALL_FOLLOW)
    {
        //Front TOF sensor detected
        FrontDetection(Tof_dist_mm[2]);
        if(Tof_dist_mm[2] < FRONT_EMERGENCY_DIS && !Tof_dist_mm[2] == 0)
        {
            //Emergency stop when a certain distance from the front TOF sensor is reached
            //Check display via LCD: Emergency_STOP
            Emergency_Stop = 1;
            LcdDisplay_Emergency_Stop_F();
        }
        else
        {
            Emergency_Stop = 0;
        }
    }
    else if(STATE ==STATE_PARKING_WALL_FOLLOW)
    {
        // Backward situation
        // Rear TOF sensor detected affect park check
        BackDetection(Tof_dist_mm[3]);
    }

}
static void AppTask100ms(void)
{
    /*A function that performs a 100ms cycle of work*/

    // 100ms counter increment
    stTestCnt.u32nuCnt100ms++;

    /*UART Debugging*/
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
    /*A function that performs a 500ms cycle of work*/

    // 500ms counter increment
    stTestCnt.u32nuCnt500ms++;
}

static void AppTask1000ms(void)
{
    /*A function that performs a 1000ms cycle of work*/

    // 1000ms counter increment
    stTestCnt.u32nuCnt1000ms++;


}
void AppScheduling(void)/* Task schedule function */
{
    AppNoTask();

    if(stSchedulingInfo.u8nuScheduling1msFlag == 1u)
    {
        stSchedulingInfo.u8nuScheduling1msFlag = 0u;
        AppTask1ms();/* Run 1ms Task Schedule Function*/

        if(stSchedulingInfo.u8nuScheduling10msFlag == 1u)
        {
            stSchedulingInfo.u8nuScheduling10msFlag = 0u;
            AppTask10ms();/* Run 10ms Task Schedule Function*/
        }
        if(stSchedulingInfo.u8nuScheduling50msFlag == 1u)
        {
            stSchedulingInfo.u8nuScheduling50msFlag = 0u;
            AppTask50ms();/* Run 50ms Task Schedule Function*/
        }
        if(stSchedulingInfo.u8nuScheduling100msFlag == 1u)
        {
            stSchedulingInfo.u8nuScheduling100msFlag = 0u;
            AppTask100ms();/* Run 100ms Task Schedule Function*/
        }
        if(stSchedulingInfo.u8nuScheduling500msFlag == 1u)
        {
            stSchedulingInfo.u8nuScheduling500msFlag = 0u;
            AppTask500ms();/* Run 500ms Task Schedule Function*/
        }
        if(stSchedulingInfo.u8nuScheduling1000msFlag == 1u)
        {
            stSchedulingInfo.u8nuScheduling1000msFlag = 0u;
            AppTask1000ms();/* Run 1000ms Task Schedule Function*/
        }
    }
}

