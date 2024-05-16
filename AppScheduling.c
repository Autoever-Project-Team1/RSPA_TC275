

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
#include "Buzzer.h"
#include "Bluetooth.h";
#include "ObstacleDetection.h"



#include <stdio.h>
#include <stdlib.h>
#include "math.h"

#include "GPT12.h"

//#include "lcd.h"
extern void setLeftMotorDuty(unsigned int duty);
extern void setRightMotorDuty(unsigned int duty);

int LCD_Mode_Parking_Go = 0;
int LCD_Mode_Parking_Done = 0;
int LCD_Mode_Exit_Go = 0;
int LCD_Mode_Emergency_Stop = 0;
int LCD_Mode_Parking_Back = 0;
int LCD_Mode_Exit_Done = 0;
int LCD_Mode_Emergency_Stop_F = 0;

char str[100];
uint32 data = 0;
int clear = 0;
int emergency_stop;

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

#define BREAK_A     &MODULE_P02,7
#define BREAK_B     &MODULE_P02,6


//MODE
#define W_F_MODE 1

double Avg_Dist = 0;

int wall_mode_flag = 1;
int wall_mode_count = 0;

int Motion_mode_flag = 0;

int step=0;
int wall_step = 0; // 0 follow //1 : back //2 : front



extern int Wall_Follow_Distance_Control_Mode;
extern int Wall_Follow_Angle_Control_Mode;

char aaa = 0;

//extern int Tof_dist_mm[10];

Motor_Speed App_Motor_Speed;
Decision_Speed App_Decision_Speed;
Wall_Vel App_Wall_Vel;

static void AppNoTask(void)
{
    readEncoderTick();
    readBlutooth();


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
    //sprintf(str, "ML %lf, MR %lf FrontDist%d BackDist%d Mode : %d,step: %d",App_Motor_Speed.Left_Motor_Speed,Right_Motor_Pos_Trajectory,Tof_dist_mm[2],Tof_dist_mm[3],Motion_mode_flag,step);
    //sprintf(str, "App_Wall_Vel.Wall_Vx : %.2lf, g_Moveflag.parkingMove: %d\n",App_Wall_Vel.Wall_Vx,g_Moveflag.parkingMove);
    sprintf(str, "%d %d %d %d", g_Moveflag.exitMove, g_Moveflag.exitStop, g_Moveflag.parkingMove, g_Moveflag.parkingStop);
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





//  wall_mode_flag


    if(wall_step ==1)
    {
        wall_mode_flag = 1;
    }
    else if(Avg_Dist < 500 && Tof_dist_mm[0] != 0 &&Tof_dist_mm[1] != 0 ) //wall AVG //INIT //CASE
    {


        if(Tof_dist_mm[0]-Tof_dist_mm[1]>50)
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



    if(wall_mode_flag)
    {
        if(wall_step == 0 && !emergency_stop && g_Moveflag.parkingMove)
        {
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





        App_Wall_Vel = Wall_Follow(wall_mode_flag,150,Avg_Dist,0,yaw);

        App_Motor_Speed = Kinematics(App_Wall_Vel.Wall_Vx,App_Wall_Vel.Wall_Vy,-1.0*App_Wall_Vel.Wall_Wz);
        //위치제어를 위한..
        Left_LPF_Encoder.LPF_Deg = 0;
        Right_LPF_Encoder.LPF_Deg = 0;
        stTestCnt.u32nuCnt1ms=0;
        }
        else if(wall_step ==1 && g_Moveflag.parkingMove)
        {

            App_Wall_Vel = Wall_Follow(wall_mode_flag,130,Avg_Dist,0,yaw);


            if(Tof_dist_mm[3] < 90 /*&& g_DetectionBack.ActualStop*/)///STOP
            {

                App_Motor_Speed.Left_Motor_Speed =0;
                App_Motor_Speed.Right_Motor_Speed =0;

                start_signature_sound();
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
                wall_step =2;

            }
            else if(Wall_Follow_Distance_Control_Mode)
            {
                App_Motor_Speed = Kinematics(-App_Wall_Vel.Wall_Vx*1.2,App_Wall_Vel.Wall_Vy,1.5*App_Wall_Vel.Wall_Wz);
            }
            else if(Wall_Follow_Angle_Control_Mode){
                App_Motor_Speed = Kinematics(-App_Wall_Vel.Wall_Vx,App_Wall_Vel.Wall_Vy,-1.0*App_Wall_Vel.Wall_Wz);
            }

            //App_Motor_Speed = Kinematics(App_Wall_Vel.Wall_Vx,App_Wall_Vel.Wall_Vy,-1.0*App_Wall_Vel.Wall_Wz);


        }
        else if(wall_step ==2 && g_Moveflag.exitMove)
        {

            if(emergency_stop)
            {
                /////
                App_Motor_Speed.Left_Motor_Speed = 0;
                App_Motor_Speed.Right_Motor_Speed = 0;
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
            else
            {
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
                App_Wall_Vel = Wall_Follow(wall_mode_flag,130,Avg_Dist,0,yaw);
                App_Motor_Speed = Kinematics(App_Wall_Vel.Wall_Vx,App_Wall_Vel.Wall_Vy,-1.0*App_Wall_Vel.Wall_Wz);

            }

        }
        else
        {
            App_Motor_Speed.Left_Motor_Speed = 0;
            App_Motor_Speed.Right_Motor_Speed = 0;

        }





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

            //RIGHT
            Right_Motor_Pos_Trajectory = MakeTrajectoryPos(-550,0,5,(double)stTestCnt.u32nuCnt1ms);
            //LEFT
            Left_Motor_Pos_Trajectory = MakeTrajectoryPos(-550,0,5,(double)stTestCnt.u32nuCnt1ms);


            if(Right_Motor_Pos_Trajectory == -550)
            {
                step =3;
                Motion_mode_flag = 0;
                wall_step =1;
                //
                Left_LPF_Encoder.LPF_Deg = 0;
                Left_Vel_PID.error_w_int = 0;
                //
                Right_LPF_Encoder.LPF_Deg = 0;
                Right_Vel_PID.error_w_int = 0;
                stTestCnt.u32nuCnt1ms=0;
            }


        }
        else if(step ==4)
        {
            //LEFT
           Left_Motor_Pos_Trajectory = MakeTrajectoryPos(-330,0,5,(double)stTestCnt.u32nuCnt1ms);
            //RIGHT
           Right_Motor_Pos_Trajectory = MakeTrajectoryPos(330,0,5,(double)stTestCnt.u32nuCnt1ms);

           if(Right_Motor_Pos_Trajectory ==330)
           {
               App_Motor_Speed.Left_Motor_Speed =0;
               App_Motor_Speed.Right_Motor_Speed =0;

               step = 5;
           }
        }
        else if(step == 5 ){
            start_signature_sound();

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


            step = 6;
        }

        if(wall_step ==2 )
        {

            if(!clear)
            {
                clear = 1;
                Left_Motor_Pos_Trajectory =0;
                Right_Motor_Pos_Trajectory = 0;
                //
                Left_LPF_Encoder.LPF_Deg = 0;
                Left_Vel_PID.error_w_int = 0;
                //
                Right_LPF_Encoder.LPF_Deg = 0;
                Right_Vel_PID.error_w_int = 0;
                stTestCnt.u32nuCnt1ms=0;

            }

            //RIGHT
           Right_Motor_Pos_Trajectory = MakeTrajectoryPos(300,0,5,(double)stTestCnt.u32nuCnt1ms);
            //LEFT
           Left_Motor_Pos_Trajectory = MakeTrajectoryPos(300,0,5,(double)stTestCnt.u32nuCnt1ms);

           if(Right_Motor_Pos_Trajectory == 300)
           {
               step =4;
               wall_step =3;
               //Motion_mode_flag = 1;
              //
              Left_LPF_Encoder.LPF_Deg = 0;
              Left_Vel_PID.error_w_int = 0;
              //
              Right_LPF_Encoder.LPF_Deg = 0;
              Right_Vel_PID.error_w_int = 0;
              stTestCnt.u32nuCnt1ms=0;

           }
        }




            //App_Motor_Speed.Left_Motor_Speed = 0;
            //App_Motor_Speed.Right_Motor_Speed = 0;


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
    //frontDetection(Tof_dist_mm[2]);


    if(wall_step != 1)
    {
        frontDetection(Tof_dist_mm[2]);

        if(Tof_dist_mm[2] < 70 && !Tof_dist_mm[2] == 0)
        {

            emergency_stop = 1;


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
        else
        {
            emergency_stop = 0;
        }
    }
    else if(wall_step ==1)
    {
        BackDetection(Tof_dist_mm[3]);

    }


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

