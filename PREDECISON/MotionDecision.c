#include "MotionDecision.h"
/*IO*/
#include "Encoder.h"
#include "ToF.h"
/*DRIVER*/
/*CONTROL*/
#include "PID_Controller.h"
#include "Mobile_Kinematics.h"
#include "Trajectory.h"
/*DECISION*/
#include "AppScheduling.h"
/*PREDECISION*/

/*********************************************************************************************************************/
/*------------------------------------------------------Macros-------------------------------------------------------*/
/*********************************************************************************************************************/
#define STATE_NORMAL_WALL_FOLLOW 0
#define STATE_PARKING_WALL_FOLLOW 1
#define STATE_EXIT_WALL_FOLLOW 2
#define STATE_FINISH 3

#define PARKING_FORWARD 0
#define PARKING_CWROTATE 1
#define PARKING_BACKWARD 2
#define PARKING_ING 3
#define EXIT_CCWROTATE 4
#define EXIT_DONE 5
#define FINISH 6

#define KEEPDISTANCE 150
#define EMERGENCY_DIS 90
/*********************************************************************************************************************/
/*-------------------------------------------------Global variables--------------------------------------------------*/
/*********************************************************************************************************************/
extern double Left_Motor_Pos_Trajectory;
extern double Right_Motor_Pos_Trajectory;
extern int Wall_Motion_Step;
extern int STATE;
extern int Wall_Motion_Mode;
extern int Init_Clear;
extern int Wall_Follow_Distance_Control_Mode;
extern int Wall_Follow_Angle_Control_Mode;
extern int Wall_Follow_Mode;
extern double Yaw;
extern double Avg_Dist;
extern int Tof_dist_mm[10];

typedef struct
{

  double Left_Motor_Speed;
  double Right_Motor_Speed;

}Motor_Speed;

extern Motor_Speed App_Motor_Speed;
typedef struct
{
        uint32 u32nuCnt1ms;
        uint32 u32nuCnt10ms;
        uint32 u32nuCnt100ms;
        uint32 u32nuCnt50ms;
        uint32 u32nuCnt500ms;
        uint32 u32nuCnt1000ms;

}TestCnt;
extern TestCnt stTestCnt;

typedef struct
{
    double Wall_Vx;
    double Wall_Vy;
    double Wall_Wz;
}Wall_Vel;
extern Wall_Vel App_Wall_Vel;

/*********************************************************************************************************************/
/*------------------------------------------------Function Prototypes------------------------------------------------*/
/*********************************************************************************************************************/

void Speed_Limit()//Limit Speed
{
    if(App_Motor_Speed.Left_Motor_Speed>3) App_Motor_Speed.Left_Motor_Speed = 3;
    if(App_Motor_Speed.Right_Motor_Speed>3) App_Motor_Speed.Right_Motor_Speed = 3;

    if(App_Motor_Speed.Left_Motor_Speed<-3) App_Motor_Speed.Left_Motor_Speed = -3;
    if(App_Motor_Speed.Right_Motor_Speed<-3) App_Motor_Speed.Right_Motor_Speed = -3;
}
void Motion_Parking_Forward()//Motion_Parking_Forward : Make Trajectory & Set Next Step
{
    //RIGHT
    Right_Motor_Pos_Trajectory = MakeTrajectoryPos(360,0,5,(double)stTestCnt.u32nuCnt1ms);
    //LEFT
    Left_Motor_Pos_Trajectory = MakeTrajectoryPos(360,0,5,(double)stTestCnt.u32nuCnt1ms);

    if(Right_Motor_Pos_Trajectory == 360)
    {
        Wall_Motion_Step = PARKING_CWROTATE;
        Left_LPF_Encoder.LPF_Deg = 0;
        Right_LPF_Encoder.LPF_Deg = 0;
        stTestCnt.u32nuCnt1ms=0;
    }
}

void Motion_CWRotate()//Motion_CWRotate : Make Trajectory & Set Next Step
{

    //RIGHT
    Right_Motor_Pos_Trajectory = MakeTrajectoryPos(330,0,5,(double)stTestCnt.u32nuCnt1ms);
    //LEFT
    Left_Motor_Pos_Trajectory = MakeTrajectoryPos(-330,0,5,(double)stTestCnt.u32nuCnt1ms);

    if(Right_Motor_Pos_Trajectory == 330)
    {
        Wall_Motion_Step = PARKING_BACKWARD;
        Left_LPF_Encoder.LPF_Deg = 0;
        Right_LPF_Encoder.LPF_Deg = 0;
        stTestCnt.u32nuCnt1ms=0;
    }
}
void Motion_Backward()//Motion_Backward : Make Trajectory & Set Next Step
{
    //RIGHT
       Right_Motor_Pos_Trajectory = MakeTrajectoryPos(-550,0,5,(double)stTestCnt.u32nuCnt1ms);
       //LEFT
       Left_Motor_Pos_Trajectory = MakeTrajectoryPos(-550,0,5,(double)stTestCnt.u32nuCnt1ms);


       if(Right_Motor_Pos_Trajectory == -550)
       {
           Wall_Motion_Step =PARKING_ING;
           Wall_Motion_Mode = 0;
           STATE =STATE_PARKING_WALL_FOLLOW;
           //
           Left_LPF_Encoder.LPF_Deg = 0;
           Left_Vel_PID.error_w_int = 0;
           //
           Right_LPF_Encoder.LPF_Deg = 0;
           Right_Vel_PID.error_w_int = 0;
           stTestCnt.u32nuCnt1ms=0;
       }
}
void Motion_CCWRotate()//Motion_CCWRotate : Make Trajectory & Set Next Step
{
    //LEFT
    Left_Motor_Pos_Trajectory = MakeTrajectoryPos(-330,0,5,(double)stTestCnt.u32nuCnt1ms);
    //RIGHT
    Right_Motor_Pos_Trajectory = MakeTrajectoryPos(330,0,5,(double)stTestCnt.u32nuCnt1ms);

    if(Right_Motor_Pos_Trajectory ==330)
    {
         App_Motor_Speed.Left_Motor_Speed =0;
         App_Motor_Speed.Right_Motor_Speed =0;

         Wall_Motion_Step = EXIT_DONE;
    }
}
void Wall_Step_Motion_Init()//Wall_Step_Motion_Init : Init Control Data
{
    if(!Init_Clear)
    {
        Init_Clear = 1;
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
}
void Motion_Exit_Forward()//Motion_Exit_Forward : Make Trajectory & Set Next Step
{
    //LEFT
    Left_Motor_Pos_Trajectory = MakeTrajectoryPos(300,0,5,(double)stTestCnt.u32nuCnt1ms);
    //RIGHT
    Right_Motor_Pos_Trajectory = MakeTrajectoryPos(300,0,5,(double)stTestCnt.u32nuCnt1ms);

    if(Right_Motor_Pos_Trajectory == 300)
    {
        Wall_Motion_Step =EXIT_CCWROTATE;
        STATE =STATE_FINISH;

       Left_LPF_Encoder.LPF_Deg = 0;
       Left_Vel_PID.error_w_int = 0;
       //
       Right_LPF_Encoder.LPF_Deg = 0;
       Right_Vel_PID.error_w_int = 0;
       stTestCnt.u32nuCnt1ms=0;

    }
}
void MotorStop()//Motor Stop
{
    App_Motor_Speed.Left_Motor_Speed =0;
    App_Motor_Speed.Right_Motor_Speed =0;
}
void Normal_Wall_Follow()//Normal_Wall_Follow PID Keep a distance.
{
    App_Wall_Vel = Wall_Follow(Wall_Follow_Mode,KEEPDISTANCE,Avg_Dist,0,Yaw);
    App_Motor_Speed = Kinematics(App_Wall_Vel.Wall_Vx,App_Wall_Vel.Wall_Vy,-1.0*App_Wall_Vel.Wall_Wz);
    Left_LPF_Encoder.LPF_Deg = 0;
    Right_LPF_Encoder.LPF_Deg = 0;
    stTestCnt.u32nuCnt1ms=0;
}
void PARKING_WALL_FOLLOW()//PARKING_WALL_FOLLOW PID Keep a distance.
{
    App_Wall_Vel = Wall_Follow(Wall_Follow_Mode,KEEPDISTANCE,Avg_Dist,0,Yaw);
    if(Tof_dist_mm[3] < EMERGENCY_DIS)
    {
        MotorStop();
        Start_Signature_Sound();
        LcdDisplay_Parking_Done();
        STATE =STATE_EXIT_WALL_FOLLOW;
    }
    else if(Wall_Follow_Distance_Control_Mode)//Wall_Follow_Distance_Control_Mode : rotational component is dominant.
    {
        App_Motor_Speed = Kinematics(-App_Wall_Vel.Wall_Vx*1.2,App_Wall_Vel.Wall_Vy,1.5*App_Wall_Vel.Wall_Wz);
    }
    else if(Wall_Follow_Angle_Control_Mode)//Wall_Follow_Angle_Control_Mode : Straight component is dominant.
    {
        App_Motor_Speed = Kinematics(-App_Wall_Vel.Wall_Vx,App_Wall_Vel.Wall_Vy,-1.0*App_Wall_Vel.Wall_Wz);
    }
}
void EXIT_WALL_FOLLOW()//EXIT_WALL_FOLLOW PID Keep a distance.
{
    App_Wall_Vel = Wall_Follow(Wall_Follow_Mode,KEEPDISTANCE,Avg_Dist,0,Yaw);
    App_Motor_Speed = Kinematics(App_Wall_Vel.Wall_Vx,App_Wall_Vel.Wall_Vy,-1.0*App_Wall_Vel.Wall_Wz);
}
