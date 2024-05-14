#include "Mobile_Kinematics.h"

#include "PID_Controller.h"
#include "math.h"

extern int Wall_Follow_Distance_Control_Mode = 0;
extern int Wall_Follow_Angle_Control_Mode = 0;

int wallisfar = 0;
int nowallcnt =0;


Motor_Speed Kinematics(double Ref_Vx,double Ref_Vy, double Ref_Wz)
{
    Motor_Speed mycarspeed;

    //mycarspeed.Left_Motor_Speed = (2*Ref_Vx/RADIUS + 2*Ref_Vy/RADIUS + (2*DISTANCE*Ref_Wz)/RADIUS)/1;
    //mycarspeed.Right_Motor_Speed = (2*Ref_Vx/RADIUS + 2*Ref_Vy/RADIUS -2*DISTANCE*Ref_Wz/RADIUS)/1;


    mycarspeed.Left_Motor_Speed = 2*Ref_Vx +1*Ref_Wz*0.1;
    mycarspeed.Right_Motor_Speed = 2*Ref_Vx -1*Ref_Wz*0.1;

    return mycarspeed;



}
/*
Decision_Speed decision(double angle)
{
    Decision_Speed  mycarspeed_decision;

    mycarspeed_decision.Reference_Vx = 0.5;

    if (angle > 10)
    {
        mycarspeed_decision.Reference_Wz = 1;
        mycarspeed_decision.Reference_Vy = 0;
    }
    else if (angle < -10)
    {
        mycarspeed_decision.Reference_Wz = -1;
        mycarspeed_decision.Reference_Vy = 0;
    }

    return mycarspeed_decision;

}*/



Wall_Vel Wall_Follow(int Mode,double Tar_Dis,double Now_Dis,double Tar_Ang,double Now_Ang)
{
    Wall_Vel W_V;
    if(Mode == 1)
    {
        //StateMachine
        //20  10,30
        int Distance_Gap = 10; //3cm
        // 거리 및 각도 제어 모드 설정
        if (Now_Dis >= Tar_Dis - Distance_Gap && Now_Dis <= Tar_Dis + Distance_Gap)
        {
            // 거리가 10cm 내외이면 각도 제어 모드 활성화
            Wall_Follow_Distance_Control_Mode = 0;
            Wall_Follow_Angle_Control_Mode = 1;

        }
        else if (Now_Dis < Tar_Dis - Distance_Gap || Now_Dis > Tar_Dis + Distance_Gap)
        {
            // 거리가 10cm 이하이거나 30cm 이상이면 거리 제어 모드 활성화
            Wall_Follow_Distance_Control_Mode = 1;
            Wall_Follow_Angle_Control_Mode = 0;
        }

        //Wall
        //If long W_z change down
        //

        if(Wall_Follow_Distance_Control_Mode)
        {
            Wall_Follow_Distance_PID_Controller(Tar_Dis,Now_Dis);
            W_V.Wall_Vx = 1;

                W_V.Wall_Wz = Wall_Follow_Distance_PID.Win;

        }
        else if(Wall_Follow_Angle_Control_Mode)
        {
            Wall_Follow_Angle_PID_Controller(Tar_Ang,Now_Ang);
            W_V.Wall_Vx = 0.5;
            W_V.Wall_Wz = Wall_Follow_Angle_PID.Win;
        }
     }
        else//Not Wall Mode
        {

                W_V.Wall_Vx = 0;
                W_V.Wall_Vy = 0;
                W_V.Wall_Wz = 0;
        }

        return W_V;


}

