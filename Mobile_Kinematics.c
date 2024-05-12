#include "Mobile_Kinematics.h"


Motor_Speed kinematics(double Ref_Vx,double Ref_Vy, double Ref_Wz)
{
    Motor_Speed mycarspeed;

    mycarspeed.Left_Motor_Speed = (2*Ref_Vx/RADIUS + 2*Ref_Vy/RADIUS + 2*DISTANCE*Ref_Wz/RADIUS)/1;
    mycarspeed.Right_Motor_Speed = (2*Ref_Vx/RADIUS + 2*Ref_Vy/RADIUS - 2*DISTANCE*Ref_Wz/RADIUS)/1;

    return mycarspeed;

}

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

}
