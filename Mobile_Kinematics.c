#include "Mobile_Kinematics.h"



motorspeed kinematics(double Ref_Vx,double Ref_Vy, double Ref_Wz)
{
    motorspeed mycarspeed;

    mycarspeed.lmotorspeed = (2*Ref_Vx/RADIUS + 2*Ref_Vy/RADIUS + 2*DISTANCE*Ref_Wz/RADIUS)/1;
    mycarspeed.rmotorspeed = (2*Ref_Vx/RADIUS + 2*Ref_Vy/RADIUS - 2*DISTANCE*Ref_Wz/RADIUS)/1;

    return mycarspeed;

}

decision_speed decision(double angle)
{
    decision_speed  mycarspeed_decision;

    mycarspeed_decision.reference_vx = 3;

    if (angle > 10)
    {
        mycarspeed_decision.reference_wz = 1;
        mycarspeed_decision.reference_vy = 0;
    }
    else if (angle < -10)
    {
        mycarspeed_decision.reference_wz = -1;
        mycarspeed_decision.reference_vy = 0;
    }

    return mycarspeed_decision;

}
