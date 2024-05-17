
#include "Trajectory.h"

double Target=0;
double TargetV=0;
double count = 0;

double MakeTrajectoryPos(double theta_f,double theta_n, double t_f,double t_now)
{
    theta_n = 0;
    t_now = t_now * 0.001;

    Target = (-2 * (theta_f - theta_n) / (t_f * t_f * t_f))*(t_now*t_now*t_now)+
            (3 * (theta_f - theta_n) / (t_f * t_f)) * (t_now * t_now) +theta_n;

    if(t_now>=t_f)Target = theta_f;

    return Target;

}
double MakeTrajectoryVel(double theta_f,double theta_n, double t_f,double t_now)
{
    theta_n = 0;
    t_now = t_now * 0.001;

    TargetV = (-6 * (theta_f - theta_n) / (t_f * t_f * t_f))*(t_now*t_now)+
            (6 * (theta_f - theta_n) / (t_f * t_f)) * (t_now );

    return TargetV;

}

