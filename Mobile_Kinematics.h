#ifndef MOBILE_KINEMATICS_H_
#define MOBILE_KINEMATICS_H_

#define RADIUS 1
#define DISTANCE 1

typedef struct
{

  double Left_Motor_Speed;
  double Right_Motor_Speed;

}Motor_Speed;

typedef struct
{
        double Reference_Vx;
        double Reference_Vy;
        double Reference_Wz;

}Decision_Speed;

typedef struct
{
    double Wall_Vx;
    double Wall_Vy;
    double Wall_Wz;
}Wall_Vel;

//Wall_Vel W_V;

Motor_Speed Kinematics(double Ref_Vx,double Ref_Vy, double Ref_Wz);
Decision_Speed decision(double angle);
Wall_Vel Wall_Follow(int Mode,double Tar_Dis,double Now_Dis,double Tar_Ang,double Now_Ang);


#endif /* MOBILE_KINEMATICS_H_ */
