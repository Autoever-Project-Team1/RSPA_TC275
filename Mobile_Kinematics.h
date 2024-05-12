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


Motor_Speed kinematics(double Ref_Vx,double Ref_Vy, double Ref_Wz);
Decision_Speed decision(double angle);



#endif /* MOBILE_KINEMATICS_H_ */
