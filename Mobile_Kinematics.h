#ifndef MOBILE_KINEMATICS_H_
#define MOBILE_KINEMATICS_H_

#define RADIUS 1
#define DISTANCE 1

typedef struct
{
  double lmotorspeed;
  double rmotorspeed;
}motorspeed;

typedef struct
{
        double reference_vx;
        double reference_vy;
        double reference_wz;

}decision_speed;


motorspeed kinematics(double Ref_Vx,double Ref_Vy, double Ref_Wz);
decision_speed decision(double angle);



#endif /* MOBILE_KINEMATICS_H_ */
