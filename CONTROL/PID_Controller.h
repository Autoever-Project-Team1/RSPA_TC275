#ifndef PID_CONTROLLER_H_
#define PID_CONTROLLER_H_

/*********************************************************************************************************************/
/*-----------------------------------------------------Includes------------------------------------------------------*/
/*********************************************************************************************************************/
#include "IfxPort.h"
#include "Bsp.h"
//LEFT
#define LEFT_MOTOR_DIR_PIN             &MODULE_P10,1
//RIGHT
#define RIGHT_MOTOR_DIR_PIN             &MODULE_P10,2

//dir2  = LEFT_MOTOR_DIR_PIN


/*********************************************************************************************************************/
/*------------------------------------------------------Macros-------------------------------------------------------*/
/*********************************************************************************************************************/

/*********************************************************************************************************************/
/*-------------------------------------------------Global variables--------------------------------------------------*/
/*********************************************************************************************************************/

/*********************************************************************************************************************/
/*-------------------------------------------------Data Structures---------------------------------------------------*/
/*********************************************************************************************************************/
typedef struct
{
      double error_w;
      double error_w_int;
      double error_w_int_old;
      double Kp;
      double Ki;
      double Vin;

}PID_Vel_Config;
typedef struct
{
      double error_p;
      double error_p_int;
      double error_p_int_old;
      double error_p_d;
      double error_p_d_old;
      double Kp;
      double Ki;
      double Kd;
      double Win;

}PID_Pos_Config;
/*********************************************************************************************************************/
/*--------------------------------------------Private Variables/Constants--------------------------------------------*/
/*********************************************************************************************************************/
extern PID_Vel_Config Right_Vel_PID;
extern PID_Pos_Config Right_Pos_PID;

extern PID_Vel_Config Left_Vel_PID;
extern PID_Pos_Config Left_Pos_PID;

extern PID_Pos_Config Wall_Follow_Distance_PID;
extern PID_Pos_Config Wall_Follow_Angle_PID;
/*********************************************************************************************************************/
/*------------------------------------------------Function Prototypes------------------------------------------------*/
/*********************************************************************************************************************/

void PID_Pos_Controller_Init(void);
void PID_Vel_Controller_Init(void);
void PID_App_Controller_Init(void);

void LeftMotor_Pos_PID_Controller(double Target,double Input);
void LeftMotor_Vel_PID_Controller(double Target,double Input);

void RightMotor_Pos_PID_Controller(double Target,double Input);
void RightMotor_Vel_PID_Controller(double Target,double Input);

void Wall_Follow_Distance_PID_Controller(double Target,double Input);
void Wall_Follow_Angle_PID_Controller(double Target,double Input);

#endif /* PID_CONTROLLER_H_ */
