
/*********************************************************************************************************************/
/*-----------------------------------------------------Includes------------------------------------------------------*/
/*********************************************************************************************************************/
#include "PID_Controller.h"
/*********************************************************************************************************************/
/*------------------------------------------------------Macros-------------------------------------------------------*/
/*********************************************************************************************************************/

/*********************************************************************************************************************/
/*-------------------------------------------------Global variables--------------------------------------------------*/
/*********************************************************************************************************************/
PID_Vel_Config Right_Vel_PID;
PID_Pos_Config Right_Pos_PID;

PID_Vel_Config Left_Vel_PID;
PID_Pos_Config Left_Pos_PID;
/*********************************************************************************************************************/
/*--------------------------------------------Private Variables/Constants--------------------------------------------*/
/*********************************************************************************************************************/

/*********************************************************************************************************************/
/*------------------------------------------------Function Prototypes------------------------------------------------*/
/*********************************************************************************************************************/

/*********************************************************************************************************************/
/*---------------------------------------------Function Implementations----------------------------------------------*/
/*********************************************************************************************************************/
void PID_Pos_Controller_Init(void)
{
    //
    Left_Pos_PID.Kp = 0.3;
    Left_Pos_PID.Ki = 0.0;
    Left_Pos_PID.Kd = 0.0;
    Left_Pos_PID.error_p_d = 0;
    //
    Right_Pos_PID.Kp = 1;
    Right_Pos_PID.Ki = 0.0;
    Right_Pos_PID.Kd = 0.5;
    Right_Pos_PID.error_p_d = 0;

}
void PID_Vel_Controller_Init(void)
{
    //
    Left_Vel_PID.Kp = 1.8;
    Left_Vel_PID.Ki = 1.2;
    //
    Right_Vel_PID.Kp = 2.2;
    Right_Vel_PID.Ki = 1.5;

}

void LeftMotor_Pos_PID_Controller(double Target, double Input) {
    Left_Pos_PID.error_p = Target - Input;

    Left_Pos_PID.error_p_d = Left_Pos_PID.error_p - Left_Pos_PID.error_p_d_old;
    Left_Pos_PID.error_p_d_old = Left_Pos_PID.error_p_d;

    Left_Pos_PID.error_p_int = Left_Pos_PID.error_p_int_old + Left_Pos_PID.error_p * 0.001;
    Left_Pos_PID.error_p_int_old = Left_Pos_PID.error_p_int;

    if (Left_Pos_PID.error_p_int > 10) Left_Pos_PID.error_p_int = 10;

    Left_Pos_PID.Win = Left_Pos_PID.Kp * Left_Pos_PID.error_p +
            Left_Pos_PID.Ki * Left_Pos_PID.error_p_int;

    if (Left_Pos_PID.Win > 654) {
        Left_Pos_PID.Win = 654;
    } else if (Left_Pos_PID.Win < -654) {
        Left_Pos_PID.Win = -654;
    }
}

void LeftMotor_Vel_PID_Controller(double Target, double Input) {
    Left_Vel_PID.error_w = Target - Input;
    Left_Vel_PID.error_w_int = Left_Vel_PID.error_w_int_old + Left_Vel_PID.error_w * 0.001;
    Left_Vel_PID.error_w_int_old = Left_Vel_PID.error_w_int;

    if (Left_Vel_PID.error_w_int > 10) Left_Vel_PID.error_w_int = 10;

    Left_Vel_PID.Vin = Left_Vel_PID.Kp * Left_Vel_PID.error_w +
            Left_Vel_PID.Ki * Left_Vel_PID.error_w_int;

    if (Left_Vel_PID.Vin > 11) {
        Left_Vel_PID.Vin = 11;
    } else if (Left_Vel_PID.Vin < -11) {
        Left_Vel_PID.Vin = -11;
    }


    if (Left_Vel_PID.Vin >= 0) IfxPort_setPinLow(LEFT_MOTOR_DIR_PIN);
    else if (Left_Vel_PID.Vin < 0) {
        IfxPort_setPinHigh(LEFT_MOTOR_DIR_PIN);
        Left_Vel_PID.Vin = Left_Vel_PID.Vin * -1;
    }
}


void RightMotor_Pos_PID_Controller(double Target,double Input)
{
    Right_Pos_PID.error_p = Target - Input;

    Right_Pos_PID.error_p_d =  Right_Pos_PID.error_p - Right_Pos_PID.error_p_d_old;
    Right_Pos_PID.error_p_d_old = Right_Pos_PID.error_p;


    Right_Pos_PID.error_p_int = Right_Pos_PID.error_p_int_old + Right_Pos_PID.error_p * 0.001;
    Right_Pos_PID.error_p_int_old = Right_Pos_PID.error_p_int;

    if (Right_Pos_PID.error_p_int > 10) Right_Pos_PID.error_p_int = 10;

    Right_Pos_PID.Win =
            Right_Pos_PID.Kp * Right_Pos_PID.error_p +
            Right_Pos_PID.Ki * Right_Pos_PID.error_p_int +
            Right_Pos_PID.Kd *Right_Pos_PID.error_p_d;

    if (Right_Pos_PID.Win > 12) {
        Right_Pos_PID.Win = 12;
    } else if (Right_Pos_PID.Win < -12) {
        Right_Pos_PID.Win = -12;
    }

}

void RightMotor_Vel_PID_Controller(double Target,double Input)
{
    Right_Vel_PID.error_w = Target - Input;
    Right_Vel_PID.error_w_int = Right_Vel_PID.error_w_int_old + Right_Vel_PID.error_w*0.001;
    Right_Vel_PID.error_w_int_old = Right_Vel_PID.error_w_int;

    if(Right_Vel_PID.error_w_int > 10) Right_Vel_PID.error_w_int = 10;

    Right_Vel_PID.Vin = Right_Vel_PID.Kp*Right_Vel_PID.error_w+
            Right_Vel_PID.Ki*Right_Vel_PID.error_w_int;

              if(Right_Vel_PID.Vin>11)
              {
                  Right_Vel_PID.Vin= 11;
              }
              else if(Right_Vel_PID.Vin<-11)
              {
                  Right_Vel_PID.Vin =-11;
              }

              if(Right_Vel_PID.Vin>=0)IfxPort_setPinHigh(RIGHT_MOTOR_DIR_PIN);
              else if(Right_Vel_PID.Vin<0)
                  {
                      IfxPort_setPinLow(RIGHT_MOTOR_DIR_PIN);
                      Right_Vel_PID.Vin =Right_Vel_PID.Vin*-1;
                  }

}


