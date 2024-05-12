
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
PID_Vel_Config vel_pid;
PID_Pos_Config pos_pid;

PID_Vel_Config vel_pid2;
PID_Pos_Config pos_pid2;
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
    pos_pid.Kp = 1;
    pos_pid.Ki = 0.0;
    pos_pid.Kd = 0.5;
    pos_pid.error_p_d = 0;
    //
    pos_pid2.Kp = 0.3;
    pos_pid2.Ki = 0.0;
    pos_pid2.Kd = 0.0;
    pos_pid2.error_p_d = 0;
}
void PID_Vel_Controller_Init(void)
{
    //
    vel_pid.Kp = 2.2;
    vel_pid.Ki = 1.5;
    //
    vel_pid2.Kp = 1.8;
    vel_pid2.Ki = 1.2;
}
void PID_Pos_Con(double Target,double Input)
{
    pos_pid.error_p = Target - Input;

    pos_pid.error_p_d =  pos_pid.error_p - pos_pid.error_p_d_old;
    pos_pid.error_p_d_old = pos_pid.error_p;


    pos_pid.error_p_int = pos_pid.error_p_int_old + pos_pid.error_p * 0.001;
    pos_pid.error_p_int_old = pos_pid.error_p_int;

    if (pos_pid.error_p_int > 10) pos_pid.error_p_int = 10;

    pos_pid.Win = pos_pid.Kp * pos_pid.error_p + pos_pid.Ki * pos_pid.error_p_int + pos_pid.Kd *pos_pid.error_p_d;

    if (pos_pid.Win > 12) {
        pos_pid.Win = 12;
    } else if (pos_pid.Win < -12) {
        pos_pid.Win = -12;
    }

}

void PID_Vel_Con(double Target,double Input)
{
    vel_pid.error_w = Target - Input;
    vel_pid.error_w_int = vel_pid.error_w_int_old + vel_pid.error_w*0.001;
    vel_pid.error_w_int_old = vel_pid.error_w_int;

    if(vel_pid.error_w_int > 10) vel_pid.error_w_int = 10;

    vel_pid.Vin = vel_pid.Kp*vel_pid.error_w+vel_pid.Ki*vel_pid.error_w_int;

              if(vel_pid.Vin>11)
              {
                 vel_pid.Vin= 11;
              }
              else if(vel_pid.Vin<-11)
              {
                  vel_pid.Vin =-11;
              }

              if(vel_pid.Vin>=0)IfxPort_setPinHigh(DIR_PIN);
              else if(vel_pid.Vin<0)
                  {
                      IfxPort_setPinLow(DIR_PIN);
                      vel_pid.Vin =vel_pid.Vin*-1;
                  }

}

void PID_Pos_Con2(double Target, double Input) {
    pos_pid2.error_p = Target - Input;

    pos_pid2.error_p_d = pos_pid2.error_p - pos_pid2.error_p_d_old;
    pos_pid2.error_p_d_old = pos_pid2.error_p_d;

    pos_pid2.error_p_int = pos_pid2.error_p_int_old + pos_pid2.error_p * 0.001;
    pos_pid2.error_p_int_old = pos_pid2.error_p_int;

    if (pos_pid2.error_p_int > 10) pos_pid2.error_p_int = 10;

    pos_pid2.Win = pos_pid2.Kp * pos_pid2.error_p + pos_pid2.Ki * pos_pid2.error_p_int;

    if (pos_pid2.Win > 654) {
        pos_pid2.Win = 654;
    } else if (pos_pid2.Win < -654) {
        pos_pid2.Win = -654;
    }
}

void PID_Vel_Con2(double Target, double Input) {
    vel_pid2.error_w = Target - Input;
    vel_pid2.error_w_int = vel_pid2.error_w_int_old + vel_pid2.error_w * 0.001;
    vel_pid2.error_w_int_old = vel_pid2.error_w_int;

    if (vel_pid2.error_w_int > 10) vel_pid2.error_w_int = 10;

    vel_pid2.Vin = vel_pid2.Kp * vel_pid2.error_w + vel_pid2.Ki * vel_pid2.error_w_int;

    if (vel_pid2.Vin > 11) {
        vel_pid2.Vin = 11;
    } else if (vel_pid2.Vin < -11) {
        vel_pid2.Vin = -11;
    }


    if (vel_pid2.Vin >= 0) IfxPort_setPinLow(DIR_PIN2);
    else if (vel_pid2.Vin < 0) {
        IfxPort_setPinHigh(DIR_PIN2);
        vel_pid2.Vin = vel_pid2.Vin * -1;
    }
}
