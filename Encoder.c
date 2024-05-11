/*********************************************************************************************************************/
/*-----------------------------------------------------Includes------------------------------------------------------*/
/*********************************************************************************************************************/
#include "Encoder.h"

/*********************************************************************************************************************/
/*------------------------------------------------------Macros-------------------------------------------------------*/
/*********************************************************************************************************************/

/*********************************************************************************************************************/
/*-------------------------------------------------Global variables--------------------------------------------------*/
/*********************************************************************************************************************/
App_Encoder g_Encoder;
PI_Controller g_PI_Controller;
PI_Controller g_PI_Controller_ang;

/*********************************************************************************************************************/
/*--------------------------------------------Private Variables/Constants--------------------------------------------*/
/*********************************************************************************************************************/

/*********************************************************************************************************************/
/*------------------------------------------------Function Prototypes------------------------------------------------*/
/*********************************************************************************************************************/

/*********************************************************************************************************************/
/*---------------------------------------------Function Implementations----------------------------------------------*/
/*********************************************************************************************************************/
void Encoder_Init(void)
{
    IfxPort_setPinModeInput(A, IfxPort_InputMode_pullUp);
    IfxPort_setPinModeInput(B, IfxPort_InputMode_pullUp);

    //kp, ki
    PI_Init(&g_PI_Controller,0.1, 5.0, 50.0);

    //PI_Init(&g_PI_Controller_ang,1.0, 1.0, 50.0);
}

void readEncoderTick(void)
{
    g_Encoder.A_data = IfxPort_getPinState(A);
    g_Encoder.B_data = IfxPort_getPinState(B);
    g_Encoder.AB_data = (g_Encoder.A_data << 1) | g_Encoder.B_data;

    if(g_Encoder.AB_data == 0){
        if(g_Encoder.AB_bef == 1){
            g_Encoder.AB_Cnt--;
        }
        if(g_Encoder.AB_bef == 2){
            g_Encoder.AB_Cnt++;
        }
    }
    else if(g_Encoder.AB_data == 1){
        if(g_Encoder.AB_bef == 3){
            g_Encoder.AB_Cnt--;
        }
        if(g_Encoder.AB_bef == 0){
            g_Encoder.AB_Cnt++;
        }
    }
    else if(g_Encoder.AB_data == 2){
        if(g_Encoder.AB_bef == 0){
            g_Encoder.AB_Cnt--;
        }
        if(g_Encoder.AB_bef == 3){
            g_Encoder.AB_Cnt++;
        }
    }
    else if(g_Encoder.AB_data == 3){
        if(g_Encoder.AB_bef == 2){
            g_Encoder.AB_Cnt--;
        }
        if(g_Encoder.AB_bef == 1){
            g_Encoder.AB_Cnt++;
        }
    }

    g_Encoder.AB_bef = g_Encoder.AB_data;
}

void Conv_rad_per_sec(void)
{
    //rad/s
    g_Encoder.diffCnt = g_Encoder.AB_Cnt - g_Encoder.AB_Cnt_bef;
    g_Encoder.rad_per_sec = (double)(g_Encoder.diffCnt)*2*PI / (SAMPLETIME * PPR);

    //lpf
    g_Encoder.rad_per_sec_filter = low_pass_filter( g_Encoder.rad_per_sec, g_Encoder.rad_per_sec_old__filter);

    //set...
    g_Encoder.rad_per_sec_old = g_Encoder.rad_per_sec;
    g_Encoder.rad_per_sec_old__filter = g_Encoder.rad_per_sec_filter;
    g_Encoder.AB_Cnt_bef = g_Encoder.AB_Cnt;
}

double low_pass_filter(double input, double prev_output) {
    double ALPHA=(100)*SAMPLETIME;
    double output = ALPHA * input + ((double)1.0 - ALPHA) * (prev_output);
    return output;
}

void PI_Init(PI_Controller *pid, float Kp, float Ki, float setpoint) {
    pid->Kp = Kp;
    pid->Ki = Ki;
    pid->integral = 0.0f;
    pid->prev_error = 0.0f;
    pid->setpoint = setpoint;
}

double PI_Control(float measured_value, float dt) {
    double error = g_PI_Controller.setpoint - measured_value;

    // 적분 항 계산
    g_PI_Controller.integral += error * dt;

    //안티와인드업
    if(g_PI_Controller.integral > 10) g_PI_Controller.integral = 10;

    // PI 출력 계산
    double output = g_PI_Controller.Kp * error + g_PI_Controller.Ki * g_PI_Controller.integral;

    //세츄레이션
    if(output > 11) output = 11;
    else if(output < 0) output = 0;

    return output;
}

double PI_Control_ang(float measured_value, float dt) {
    double error = g_PI_Controller_ang.setpoint - measured_value;

    // 적분 항 계산
    g_PI_Controller_ang.integral += error * dt;

    //안티와인드업
    if(g_PI_Controller_ang.integral > 100) g_PI_Controller_ang.integral = 100;

    // PI 출력 계산
    double output = g_PI_Controller_ang.Kp * error + g_PI_Controller_ang.Ki * g_PI_Controller_ang.integral;

    //세츄레이션
    if(output > 650) output = 650;
    else if(output < -650) output = 650;

    return output;
}


void EncoderPulsesToAngle(int pulse_count) {
    double angle = ((double)pulse_count / PPR) * 2.0 * PI;
    g_Encoder.cur_angle = angle;
}

void Angle_To_rad(){
    double delta_angle = g_Encoder.cur_angle - g_Encoder.old_angle;

    if (delta_angle > PI) {
        delta_angle -= 2.0 * PI;
    } else if (delta_angle < -PI) {
        delta_angle += 2.0 * PI;
    }


    g_Encoder.rad_per_sec = delta_angle / SAMPLETIME;

    //lpf
    g_Encoder.rad_per_sec_filter = low_pass_filter( g_Encoder.rad_per_sec, g_Encoder.rad_per_sec_old__filter);

    //set...
    g_Encoder.rad_per_sec_old = g_Encoder.rad_per_sec;
    g_Encoder.rad_per_sec_old__filter = g_Encoder.rad_per_sec_filter;
    g_Encoder.old_angle = g_Encoder.cur_angle;
}
