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
App_Encoder Left_Encoder;
App_LPF_Encoder Left_LPF_Encoder;

App_Encoder Right_Encoder;
App_LPF_Encoder Right_LPF_Encoder;

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
    //LEFT
    IfxPort_setPinModeInput(A, IfxPort_InputMode_pullUp);
    IfxPort_setPinModeInput(B, IfxPort_InputMode_pullUp);
    //RIGHT
    IfxPort_setPinModeInput(A2, IfxPort_InputMode_pullUp);
    IfxPort_setPinModeInput(B2, IfxPort_InputMode_pullUp);

}

void readEncoderTick(void)
{
    /*****A,B******/
    Left_Encoder.A_data = IfxPort_getPinState(A);
    Left_Encoder.B_data = IfxPort_getPinState(B);
    Left_Encoder.AB_data = (Left_Encoder.A_data << 1) | Left_Encoder.B_data;

    if(Left_Encoder.AB_data == 0){
        if(Left_Encoder.AB_bef == 1){
            Left_Encoder.AB_Cnt--;
        }
        if(Left_Encoder.AB_bef == 2){
            Left_Encoder.AB_Cnt++;
        }
    }
    else if(Left_Encoder.AB_data == 1){
        if(Left_Encoder.AB_bef == 3){
            Left_Encoder.AB_Cnt--;
        }
        if(Left_Encoder.AB_bef == 0){
            Left_Encoder.AB_Cnt++;
        }
    }
    else if(Left_Encoder.AB_data == 2){
        if(Left_Encoder.AB_bef == 0){
            Left_Encoder.AB_Cnt--;
        }
        if(Left_Encoder.AB_bef == 3){
            Left_Encoder.AB_Cnt++;
        }
    }
    else if(Left_Encoder.AB_data == 3){
        if(Left_Encoder.AB_bef == 2){
            Left_Encoder.AB_Cnt--;
        }
        if(Left_Encoder.AB_bef == 1){
            Left_Encoder.AB_Cnt++;
        }
    }

    Left_Encoder.AB_bef = Left_Encoder.AB_data;


    /*****A2,B2******/

    Right_Encoder.A_data = IfxPort_getPinState(A2);
    Right_Encoder.B_data = IfxPort_getPinState(B2);
    Right_Encoder.AB_data = (Right_Encoder.A_data << 1) | Right_Encoder.B_data;

    if(Right_Encoder.AB_data == 0){
        if(Right_Encoder.AB_bef == 1){
            Right_Encoder.AB_Cnt--;
        }
        if(Right_Encoder.AB_bef == 2){
            Right_Encoder.AB_Cnt++;
        }
    }
    else if(Right_Encoder.AB_data == 1){
        if(Right_Encoder.AB_bef == 3){
            Right_Encoder.AB_Cnt--;
        }
        if(Right_Encoder.AB_bef == 0){
            Right_Encoder.AB_Cnt++;
        }
    }
    else if(Right_Encoder.AB_data == 2){
        if(Right_Encoder.AB_bef == 0){
            Right_Encoder.AB_Cnt--;
        }
        if(Right_Encoder.AB_bef == 3){
            Right_Encoder.AB_Cnt++;
        }
    }
    else if(Right_Encoder.AB_data == 3){
        if(Right_Encoder.AB_bef == 2){
            Right_Encoder.AB_Cnt--;
        }
        if(Right_Encoder.AB_bef == 1){
            Right_Encoder.AB_Cnt++;
        }
    }

    Right_Encoder.AB_bef = Right_Encoder.AB_data;



}

void Conv_rad_per_sec(void)
{

    /*****A,B******/


    Left_Encoder.diffCnt = Left_Encoder.AB_Cnt - Left_Encoder.AB_Cnt_bef;

    Left_Encoder.rad_per_sec = (double)(Left_Encoder.diffCnt)*2*PI / (SAMPLETIME * PPR * GEAR_RATIO);

    Left_Encoder.AB_Cnt_bef = Left_Encoder.AB_Cnt;

    Left_LPF_Encoder.LPF_rad_per_sec = LPF(Left_LPF_Encoder.LPF_rad_per_sec_bef,Left_Encoder.rad_per_sec,100,0.001);
    Left_LPF_Encoder.LPF_rad_per_sec_bef = Left_LPF_Encoder.LPF_rad_per_sec;

    Left_LPF_Encoder.LPF_Deg += Left_LPF_Encoder.LPF_rad_per_sec * SAMPLETIME *360 / (2*PI);
    //if(g_LPF_Encoder.LPF_Deg>360)g_LPF_Encoder.LPF_Deg = 0;


    /*******A2,B2******/

    Right_Encoder.diffCnt = Right_Encoder.AB_Cnt - Right_Encoder.AB_Cnt_bef;

    Right_Encoder.rad_per_sec = (double)(Right_Encoder.diffCnt)*2*PI / (SAMPLETIME * PPR * GEAR_RATIO);

    Right_Encoder.AB_Cnt_bef = Right_Encoder.AB_Cnt;

    Right_LPF_Encoder.LPF_rad_per_sec = LPF(Right_LPF_Encoder.LPF_rad_per_sec_bef,Right_Encoder.rad_per_sec,100,0.001);
    Right_LPF_Encoder.LPF_rad_per_sec_bef = Right_LPF_Encoder.LPF_rad_per_sec;

    Right_LPF_Encoder.LPF_Deg += Right_LPF_Encoder.LPF_rad_per_sec * SAMPLETIME *360 / (2*PI);
    //if(g_LPF_Encoder.LPF_Deg>360)g_LPF_Encoder.LPF_Deg = 0;







}

double LPF(double previous_output, double input_signal, double cutoff_frequency, double Ts)
{
    //omega*2
    double LPF_Coef = cutoff_frequency * Ts;


    double output = LPF_Coef * input_signal + (1 - LPF_Coef) * previous_output;
    return output;
}
