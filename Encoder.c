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
App_LPF_Encoder g_LPF_Encoder;

App_Encoder g_Encoder2;
App_LPF_Encoder g_LPF_Encoder2;

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

    IfxPort_setPinModeInput(A2, IfxPort_InputMode_pullUp);
    IfxPort_setPinModeInput(B2, IfxPort_InputMode_pullUp);

}

void readEncoderTick(void)
{
    /*****A,B******/
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


    /*****A2,B2******/

    g_Encoder2.A_data = IfxPort_getPinState(A2);
    g_Encoder2.B_data = IfxPort_getPinState(B2);
    g_Encoder2.AB_data = (g_Encoder2.A_data << 1) | g_Encoder2.B_data;

    if(g_Encoder2.AB_data == 0){
        if(g_Encoder2.AB_bef == 1){
            g_Encoder2.AB_Cnt--;
        }
        if(g_Encoder2.AB_bef == 2){
            g_Encoder2.AB_Cnt++;
        }
    }
    else if(g_Encoder2.AB_data == 1){
        if(g_Encoder2.AB_bef == 3){
            g_Encoder2.AB_Cnt--;
        }
        if(g_Encoder2.AB_bef == 0){
            g_Encoder2.AB_Cnt++;
        }
    }
    else if(g_Encoder2.AB_data == 2){
        if(g_Encoder2.AB_bef == 0){
            g_Encoder2.AB_Cnt--;
        }
        if(g_Encoder2.AB_bef == 3){
            g_Encoder2.AB_Cnt++;
        }
    }
    else if(g_Encoder2.AB_data == 3){
        if(g_Encoder2.AB_bef == 2){
            g_Encoder2.AB_Cnt--;
        }
        if(g_Encoder2.AB_bef == 1){
            g_Encoder2.AB_Cnt++;
        }
    }

    g_Encoder2.AB_bef = g_Encoder2.AB_data;



}

void Conv_rad_per_sec(void)
{

    /*****A,B******/


    g_Encoder.diffCnt = g_Encoder.AB_Cnt - g_Encoder.AB_Cnt_bef;

    g_Encoder.rad_per_sec = (double)(g_Encoder.diffCnt)*2*PI / (SAMPLETIME * PPR * GEAR_RATIO);

    g_Encoder.AB_Cnt_bef = g_Encoder.AB_Cnt;

    g_LPF_Encoder.LPF_rad_per_sec = LPF(g_LPF_Encoder.LPF_rad_per_sec_bef,g_Encoder.rad_per_sec,100,0.001);
    g_LPF_Encoder.LPF_rad_per_sec_bef = g_LPF_Encoder.LPF_rad_per_sec;

    g_LPF_Encoder.LPF_Deg += g_LPF_Encoder.LPF_rad_per_sec * SAMPLETIME *360 / (2*PI);
    //if(g_LPF_Encoder.LPF_Deg>360)g_LPF_Encoder.LPF_Deg = 0;


    /*******A2,B2******/

    g_Encoder2.diffCnt = g_Encoder2.AB_Cnt - g_Encoder2.AB_Cnt_bef;

    g_Encoder2.rad_per_sec = (double)(g_Encoder2.diffCnt)*2*PI / (SAMPLETIME * PPR * GEAR_RATIO);

    g_Encoder2.AB_Cnt_bef = g_Encoder2.AB_Cnt;

    g_LPF_Encoder2.LPF_rad_per_sec = LPF(g_LPF_Encoder2.LPF_rad_per_sec_bef,g_Encoder2.rad_per_sec,100,0.001);
    g_LPF_Encoder2.LPF_rad_per_sec_bef = g_LPF_Encoder2.LPF_rad_per_sec;

    g_LPF_Encoder2.LPF_Deg += g_LPF_Encoder2.LPF_rad_per_sec * SAMPLETIME *360 / (2*PI);
    //if(g_LPF_Encoder.LPF_Deg>360)g_LPF_Encoder.LPF_Deg = 0;













}

double LPF(double previous_output, double input_signal, double cutoff_frequency, double Ts)
{
    //omega*2
    double LPF_Coef = cutoff_frequency * Ts;


    double output = LPF_Coef * input_signal + (1 - LPF_Coef) * previous_output;
    return output;
}
