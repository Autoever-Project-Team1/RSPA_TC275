#include "gpt12.h"
#include <Ifx_reg.h>
#include <Ifx_Types.h>
#include <IfxGpt12.h>
#include <IfxPort.h>
#include <Motor.h>
#include "Ifx_Types.h"
#include "IfxGpt12.h"
#include "IfxPort.h"
#include "asclin.h"
#include "isr_priority.h"

#define ISR_PROVIDER_GPT12_TIMER    0       //IfxSrc_Tos_cpu0
#define GPT120_MODULE_FREQUENCY     100000000

#define PWM_A       &MODULE_P02,1
#define PWM_B       &MODULE_P10,3

extern unsigned int lMotorDuty = 50;
extern unsigned int rMotorDuty = 50;
static volatile unsigned int cnt_10us = 0;
static volatile unsigned int cntDelay = 0;

IFX_INTERRUPT(IsrGpt2T6Handler, 0, ISR_PRIORITY_GPT2T6_TIMER);


extern int g_cnt = 0;


void IsrGpt2T6Handler()
{
    if (cnt_10us  < lMotorDuty) {
        IfxPort_setPinHigh(PWM_A);  /*Left Motor (CH-A)*/
    } else {
        IfxPort_setPinLow(PWM_A);   /*Left Motor (CH-A)*/
    }

    if (cnt_10us < rMotorDuty) {
        IfxPort_setPinHigh(PWM_B);  /*Right Motor (CH-B)*/
    } else {
        IfxPort_setPinLow(PWM_B);   /*Right Motor (CH-B)*/
    }

    if (cnt_10us == 5)//20Khz
    {
        cnt_10us = 0;
    } else {
        cnt_10us++;
    }
    g_cnt = cnt_10us;
}

unsigned int getLeftMotorDuty()
{
    return lMotorDuty;
}

unsigned int getRightMotorDuty()
{
    return rMotorDuty;
}

void setLeftMotorDuty(unsigned int duty)
{
    lMotorDuty = duty;
}

void setRightMotorDuty(unsigned int duty)
{
    rMotorDuty = duty;
}

unsigned int getcntDelay()
{
    return cntDelay;
}

void setcntDelay(unsigned int n)
{
    cntDelay = n;
}

void init_gpt2(void)
{
    /* Initialize the GPT12 module */
    IfxGpt12_enableModule(&MODULE_GPT120); /* Enable the GPT12 module */ /* Enable the GPT12 module */

    /* Initialize the Timer T6 for delay_ms */
    IfxGpt12_setGpt2BlockPrescaler(&MODULE_GPT120, IfxGpt12_Gpt1BlockPrescaler_4);  /* Set GPT2 block prescaler: 4 */
    IfxGpt12_T6_setMode(&MODULE_GPT120, IfxGpt12_Mode_timer);                       /* Set T6 to timer mode */
    IfxGpt12_T6_setTimerDirection(&MODULE_GPT120, IfxGpt12_TimerDirection_down);    /* Set T6 count direction(down) */
    IfxGpt12_T6_setTimerPrescaler(&MODULE_GPT120, IfxGpt12_TimerInputPrescaler_1);  /* Set T6 input prescaler(2^0=1) */

    MODULE_GPT120.T6CON.B.T6OE = 0x1;                                               /* Overflow/Underflow Output Enable */
    IfxGpt12_T6_setReloadMode(&MODULE_GPT120, IfxGpt12_TimerReloadMode_enable);     /* Reload from register CAPREL Enabled */
    IfxGpt12_T6_setTimerValue(&MODULE_GPT120, 250u);                                 /* Set T6 start value (10us) */
    MODULE_GPT120.CAPREL.U = 250u;                                                  /* Set CAPREL reload value */                                 /* Set CAPREL reload value */

    /* Initialize the interrupt */
    volatile Ifx_SRC_SRCR *src = IfxGpt12_T6_getSrc(&MODULE_GPT120);                /* Get the interrupt address    */
    IfxSrc_init(src, ISR_PROVIDER_GPT12_TIMER, ISR_PRIORITY_GPT2T6_TIMER);          /* Initialize service request   */
    IfxSrc_enable(src);                                                             /* Enable GPT12 interrupt       */

    IfxGpt12_T6_run(&MODULE_GPT120, IfxGpt12_TimerRun_start);
}

void runGpt12_T6()
{
    IfxGpt12_T6_run(&MODULE_GPT120, IfxGpt12_TimerRun_start);
}

void stopGpt12_T6()
{
    IfxGpt12_T6_run(&MODULE_GPT120, IfxGpt12_TimerRun_stop);
}

void runGpt12_T4()
{
    IfxGpt12_T4_run(&MODULE_GPT120, IfxGpt12_TimerRun_start);
}

void stopGpt12_T4()
{
    IfxGpt12_T4_run(&MODULE_GPT120, IfxGpt12_TimerRun_stop);
}

void setGpt12_T4(unsigned short value)
{
    IfxGpt12_T4_setTimerValue(&MODULE_GPT120, value);
}

unsigned int getGpt12_T4()
{
    return IfxGpt12_T4_getTimerValue(&MODULE_GPT120);
}
