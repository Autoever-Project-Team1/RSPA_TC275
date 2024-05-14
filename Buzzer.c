#include <stdlib.h>
#include <Ifx_reg.h>
#include <IfxPort.h>
#include <IfxGpt12.h>
#include "Buzzer.h"
#include "asclin.h"

#define Buzzer                      &MODULE_P02,3

#define ISR_PROVIDER_GPT12_TIMER    IfxSrc_Tos_cpu0
#define GPT1_BLOCK_PRESCALER        32
#define TIMER_T3_INPUT_PRESCALER    32
#define FREQUENCY     28000

int beepCnt = 0;
int toneCnt = 0;
int beepOnOff = 100;
int toneOnOff = 100;

int tone[] = {523,587,659,698,783,880,987,1064};
int H_sound[] = {415, 415, 440, 659, 1, 622, 740, 1108, 1};
int SongMode = 0;

IFX_INTERRUPT(IsrGpt120T3Handler_Beep, 0, ISR_PRIORITY_GPT1T3_TIMER);
void IsrGpt120T3Handler_Beep()
{
    //this.. song
    if(SongMode > 0){
        setToneCycle(H_sound[SongMode-1]);

        if(beepCnt % 5000 == 0){
            SongMode++;
        }

        if(SongMode == 10){
            SongMode = 0;
            setToneCycle(tone[7]);
        }
    }

    //this.. buzz...
    if ((beepCnt < beepOnOff) || (beepOnOff == 28) || (SongMode > 0)) {
        if(toneCnt >= toneOnOff){
            IfxPort_togglePin(Buzzer);
            toneCnt = 0;
        }
    } else if (beepCnt < beepOnOff * 2) {
        IfxPort_setPinLow(Buzzer);
    } else {
        beepCnt = 0;
    }
    beepCnt++;
    toneCnt++;
}

void Init_Buzzer()
{
    IfxPort_setPinModeOutput(Buzzer, IfxPort_OutputMode_pushPull, IfxPort_OutputIdx_general);
    init_gpt1();
    runGpt12_T3();

    setToneCycle(tone[7]);
    setBeepCycle(0);
}

void setBeepCycle(int cycle)
{
    beepOnOff = cycle*28;
}

void setToneCycle(int cycle)
{
    toneOnOff = FREQUENCY / cycle;
}

void start_signature_sound(void){
    beepCnt = 0;
    SongMode = 1;
}

void init_gpt1(void)
{
    /* Initialize the GPT12 module */
    IfxGpt12_enableModule(&MODULE_GPT120); /* Enable the GPT12 module */

    /* Initialize the Timer T3 (PWM) */
    IfxGpt12_setGpt1BlockPrescaler(&MODULE_GPT120, IfxGpt12_Gpt1BlockPrescaler_32); /* Set GPT1 block prescaler: 32 */
    IfxGpt12_T3_setMode(&MODULE_GPT120, IfxGpt12_Mode_timer);                       /* Set T3 to timer mode */
    IfxGpt12_T3_setTimerDirection(&MODULE_GPT120, IfxGpt12_TimerDirection_down);    /* Set T3 count direction(down) */
    //IfxGpt12_TimerInputPrescaler_32
    IfxGpt12_T3_setTimerPrescaler(&MODULE_GPT120, IfxGpt12_TimerInputPrescaler_1); /* Set T3 input prescaler(2^5=32) */

    /* Calculate dutyUpTime and dutyDownTime for reloading timer T3 */
    IfxGpt12_T3_setTimerValue(&MODULE_GPT120, 100);       /* Set timer T3 value */

    /* Timer T2: reloads the value DutyDownTime in timer T3 */
    IfxGpt12_T2_setMode(&MODULE_GPT120, IfxGpt12_Mode_reload);                                  /* Set the timer T2 in reload mode */
    IfxGpt12_T2_setReloadInputMode(&MODULE_GPT120, IfxGpt12_ReloadInputMode_bothEdgesTxOTL);    /* Reload Input Mode : Rising/Falling Edge T3OTL */
    IfxGpt12_T2_setTimerValue(&MODULE_GPT120, 100);

    /* Initialize the interrupt */
    volatile Ifx_SRC_SRCR *src = IfxGpt12_T3_getSrc(&MODULE_GPT120);                /* Get the interrupt address    */
    IfxSrc_init(src, ISR_PROVIDER_GPT12_TIMER, ISR_PRIORITY_GPT1T3_TIMER);          /* Initialize service request   */
    IfxSrc_enable(src);                                                             /* Enable GPT12 interrupt       */

    /* Initialize the Timer T4 for Ultrasonic */
    IfxGpt12_T4_setMode(&MODULE_GPT120, IfxGpt12_Mode_timer);
    IfxGpt12_T4_setTimerDirection(&MODULE_GPT120, IfxGpt12_TimerDirection_up);
    IfxGpt12_T4_setTimerPrescaler(&MODULE_GPT120, IfxGpt12_TimerInputPrescaler_32);
    IfxGpt12_T4_setTimerValue(&MODULE_GPT120, 0u);
}

void runGpt12_T3()
{
    IfxGpt12_T3_run(&MODULE_GPT120, IfxGpt12_TimerRun_start);
}
