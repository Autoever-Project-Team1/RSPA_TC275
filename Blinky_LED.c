#include "IfxPort.h"
#include "Bsp.h"


#define LED         &MODULE_P10,2
#define motor2        &MODULE_P10,1
#define WAIT_TIME   500


void initLED(void)
{
    IfxPort_setPinModeOutput(LED, IfxPort_OutputMode_pushPull, IfxPort_OutputIdx_general);
    IfxPort_setPinModeOutput(motor2, IfxPort_OutputMode_pushPull, IfxPort_OutputIdx_general);
    IfxPort_setPinLow(LED);
    IfxPort_setPinLow(motor2);
}

void blinkLED(void)
{

    IfxPort_togglePin(LED);
    //for(int i = 0; i < 1000000; i++);
    //wait(IfxStm_getTicksFromMilliseconds(BSP_DEFAULT_TIMER, 500));
}

void LED_Off(void)
{
    IfxPort_setPinLow(LED);
}
