
#include "Ifx_Types.h"
#include "IfxCpu.h"
#include "IfxScuWdt.h"

#include "Driver_STM.h"
#include "AppScheduling.h"
#include "Bluetooth.h"
#include "Button.h"

IfxCpu_syncEvent g_cpuSyncEvent = 0;

void core0_main(void)
{
    IfxCpu_enableInterrupts();
    
    /* !!WATCHDOG0 AND SAFETY WATCHDOG ARE DISABLED HERE!!
     * Enable the watchdogs and service them periodically if it is required
     */
    IfxScuWdt_disableCpuWatchdog(IfxScuWdt_getCpuWatchdogPassword());
    IfxScuWdt_disableSafetyWatchdog(IfxScuWdt_getSafetyWatchdogPassword());
    
    /* Wait for CPU sync event */
    IfxCpu_emitEvent(&g_cpuSyncEvent);
    IfxCpu_waitEvent(&g_cpuSyncEvent, 1);

    Driver_Stm_Init();      //init stm, using AppScheduling
    Button_Init();          //init button 1, 2
    Bluetooth_Init();       //init uart0, using bluetooth

    IfxCpu_enableInterrupts();
        
    while(1)
    {
        AppScheduling();
    }
}
