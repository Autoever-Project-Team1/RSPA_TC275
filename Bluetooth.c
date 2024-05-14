#include "Bluetooth.h"
#include "Buzzer.h"
#include "ObstacleDetection.h"
#include "asclin.h"
#include "Ifx_Types.h"
#include "Asclin/Asc/IfxAsclin_Asc.h"

Moveflag g_Moveflag;
uint8 g_firstBluConnect;

#define IFX_PRIORITY_ASCLIN_TX  53
#define IFX_PRIORITY_ASCLIN_RX  54
#define IFX_PRIORITY_ASCLIN_ER  55

#define IFX_PRIORITY_ASCLIN_TOF_TX  50
#define IFX_PRIORITY_ASCLIN_TOF_RX  51
#define IFX_PRIORITY_ASCLIN_TOF_ER  52

#define ISR_PRIORITY_ASCLIN_BLUETOOTH_TX  56
#define ISR_PRIORITY_ASCLIN_BLUETOOTH_RX  57
#define ISR_PRIORITY_ASCLIN_BLUETOOTH_ER  58

char bef_p_state = 0;
char bef_e_state = 0;



void Bluetooth_init(void){
   _init_uart0();
}
