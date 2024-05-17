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

void readBlutooth(void){
    unsigned char c;
    int res;
    res = _nonBlock_poll_uart0(&c);


    //Press Button - One time
    if(res){
        if(g_firstBluConnect == 0){
            if(c == 'p' || c == 'P' || c == 'e' || c == 'E'){
                Start_Signature_Sound();
                g_firstBluConnect = 1;
            }
        }
        if(c == 'p'){
            g_Moveflag.parkingStop = 1;
            g_Moveflag.parkingMove = 0;
        }
        else if(c == 'P'){
            g_Moveflag.parkingStop = 0;
            g_Moveflag.parkingMove = 1;
        }
        else if(c == 'e'){
            g_Moveflag.exitStop = 1;
            g_Moveflag.exitMove = 0;
        }
        else if(c == 'E'){
            g_Moveflag.exitStop = 0;
            g_Moveflag.exitMove = 1;
        }

        //Press Button -ing
        //Comparing before and after
        if(g_Detectionfront.Stop || g_DetectionBack.Stop){

            if(c == 'p' || c == 'P'){
                g_Moveflag.parkingStop = 1;
                g_Moveflag.parkingMove = 0;
            }
            if(c == 'e' || c == 'E'){
                g_Moveflag.exitStop = 1;
                g_Moveflag.exitMove = 0;
            }
        }
        if(c == 'p' || c == 'P'){
            bef_p_state = c;
        }
        else if(c == 'e' || c == 'E'){
            bef_e_state = c;
        }
    }
}

void Bluetooth_init(void){
   _init_uart0();
}
