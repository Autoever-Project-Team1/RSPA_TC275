#ifndef BLUETOOTH_H_
#define BLUETOOTH_H_

#include "Ifx_Types.h"

typedef struct{
        uint8 parkingMove;
        uint8 parkingStop;
        uint8 exitMove;
        uint8 exitStop;
}Moveflag;

extern Moveflag g_Moveflag;
extern uint8 g_firstBluConnect;

void readBlutooth(void);
void Bluetooth_init(void);

#endif /* BLUETOOTH_H_ */
