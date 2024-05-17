#ifndef BUTTON_H_
#define BUTTON_H_

#include "Ifx_Types.h"

#define BUTTON1      &MODULE_P33,5
#define BUTTON2      &MODULE_P11,12

 typedef struct
{
        uint8 parking;
        uint8 exit;

}ButtonState;

extern ButtonState g_ButtonState;

void Button_Init(void);
void getButtonState(void);

#endif /* BUTTON_H_ */
