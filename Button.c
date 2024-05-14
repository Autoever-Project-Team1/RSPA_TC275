#include "Button.h"
#include "IfxPort.h"
#include "Bsp.h"

ButtonState g_ButtonState;

//init gpio, by pullup
void Button_Init(void)
{
    IfxPort_setPinModeInput(BUTTON1, IfxPort_InputMode_pullUp);
    IfxPort_setPinModeInput(BUTTON2, IfxPort_InputMode_pullUp);
}

//read button state, polling
void getButtonState(void){
    g_ButtonState.parking   = !IfxPort_getPinState(BUTTON1);
    g_ButtonState.exit      = !IfxPort_getPinState(BUTTON2);
}


