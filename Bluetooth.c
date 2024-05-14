#include "Bluetooth.h"
#include "Button.h"

#include "Asclin/Asc/IfxAsclin_Asc.h"
#include "Cpu/Irq/IfxCpu_Irq.h"
#include <stdio.h>
#include "IfxStdIf_DPipe.h"
#include "Asclin/Std/IfxAsclin.h"//

#define ASC_TX_BUFFER_SIZE      256
#define ASC_RX_BUFFER_SIZE      256

#define BLUETOOTH_BAUDRATE      115200

#define ISR_PRIORITY_ASCLIN_BLUETOOTH_TX  56
#define ISR_PRIORITY_ASCLIN_BLUETOOTH_RX  57
#define ISR_PRIORITY_ASCLIN_BLUETOOTH_ER  58

static IfxAsclin_Asc g_ascHandle0;

uint8 g_uartTxBuffer_0[ASC_TX_BUFFER_SIZE + sizeof(Ifx_Fifo)+ 8];
uint8 g_uartRxBuffer_0[ASC_RX_BUFFER_SIZE + sizeof(Ifx_Fifo)+ 8];


IFX_INTERRUPT(asclin0TxISR, 0, ISR_PRIORITY_ASCLIN_BLUETOOTH_TX);
void asclin0TxISR(void)
{
    IfxAsclin_Asc_isrTransmit(&g_ascHandle0);
}

IFX_INTERRUPT(asclin0RxISR, 0, ISR_PRIORITY_ASCLIN_BLUETOOTH_RX);
void asclin0RxISR(void)
{
    IfxAsclin_Asc_isrReceive(&g_ascHandle0);
}

IFX_INTERRUPT(asclin0ErrISR, 0, ISR_PRIORITY_ASCLIN_BLUETOOTH_ER);
void asclin0ErrISR(void)
{
    while(1);
}

void _init_uart0(void)
{
    IfxAsclin_Asc_Config ascConf;

    /* Set default configurations */
    IfxAsclin_Asc_initModuleConfig(&ascConf, &MODULE_ASCLIN0); /* Initialize the structure with default values      */

    /* Set the desired baud rate */
    ascConf.baudrate.baudrate = BLUETOOTH_BAUDRATE;                                   /* Set the baud rate in bit/s       */
    ascConf.baudrate.oversampling = IfxAsclin_OversamplingFactor_16;            /* Set the oversampling factor      */

    /* Configure the sampling mode */
    ascConf.bitTiming.medianFilter = IfxAsclin_SamplesPerBit_three;             /* Set the number of samples per bit*/
    ascConf.bitTiming.samplePointPosition = IfxAsclin_SamplePointPosition_8;    /* Set the first sample position    */

    /* ISR priorities and interrupt target */
    ascConf.interrupt.txPriority = ISR_PRIORITY_ASCLIN_BLUETOOTH_TX;  /* Set the interrupt priority for TX events             */
    ascConf.interrupt.rxPriority = ISR_PRIORITY_ASCLIN_BLUETOOTH_RX;  /* Set the interrupt priority for RX events             */
    ascConf.interrupt.erPriority = ISR_PRIORITY_ASCLIN_BLUETOOTH_ER;  /* Set the interrupt priority for Error events          */
    ascConf.interrupt.typeOfService = IfxSrc_Tos_cpu0;

    /* Pin configuration */
    const IfxAsclin_Asc_Pins pins = {
            .cts        = NULL_PTR,                         /* CTS pin not used                                     */
            .ctsMode    = IfxPort_InputMode_pullUp,
            .rx         = &IfxAsclin0_RXA_P14_1_IN ,        /* Select the pin for RX connected to the USB port      */
            .rxMode     = IfxPort_InputMode_pullUp,         /* RX pin                                               */
            .rts        = NULL_PTR,                         /* RTS pin not used                                     */
            .rtsMode    = IfxPort_OutputMode_pushPull,
            .tx         = &IfxAsclin0_TX_P14_0_OUT,         /* Select the pin for TX connected to the USB port      */
            .txMode     = IfxPort_OutputMode_pushPull,      /* TX pin                                               */
            .pinDriver  = IfxPort_PadDriver_cmosAutomotiveSpeed1
    };
    ascConf.pins = &pins;

    /* FIFO buffers configuration */
    ascConf.txBuffer = g_uartTxBuffer_0;                      /* Set the transmission buffer                          */
    ascConf.txBufferSize = ASC_TX_BUFFER_SIZE;              /* Set the transmission buffer size                     */
    ascConf.rxBuffer = g_uartRxBuffer_0;                      /* Set the receiving buffer                             */
    ascConf.rxBufferSize = ASC_RX_BUFFER_SIZE;              /* Set the receiving buffer size                        */

    /* Init ASCLIN module */
    IfxAsclin_Asc_initModule(&g_ascHandle0, &ascConf);          /* Initialize the module with the given configuration   */
}

/* Send character CHR via the serial line */
void _out_uart0(const unsigned char chr)
{
 //   while(IfxAsclin_Asc_canWriteCount(&g_ascHandle3, 1, TIME_INFINITE) != TRUE);
    IfxAsclin_Asc_blockingWrite(&g_ascHandle0, chr);
}

/* Receive (and wait for) a character from the serial line */
unsigned char _in_uart0(void)
{
    return IfxAsclin_Asc_blockingRead(&g_ascHandle0);
}

int _poll_uart0(unsigned char *chr)
{
    unsigned char ch;
    Ifx_SizeT count = 1;
    int res = 0;

    res = IfxAsclin_Asc_read(&g_ascHandle0, &ch, &count, TIME_INFINITE);
    if(res == TRUE)
    {
        *chr = ch;
    }

    return res;
}

int _nonBlock_poll_uart0(unsigned char *chr)
{
    unsigned char ch;
    Ifx_SizeT count = 1;
    int res = 0;

    res = IfxAsclin_Asc_read(&g_ascHandle0, &ch, &count, TIME_NULL);
    if (res == TRUE)
    {
        *chr = ch;
    }

    return res;
}

void Bluetooth_Init(void){
    _init_uart0();
}

//Convert the value read from gpio to text and send it
void SendBluetoothData_500ms_cycle(void){
    if(g_ButtonState.parking){
        _out_uart0('P');
    }
    else{
        _out_uart0('p');
    }

    if(g_ButtonState.exit){
        _out_uart0('E');
    }
    else{
        _out_uart0('e');
    }
}


