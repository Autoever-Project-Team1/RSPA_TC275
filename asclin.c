#include "Asclin/Asc/IfxAsclin_Asc.h"
#include "Cpu/Irq/IfxCpu_Irq.h"
#include <stdio.h>
#include "IfxStdIf_DPipe.h"
#include "Asclin/Std/IfxAsclin.h"
#include "asclin.h"
#include "Ifx_Console.h"



#define ASC_TX_BUFFER_SIZE      256
#define ASC_RX_BUFFER_SIZE      256
#define ASC_BAUDRATE            115200
#define TOF_BAUDRATE            921600
#define BLUETOOTH_BAUDRATE      9600

#define IFX_PRIORITY_ASCLIN_TX  4
#define IFX_PRIORITY_ASCLIN_RX  3
#define IFX_PRIORITY_ASCLIN_ER  2


//static IfxAsclin_Asc g_ascHandle0;
//static IfxAsclin_Asc g_ascHandle1;
static IfxAsclin_Asc g_ascHandle3;
static IfxStdIf_DPipe g_ascStandardInterface;

uint8 g_uartTxBuffer[ASC_TX_BUFFER_SIZE + sizeof(Ifx_Fifo)+ 8];
uint8 g_uartRxBuffer[ASC_RX_BUFFER_SIZE + sizeof(Ifx_Fifo)+ 8];
uint8 g_uartTxBuffer_0[ASC_TX_BUFFER_SIZE + sizeof(Ifx_Fifo)+ 8];
uint8 g_uartRxBuffer_0[ASC_RX_BUFFER_SIZE + sizeof(Ifx_Fifo)+ 8];
uint8 g_uartTxBuffer_1[ASC_TX_BUFFER_SIZE + sizeof(Ifx_Fifo)+ 8];
uint8 g_uartRxBuffer_1[ASC_RX_BUFFER_SIZE + sizeof(Ifx_Fifo)+ 8];


IFX_INTERRUPT(asclin3TxISR, 0, IFX_PRIORITY_ASCLIN_TX);
void asclin3TxISR(void)
{
    IfxAsclin_Asc_isrTransmit(&g_ascHandle3);
}

IFX_INTERRUPT(asclin3RxISR, 0, IFX_PRIORITY_ASCLIN_RX);
void asclin3RxISR(void)
{
    IfxAsclin_Asc_isrReceive(&g_ascHandle3);
}

IFX_INTERRUPT(asclin3ErrISR, 0, IFX_PRIORITY_ASCLIN_ER);
void asclin3ErrISR(void)
{
    while(1);
}
void _init_uart3(void)
{
    IfxAsclin_Asc_Config ascConf;
    IfxAsclin_Asc_initModuleConfig(&ascConf, &MODULE_ASCLIN3);

    ascConf.baudrate.baudrate = ASC_BAUDRATE;
    ascConf.baudrate.oversampling = IfxAsclin_OversamplingFactor_16;

    ascConf.bitTiming.medianFilter = IfxAsclin_SamplesPerBit_three;
    ascConf.bitTiming.samplePointPosition = IfxAsclin_SamplePointPosition_8;

    ascConf.interrupt.txPriority = IFX_PRIORITY_ASCLIN_TX;
    ascConf.interrupt.rxPriority = IFX_PRIORITY_ASCLIN_RX;
    ascConf.interrupt.erPriority = IFX_PRIORITY_ASCLIN_ER;

    const IfxAsclin_Asc_Pins pins = {
            .cts        = NULL_PTR,
            .ctsMode    = IfxPort_InputMode_pullUp,
            .rx         = &IfxAsclin3_RXD_P32_2_IN,
            .rxMode     = IfxPort_InputMode_pullUp,
            .rts        = NULL_PTR,
            .tx         = &IfxAsclin3_TX_P15_7_OUT,
            .txMode     = IfxPort_OutputMode_pushPull,
            .pinDriver  = IfxPort_PadDriver_cmosAutomotiveSpeed1
    };
    ascConf.pins = &pins;

    ascConf.txBuffer = g_uartTxBuffer;
    ascConf.txBufferSize = ASC_TX_BUFFER_SIZE;
    ascConf.rxBuffer = g_uartRxBuffer;
    ascConf.rxBufferSize = ASC_RX_BUFFER_SIZE;

    IfxAsclin_Asc_initModule(&g_ascHandle3, &ascConf);

    IfxAsclin_Asc_stdIfDPipeInit(&g_ascStandardInterface, &g_ascHandle3);

    Ifx_Console Ifx_g_console;
    Ifx_g_console.standardIo = &g_ascStandardInterface;
    Ifx_g_console.align      = 0;
}

void _out_uart3(const unsigned char chr)
{
    IfxAsclin_Asc_blockingWrite(&g_ascHandle3, chr);
}

unsigned char _in_uart3(void)
{
    return IfxAsclin_Asc_blockingRead(&g_ascHandle3);
}
