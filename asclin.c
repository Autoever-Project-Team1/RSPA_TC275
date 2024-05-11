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
#define BLUETOOTH_BAUDRATE      115200

#define IFX_PRIORITY_ASCLIN_TX  53
#define IFX_PRIORITY_ASCLIN_RX  54
#define IFX_PRIORITY_ASCLIN_ER  55

#define IFX_PRIORITY_ASCLIN_TOF_TX  50
#define IFX_PRIORITY_ASCLIN_TOF_RX  51
#define IFX_PRIORITY_ASCLIN_TOF_ER  52

#define ISR_PRIORITY_ASCLIN_BLUETOOTH_TX  56
#define ISR_PRIORITY_ASCLIN_BLUETOOTH_RX  57
#define ISR_PRIORITY_ASCLIN_BLUETOOTH_ER  58



static IfxAsclin_Asc g_ascHandle0;
static IfxAsclin_Asc g_ascHandle1;
static IfxAsclin_Asc g_ascHandle2;
static IfxAsclin_Asc g_ascHandle3;
static IfxStdIf_DPipe g_ascStandardInterface;

uint8 g_uartTxBuffer[ASC_TX_BUFFER_SIZE + sizeof(Ifx_Fifo)+ 8];
uint8 g_uartRxBuffer[ASC_RX_BUFFER_SIZE + sizeof(Ifx_Fifo)+ 8];
uint8 g_uartTxBuffer_0[ASC_TX_BUFFER_SIZE + sizeof(Ifx_Fifo)+ 8];
uint8 g_uartRxBuffer_0[ASC_RX_BUFFER_SIZE + sizeof(Ifx_Fifo)+ 8];
uint8 g_uartTxBuffer_1[ASC_TX_BUFFER_SIZE + sizeof(Ifx_Fifo)+ 8];
uint8 g_uartRxBuffer_1[ASC_RX_BUFFER_SIZE + sizeof(Ifx_Fifo)+ 8];
uint8 g_uartTxBuffer_2[ASC_TX_BUFFER_SIZE + sizeof(Ifx_Fifo)+ 8];
uint8 g_uartRxBuffer_2[ASC_RX_BUFFER_SIZE + sizeof(Ifx_Fifo)+ 8];


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
//////


IFX_INTERRUPT(asclin2TxISR, 0, IFX_PRIORITY_ASCLIN_TOF_TX);
void asclin2TxISR(void)
{
    IfxAsclin_Asc_isrTransmit(&g_ascHandle2);
}

IFX_INTERRUPT(asclin2ErrISR, 0, IFX_PRIORITY_ASCLIN_TOF_ER);
void asclin2ErrISR(void)
{
    while(1);
}


void _init_uart2(void)
{
    IfxAsclin_Asc_Config ascConf;
    IfxAsclin_Asc_initModuleConfig(&ascConf, &MODULE_ASCLIN2);

    ascConf.baudrate.baudrate = TOF_BAUDRATE;
    ascConf.baudrate.oversampling = IfxAsclin_OversamplingFactor_16;

    ascConf.bitTiming.medianFilter = IfxAsclin_SamplesPerBit_three;
    ascConf.bitTiming.samplePointPosition = IfxAsclin_SamplePointPosition_8;

    ascConf.interrupt.txPriority = IFX_PRIORITY_ASCLIN_TOF_TX;
    ascConf.interrupt.rxPriority = IFX_PRIORITY_ASCLIN_TOF_RX;
    ascConf.interrupt.erPriority = IFX_PRIORITY_ASCLIN_TOF_ER;

    const IfxAsclin_Asc_Pins pins = {
            .cts        = NULL_PTR,
            .ctsMode    = IfxPort_InputMode_pullUp,
            .rx         = &IfxAsclin2_RXE_P33_8_IN,
            .rxMode     = IfxPort_InputMode_pullUp,
            .rts        = NULL_PTR,
            .tx         = &IfxAsclin2_TX_P33_9_OUT,
            .txMode     = IfxPort_OutputMode_pushPull,
            .pinDriver  = IfxPort_PadDriver_cmosAutomotiveSpeed1
    };
    ascConf.pins = &pins;

    ascConf.txBuffer = g_uartTxBuffer_2;
    ascConf.txBufferSize = ASC_TX_BUFFER_SIZE;
    ascConf.rxBuffer = g_uartRxBuffer_2;
    ascConf.rxBufferSize = ASC_RX_BUFFER_SIZE;

    IfxAsclin_Asc_initModule(&g_ascHandle2, &ascConf);
}

void _out_uart2(const unsigned char chr)
{
    IfxAsclin_Asc_blockingWrite(&g_ascHandle2, chr);
}

unsigned char _in_uart2(void)
{
    unsigned char ch;

    /* wait for a new character */
    while (_poll_uart2(&ch) == 0);

    return ch;
}

int _poll_uart2(unsigned char *chr)
{
    unsigned char ch;
    Ifx_SizeT count = 0;
    int res = 0;

    count = IfxAsclin_getRxFifoFillLevel(g_ascHandle2.asclin);
    if(count >= 1)
    {
        IfxAsclin_read8(g_ascHandle2.asclin, &ch, 1);
        *chr = ch;
        res = TRUE;
    }
    else
        res = FALSE;
    return res;
}


//

/*
//uart2..? tof??
IFX_INTERRUPT(asclin1TxISR, 0, IFX_PRIORITY_ASCLIN_TOF1_TX);
void asclin1TxISR(void)
{
    IfxAsclin_Asc_isrTransmit(&g_ascHandle1);
}

IFX_INTERRUPT(asclin1ErrISR, 0, IFX_PRIORITY_ASCLIN_TOF1_ER);
void asclin1ErrISR(void)
{
    while(1);
}

void _init_uart1(void)
{
    IfxAsclin_Asc_Config ascConf;
    IfxAsclin_Asc_initModuleConfig(&ascConf, &MODULE_ASCLIN1);

    ascConf.baudrate.baudrate = TOF_BAUDRATE;
    ascConf.baudrate.oversampling = IfxAsclin_OversamplingFactor_16;

    ascConf.bitTiming.medianFilter = IfxAsclin_SamplesPerBit_three;
    ascConf.bitTiming.samplePointPosition = IfxAsclin_SamplePointPosition_8;

    ascConf.interrupt.txPriority = IFX_PRIORITY_ASCLIN_TOF1_TX;
    ascConf.interrupt.rxPriority = IFX_PRIORITY_ASCLIN_TOF1_RX;
    ascConf.interrupt.erPriority = IFX_PRIORITY_ASCLIN_TOF1_ER;

    const IfxAsclin_Asc_Pins pins = {
            .cts        = NULL_PTR,
            .ctsMode    = IfxPort_InputMode_pullUp,
            .rx         = &IfxAsclin1_RXA_P15_1_IN,
            .rxMode     = IfxPort_InputMode_pullUp,
            .rts        = NULL_PTR,
            .tx         = &IfxAsclin1_TX_P15_0_OUT,
            .txMode     = IfxPort_OutputMode_pushPull,
            .pinDriver  = IfxPort_PadDriver_cmosAutomotiveSpeed1
    };
    ascConf.pins = &pins;

    ascConf.txBuffer = g_uartTxBuffer_1;
    ascConf.txBufferSize = ASC_TX_BUFFER_SIZE;
    ascConf.rxBuffer = g_uartRxBuffer_1;
    ascConf.rxBufferSize = ASC_RX_BUFFER_SIZE;

    IfxAsclin_Asc_initModule(&g_ascHandle1, &ascConf);
}

void _out_uart1(const unsigned char chr)
{
    IfxAsclin_Asc_blockingWrite(&g_ascHandle1, chr);
}

unsigned char _in_uart1(void)
{
    unsigned char ch;

    while (_poll_uart1(&ch) == 0);

    return ch;
}

int _poll_uart1(unsigned char *chr)
{
    unsigned char ch;
    Ifx_SizeT count = 0;
    int res = 0;

    count = IfxAsclin_getRxFifoFillLevel(g_ascHandle1.asclin);
    if(count >= 1)
    {
        IfxAsclin_read8(g_ascHandle1.asclin, &ch, 1);
        *chr = ch;
        res = TRUE;
    }
    else
        res = FALSE;
    return res;
}
*/

//UART
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

/* This function initializes the ASCLIN UART module */
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



