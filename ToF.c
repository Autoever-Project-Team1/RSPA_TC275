#include "Asclin/Asc/IfxAsclin_Asc.h"
#include "Cpu/Irq/IfxCpu_Irq.h"
#include <stdio.h>
#include "IfxStdIf_DPipe.h"
#include "Asclin/Std/IfxAsclin.h"
#include "asclin.h"
#include "Ifx_Console.h"
#include "ToF.h"
#include "math.h"

#define ISR_PRIORITY_ASCLIN_TOF_TX  50
#define ISR_PRIORITY_ASCLIN_TOF_RX  51
#define ISR_PRIORITY_ASCLIN_TOF_ER  52

uint8 Tof_ID = 0;
static const unsigned char TOF_length = 16;

static unsigned int rxBufIdx = 0;
unsigned char gBuf_tof[16] = { 0 };

int Tof_dist_mm[10] = {0};


void Init_ToF()
{
    _init_uart2();
}

IFX_INTERRUPT(asclin2RxISR, 0, ISR_PRIORITY_ASCLIN_TOF_RX);
void asclin2RxISR(void)
{
    static unsigned char rxBuf[16] = { 0 };

    unsigned char c = (unsigned char) _in_uart2();
    //_out_uart3(c);

    rxBuf[rxBufIdx] = c;
    ++rxBufIdx;

    /* 버퍼가 꽉 차면, buf_tof에 복사 */
    if (rxBufIdx == TOF_length) {
        memcpy(gBuf_tof, rxBuf, TOF_length);
        rxBufIdx = 0;
    }
}

/* 수신 데이터가 정상이면 1, 그렇지 않으면 0 반환 */
static int verifyCheckSum (unsigned char data[])
{
    unsigned char checksum = 0;
    for (int i = 0; i < TOF_length-1; i++)
    {
        checksum += data[i];
    }
    if (data[0] == 0x57 && data[1] == 0x0 && data[2] == 0xFF)
    {
        return checksum == data[TOF_length-1];
    }
    else
    {
        return 0;
    }
}

/* 유효 거리인 경우 1 반환, 그렇지 않으면 0 반환 */
static int checkTofStrength (unsigned char data[])
{
    int TOF_distance = data[8] | (data[9] << 8) | (data[10] << 16);
    int TOF_signal_strength = data[12] | (data[13] << 8);
    /* when distance over 2m - out of range */
    if (TOF_signal_strength != 0 && TOF_distance != 16777215)
    {
        return 1;
    }
    else
    {
        return 0;
    }
}

/* Return Distance(mm) */
int getTofDistance_Tof()
{
    int TOF_distance = 0;
    unsigned char buf_ToF[TOF_length];

    /* copy buf_tof into tmp */
    memcpy(buf_ToF, gBuf_tof, TOF_length);

    if (!verifyCheckSum(buf_ToF))
    {
        return -1;
    }
    if (!checkTofStrength(buf_ToF))
    {
        return -2;
    }

    TOF_distance = buf_ToF[8] | (buf_ToF[9] << 8) | (buf_ToF[10] << 16);

    return TOF_distance;
}

void Nlink_Read_Tof(char id){
    uint8 TOF[8] = {0x57,0x10,0xff,0xff,0x00,0xff,0xff,0x63};
    TOF[4] = TOF[4] + id;
    TOF[7] = TOF[7] + id;

    char idx = 7;
    while(1)
    {
        _out_uart2(TOF[7 - idx]);
        if(idx == 0)break;
        idx--;
    }
}

uint8 Nlink_Read_Tof_ID(void){
    return gBuf_tof[3];
}

void Nlink_getTofDistance_Tof(uint8 n_Tof){
    Nlink_Read_Tof(Tof_ID);
    int dist = getTofDistance_Tof();
    //dist = dist/10;
    //error filter
    if(dist > 0)Tof_dist_mm[Nlink_Read_Tof_ID()] = dist;
    Tof_ID++;
    if(Tof_ID > n_Tof) Tof_ID = 0;
}

double calculateYawAngle(int tof1, int tof2) {

    double distance_difference = (double)(tof1 - tof2);
    double scaled_distance_diff = TOF_DISTDIFF * (distance_difference / (fabs(distance_difference) + TOF_DISTDIFF));

    // arcsin(y/x) 怨꾩궛?섏뿬 ?쇰뵒?덉쑝濡?諛섑솚
    double yaw_angle_radians = asin(scaled_distance_diff / TOF_DISTDIFF);
    yaw_angle_radians = yaw_angle_radians /PI * 180.0;
    return yaw_angle_radians;
}
