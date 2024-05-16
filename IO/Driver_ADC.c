
/*********************************************************************************************************************/
/*-----------------------------------------------------Includes------------------------------------------------------*/
/*********************************************************************************************************************/
#include "Driver_ADC.h"
#include "IfxVadc.h"
#include "IfxVadc_Adc.h"
/*********************************************************************************************************************/
/*------------------------------------------------------Macros-------------------------------------------------------*/
/*********************************************************************************************************************/
typedef struct
{
        IfxVadc_Adc vadc;
        IfxVadc_Adc_Group adcGroup;
}App_VadcAutoScan;
/*********************************************************************************************************************/
/*-------------------------------------------------Global variables--------------------------------------------------*/
/*********************************************************************************************************************/

/*********************************************************************************************************************/
/*--------------------------------------------Private Variables/Constants--------------------------------------------*/
/*********************************************************************************************************************/
static void DrvAdc_Group0Init(void);
static void DrvAdc_Group0ChannelInit(uint8 param_ChNum);

App_VadcAutoScan g_VadcAutoScan;
IfxVadc_Adc_Channel adc0Channel[ADC_GROUP0_MAX];
SensorAdcRaw stSensorAdcRaw;
/*********************************************************************************************************************/
/*------------------------------------------------Function Prototypes------------------------------------------------*/
/*********************************************************************************************************************/

/*********************************************************************************************************************/
/*---------------------------------------------Function Implementations----------------------------------------------*/
/*********************************************************************************************************************/
void DrvAdcInit(void)
{
    DrvAdc_Group0Init();
}

static void DrvAdc_Group0Init(void)
{
    IfxVadc_Adc_Config adcConfig;
    IfxVadc_Adc_initModuleConfig(&adcConfig, &MODULE_VADC);

    IfxVadc_Adc_initModule(&g_VadcAutoScan.vadc, &adcConfig);

    IfxVadc_Adc_GroupConfig adcGroupConfig;
    IfxVadc_Adc_initGroupConfig(&adcGroupConfig, &g_VadcAutoScan.vadc);

    //group4 & CH7 : POT
    adcGroupConfig.groupId = IfxVadc_GroupId_4;
    adcGroupConfig.master = adcGroupConfig.groupId;

    adcGroupConfig.arbiter.requestSlotScanEnabled = TRUE;
    adcGroupConfig.scanRequest.autoscanEnabled = TRUE;

    adcGroupConfig.scanRequest.triggerConfig.gatingMode = 1;

    IfxVadc_Adc_initGroup(&g_VadcAutoScan.adcGroup, &adcGroupConfig);

    //adc channel
    DrvAdc_Group0ChannelInit(ADC_GROUP0_CH7);

    IfxVadc_Adc_startScan(&g_VadcAutoScan.adcGroup);
}

static void DrvAdc_Group0ChannelInit(uint8 param_ChNum)
{
    IfxVadc_Adc_ChannelConfig adcChannelConfigInfo;
    uint32 ulTemp = ((uint32)1u << param_ChNum);

    IfxVadc_Adc_initChannelConfig(&adcChannelConfigInfo, &g_VadcAutoScan.adcGroup);

    adcChannelConfigInfo.channelId = (IfxVadc_ChannelId)(param_ChNum);
    adcChannelConfigInfo.resultRegister = (IfxVadc_ChannelResult)(param_ChNum);

    IfxVadc_Adc_initChannel(&adc0Channel[param_ChNum], &adcChannelConfigInfo);

    IfxVadc_Adc_setScan(&g_VadcAutoScan.adcGroup, ulTemp, ulTemp);
}

void DrvAdc_GetAdcRawGroup0(void)
{
    Ifx_VADC_RES conversionResult;

    conversionResult = IfxVadc_Adc_getResult(&adc0Channel[ADC_GROUP0_CH7]);
    stSensorAdcRaw.sen1_Raw = conversionResult.B.RESULT;

    IfxVadc_Adc_startScan(&g_VadcAutoScan.adcGroup);
}
