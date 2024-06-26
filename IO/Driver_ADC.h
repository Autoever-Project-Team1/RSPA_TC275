
#ifndef DRIVER_ADC_H_
#define DRIVER_ADC_H_

/*********************************************************************************************************************/
/*-----------------------------------------------------Includes------------------------------------------------------*/
/*********************************************************************************************************************/
#include "Ifx_Types.h"
/*********************************************************************************************************************/
/*------------------------------------------------------Macros-------------------------------------------------------*/
/*********************************************************************************************************************/

/*********************************************************************************************************************/
/*-------------------------------------------------Global variables--------------------------------------------------*/
/*********************************************************************************************************************/

/*********************************************************************************************************************/
/*-------------------------------------------------Data Structures---------------------------------------------------*/
/*********************************************************************************************************************/
 typedef enum
 {
     ADC_GROUP0_CH0 = 0u,
     ADC_GROUP0_CH1,
     ADC_GROUP0_CH2,
     ADC_GROUP0_CH3,
     ADC_GROUP0_CH4,
     ADC_GROUP0_CH5,
     ADC_GROUP0_CH6,
     ADC_GROUP0_CH7,
     ADC_GROUP0_MAX
 }ADC_GROUP0;

 typedef struct
 {
         uint32 sen1_Raw;
 }SensorAdcRaw;
 
 extern SensorAdcRaw stSensorAdcRaw;
/*********************************************************************************************************************/
/*--------------------------------------------Private Variables/Constants--------------------------------------------*/
/*********************************************************************************************************************/

/*********************************************************************************************************************/
/*------------------------------------------------Function Prototypes------------------------------------------------*/
/*********************************************************************************************************************/

 extern void DrvAdcInit(void);
 extern void DrvAdc_GetAdcRawGroup0(void);

#endif /* DRIVER_ADC_H_ */
