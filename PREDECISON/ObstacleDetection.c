#include "ObstacleDetection.h"
#include "Buzzer.h"
#include "ToF.h"
#include "Driver_STM.h"
#include "AppScheduling.h"

#include "Ifx_Types.h"

Detectionflag g_Detectionfront;
Detectionflag g_DetectionBack;
char stopFlag_f = 0;
char stopFlag_b = 0;

void FrontDetection(int tof_data)//Create distance-specific sound
{

    if(tof_data < MIN_DETECT_MM_F){
        setBeepCycle(1);
        g_Detectionfront.Detection = 1;
        g_Detectionfront.Stop = 1;
        stopFlag_f++;

        if(stopFlag_f >= STOPTIME/RUNTIME){
            stopFlag_f = STOPTIME/RUNTIME;
        }
    }
    else if(tof_data < MIN_DETECT_MM_F + GAP_DETECT_MM){
        setBeepCycle(50);
        g_Detectionfront.Detection = 1;
        g_Detectionfront.Stop = 0;
        stopFlag_f = 0;
    }
    else if(tof_data < MIN_DETECT_MM_F + GAP_DETECT_MM*2){
        setBeepCycle(100);
        g_Detectionfront.Detection = 1;
        g_Detectionfront.Stop = 0;
        stopFlag_f = 0;
    }
    else if(tof_data < MIN_DETECT_MM_F + GAP_DETECT_MM*3){
        setBeepCycle(200);
        g_Detectionfront.Detection = 1;
        g_Detectionfront.Stop = 0;
        stopFlag_f = 0;
    }
    else{
        setBeepCycle(0);
        g_Detectionfront.Detection = 0;
        g_Detectionfront.Stop = 0;
        stopFlag_f = 0;
    }

    if(STOPTIME <= RUNTIME * stopFlag_f){
        setBeepCycle(0);
        g_Detectionfront.Stop = 1;
        g_Detectionfront.ActualStop = 1;
    }
}


void BackDetection(int tof_data)//Create distance-specific sound
{

    if(tof_data < MIN_DETECT_MM_B){
        setBeepCycle(1);
        g_DetectionBack.Detection = 1;
        g_DetectionBack.Stop = 1;
        stopFlag_b++;

        if(stopFlag_b >= STOPTIME/RUNTIME){
            stopFlag_b = STOPTIME/RUNTIME;
        }
    }
    else if(tof_data < MIN_DETECT_MM_B + GAP_DETECT_MM){
        setBeepCycle(50);
        g_DetectionBack.Detection = 2;
        g_DetectionBack.Stop = 0;
        stopFlag_b = 0;
    }
    else if(tof_data < MIN_DETECT_MM_B + GAP_DETECT_MM*2){
        setBeepCycle(100);
        g_DetectionBack.Detection = 3;
        g_DetectionBack.Stop = 0;
        stopFlag_b = 0;
    }
    else if(tof_data < MIN_DETECT_MM_B + GAP_DETECT_MM*3){
        setBeepCycle(200);
        g_DetectionBack.Detection = 4;
        g_DetectionBack.Stop = 0;
        stopFlag_b = 0;
    }
    else{
        setBeepCycle(0);
        g_DetectionBack.Detection = 0;
        g_DetectionBack.Stop = 0;
        stopFlag_b = 0;
    }

    if(STOPTIME <= RUNTIME * stopFlag_b){
        setBeepCycle(0);
        g_DetectionBack.Stop = 1;
        g_DetectionBack.ActualStop = 1;
    }
}
