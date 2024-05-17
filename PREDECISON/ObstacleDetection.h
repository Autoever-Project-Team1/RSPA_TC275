#ifndef OBSTACLEDETECTION_H_
#define OBSTACLEDETECTION_H_

#include "Ifx_Types.h"

#define GAP_DETECT_MM           50
#define MIN_DETECT_MM_F         70
#define MIN_DETECT_MM_B         90

#define RUNTIME               50 //ms
#define STOPTIME              2000


typedef struct{
        uint8 Detection;
        uint8 Stop;
        uint8 ActualStop;
}Detectionflag;


extern Detectionflag g_Detectionfront;
extern Detectionflag g_DetectionBack;


void FrontDetection(int tof_data);
void BackDetection(int tof_data);
#endif /* OBSTACLEDETECTION_H_ */
