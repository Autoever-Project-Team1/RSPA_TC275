#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <Observer.h>


LuenbergerObserver observer;

void Tof_to_yaw_Init(void){
    initializeObserver(&observer);
}

// 관측기 초기화 함수
void initializeObserver(LuenbergerObserver* observer) {
    observer->estimated.yaw = 0.0;  // 초기 yaw 각도
    observer->gain = OBSERVER_GAIN; // 관측기 이득
}

// 관측기 업데이트 함수
void updateObserver(LuenbergerObserver* observer, int tof1, int tof2) {


    double distance_difference = (double)(tof1 - tof2);
    double scaled_distance_diff = TOF_DISTDIFF * (distance_difference / (fabs(distance_difference) + TOF_DISTDIFF));

    // arcsin(y/x) 계산하여 라디안으로 반환
    double yaw_angle_radians = asin(scaled_distance_diff / TOF_DISTDIFF);
    double yaw_error = yaw_angle_radians - observer->estimated.yaw;
    observer->estimated.yaw += observer->gain * yaw_error;
}

// 관측기에서 yaw 값을 반환하는 함수
double getYawFromObserver(const LuenbergerObserver* observer) {
    return observer->estimated.yaw;
}


///////


double calculateYawAngle(int tof1, int tof2) {

    double distance_difference = (double)(tof1 - tof2);
    double scaled_distance_diff = TOF_DISTDIFF * (distance_difference / (fabs(distance_difference) + TOF_DISTDIFF));

    // arcsin(y/x) 계산하여 라디안으로 반환
    double yaw_angle_radians = asin(scaled_distance_diff / TOF_DISTDIFF);
    return yaw_angle_radians;
}

