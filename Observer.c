#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <Observer.h>


double calculateYawAngle(int tof1, int tof2) {

    double distance_difference = (double)(tof1 - tof2);
    double scaled_distance_diff = TOF_DISTDIFF * (distance_difference / (fabs(distance_difference) + TOF_DISTDIFF));

    // arcsin(y/x) 계산하여 라디안으로 반환
    double yaw_angle_radians = asin(scaled_distance_diff / TOF_DISTDIFF);
    yaw_angle_radians = yaw_angle_radians /PI * 180.0;
    return yaw_angle_radians;
}


