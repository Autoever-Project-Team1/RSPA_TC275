#ifndef OBSERVER_H_
#define OBSERVER_H_

#define OBSERVER_GAIN       0.1 // 크면 잡음에 민감?
#define PI                  3.14159265358979323846
#define TOF_DISTDIFF        52 //mm

// 상태 변수 구조체
typedef struct {
    double yaw;       // 차량의 요각 추정값
} State;

// 루엔버거 관측기 상태 구조체
typedef struct {
    State estimated;  // 추정된 상태 변수
    double gain;      // 관측기 이득
} LuenbergerObserver;

extern LuenbergerObserver observer;


void Tof_to_yaw_Init(void);
void initializeObserver(LuenbergerObserver* observer);
void updateObserver(LuenbergerObserver* observer, int tof1, int tof2);
double getYawFromObserver(const LuenbergerObserver* observer);

double calculateYawAngle(int tof1, int tof2);

#endif /* OBSERVER_H_ */
