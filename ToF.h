#ifndef TOF_H_
#define TOF_H_

static int verifyCheckSum(unsigned char data[]);
static int checkTofStrength(unsigned char data[]);

extern void Init_ToF(void);
extern int getTofDistance_Tof(void);


void Nlink_Read_Tof(char id);
uint8 Nlink_Read_Tof_ID(void);

void Nlink_getTofDistance_Tof(uint8 n_Tof);

extern uint8 Tof_ID;
extern int Tof_dist_mm[10];

#endif /* TOF_H_ */
