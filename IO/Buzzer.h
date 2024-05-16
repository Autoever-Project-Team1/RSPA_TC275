#ifndef BUZZER_H_
#define BUZZER_H_

void IsrGpt120T3Handler_Beep(void);
void Init_Buzzer(void);
void setBeepCycle(int cycle);
void setToneCycle(int cycle);
void Start_Signature_Sound(void);

void init_gpt1(void);
void runGpt12_T3();



extern int flagf;
extern int flag;
extern int beepCnt;
extern int tone[];
extern int H_sound[];

#define ISR_PRIORITY_GPT1T3_TIMER   2

#endif /* BUZZER_H_ */
