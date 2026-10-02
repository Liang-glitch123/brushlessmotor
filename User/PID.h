#ifndef __PID_H
#define __PID_H

#include "stm32f4xx.h"
#include <stdbool.h>

extern bool PIDflag;

typedef struct Factor
{
		float Target;
		float Actual;
		float Out;
	
		float Kp;
		float Ki;
		float Kd;
	
		float Error0;
		float Error1;
		float ErrorInt;
}PID_Factor;

extern PID_Factor LocationPID;
extern PID_Factor SpeedPID;

void PID_Update(PID_Factor *PID);
void PID_Control(void);

#endif
