#include "PID.h"
#include "bsp_motor.h"
#include "bsp_bldcm_control.h"
#include <math.h>

bool PIDflag = false;

PID_Factor LocationPID = {
	.Kp = 0.4,
	.Ki = 0,
	.Kd = 0.3,
};

PID_Factor SpeedPID = {
	.Kp = 2,
	.Ki = 0.1,
	.Kd = 0,
};

void PID_Update(PID_Factor *PID)
{
		PID->Error1 = PID->Error0;
		PID->Error0 = PID->Target - PID->Actual;
		
		if (PID->Ki != 0)
		{
			PID->ErrorInt += PID->Error0;
		}
		else
		{
			PID->ErrorInt = 0;
		}
		
		PID->Out = PID->Kp * PID->Error0 + PID->Ki * PID->ErrorInt + PID->Kd * (PID->Error0 - PID->Error1);
		
		if (PID->Out > PID->Target*1.5f) {PID->Out = PID->Target*1.5f;}
		if (PID->Out < PID->Target*(-1.5f)) {PID->Out = PID->Target*(-1.5f);}
}

void PID_Control(void)
{
	if(PIDflag && bldcm_data.is_enable)
	{
		SpeedPID.Actual = motor_drive.speed;
		
		PID_Update(&SpeedPID);
		
		set_bldcm_speed(SpeedPID.Out);
		PIDflag = false;
	}
}
