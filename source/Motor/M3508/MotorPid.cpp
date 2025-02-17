#include "MotorPid.hpp"

ClassicPidStructTypedef MotorPositionPid1, MotorSpeedPid1, MotorCurrentPid1;
ClassicPidStructTypedef MotorPositionPid2, MotorSpeedPid2, MotorCurrentPid2;

void MotorCurrentPidInit(void)
{
	MotorCurrentPid1.Kp = 1.0f;
	MotorCurrentPid1.Ki = 0.05f;
	// MotorCurrentPid1.Ki = 0.0f;
	MotorCurrentPid1.Kd = 0.0f;
	MotorCurrentPid1.LimitOutput = 5000.0f;
	MotorCurrentPid1.LimitIntegral = 4000.0f;
	MotorCurrentPid1.Integral = 0;
	MotorCurrentPid1.PreError = 0;
	MotorCurrentPid1.SectionFlag = 0;

	MotorCurrentPid2.Kp = 1.0f;
	MotorCurrentPid2.Ki = 0.05f;
	// MotorCurrentPid2.Ki = 0.0f;
	MotorCurrentPid2.Kd = 0.0f;
	MotorCurrentPid2.LimitOutput = 5000.0f;
	MotorCurrentPid2.LimitIntegral = 4000.0f;
	MotorCurrentPid2.Integral = 0;
	MotorCurrentPid2.PreError = 0;
	MotorCurrentPid2.SectionFlag = 0;
}

void MotorSpeedPidInit(void)
{

	MotorSpeedPid1.Kp = 250.0f;
	MotorSpeedPid1.Ki = 5.0f;
	MotorSpeedPid1.Kd = 0.0f;
	MotorSpeedPid1.LimitOutput = 16000.0f;	 // 16000.0f;
	MotorSpeedPid1.LimitIntegral = 10000.0f; // 10000.0f;
	MotorSpeedPid1.Integral = 0;
	MotorSpeedPid1.PreError = 0;
	MotorSpeedPid1.SectionFlag = 0;
	MotorSpeedPid1.ErrorLine = 1000;
	MotorSpeedPid1.KpMax = 400;

	MotorSpeedPid2.Kp = 250.0f;
	MotorSpeedPid2.Ki = 5.0f;
	MotorSpeedPid2.Kd = 0.0f;
	MotorSpeedPid2.LimitOutput = 16000.0f;	 // 16000.0f;
	MotorSpeedPid2.LimitIntegral = 10000.0f; // 10000.0f;
	MotorSpeedPid2.Integral = 0;
	MotorSpeedPid2.PreError = 0;
	MotorSpeedPid2.SectionFlag = 0;
	MotorSpeedPid2.ErrorLine = 1000;
	MotorSpeedPid2.KpMax = 400;

	// MotorSpeedPid1.Kp = 200.0f;
	// MotorSpeedPid1.Ki = 1.0f;
	// MotorSpeedPid1.Kd = 0.0f;
	// MotorSpeedPid1.LimitOutput = 5000.0f;
	// MotorSpeedPid1.LimitIntegral = 4000.0f;
	// MotorSpeedPid1.Integral = 0;
	// MotorSpeedPid1.PreError = 0;
	// MotorSpeedPid1.SectionFlag = 0;
	// MotorSpeedPid1.ErrorLine = 1000;
	// MotorSpeedPid1.KpMax = 400;

	// MotorSpeedPid2.Kp = 200.0f;
	// MotorSpeedPid2.Ki = 1.0f;
	// MotorSpeedPid2.Kd = 0.0f;
	// MotorSpeedPid2.LimitOutput = 5000.0f;
	// MotorSpeedPid2.LimitIntegral = 4000.0f;
	// MotorSpeedPid2.Integral = 0;
	// MotorSpeedPid2.PreError = 0;
	// MotorSpeedPid2.SectionFlag = 0;
	// MotorSpeedPid2.ErrorLine = 1000;
	// MotorSpeedPid2.KpMax = 400;
}

void MotorPositionPidInit(void)
{

	MotorPositionPid1.Kp = 25; // 2.5
	MotorPositionPid1.Ki = 0.01;
	MotorPositionPid1.Kd = 0.5;
	MotorPositionPid1.LimitOutput = 40; // 00
	MotorPositionPid1.LimitIntegral = 200;
	MotorPositionPid1.Integral = 0;
	MotorPositionPid1.PreError = 0;
	MotorPositionPid1.SectionFlag = 0;
	MotorPositionPid1.ErrorLine = 20;
	MotorPositionPid1.KpMax = 50;

	MotorPositionPid2.Kp = 25; // 2.5
	MotorPositionPid2.Ki = 0.01;
	MotorPositionPid2.Kd = 0.5;
	MotorPositionPid2.LimitOutput = 40; // 100
	MotorPositionPid2.LimitIntegral = 200;
	MotorPositionPid2.Integral = 0;
	MotorPositionPid2.PreError = 0;
	MotorPositionPid2.SectionFlag = 0;
	MotorPositionPid2.ErrorLine = 0;
	MotorPositionPid2.KpMax = 50;
}

void MotorPidInit(void)
{
	MotorCurrentPidInit();
	MotorSpeedPidInit();
	MotorPositionPidInit();
}

float ClassicPidRegulate(float Reference, float PresentFeedback, ClassicPidStructTypedef *PID_Struct)
{
	float error;
	float error_inc;
	float pTerm;
	float iTerm;
	float dTerm;
	float dwAux;
	float output;
	/*error computation*/
	error = Reference - PresentFeedback;

	/*proportional term computation*/

	if (PID_Struct->SectionFlag == 1)
	{
		if (fabs(error) >= PID_Struct->ErrorLine)
			pTerm = error * PID_Struct->KpMax;
		else
			pTerm = error * PID_Struct->Kp;
	}
	else
		pTerm = error * PID_Struct->Kp;

	/*Integral term computation*/

	// iTerm = ( fabs(error) <  Motor_1.SpeedExpected/5 )  ? error * PID_Struct->Ki : error * 0.001f ;
	iTerm = error * PID_Struct->Ki;

	dwAux = PID_Struct->Integral + iTerm;
	/*limit integral*/
	if (dwAux > PID_Struct->LimitIntegral)
	{
		PID_Struct->Integral = PID_Struct->LimitIntegral;
	}
	else if (dwAux < -1 * PID_Struct->LimitIntegral)
	{
		PID_Struct->Integral = -1 * PID_Struct->LimitIntegral;
	}
	else
	{
		PID_Struct->Integral = dwAux;
	}
	/*differential term computation*/

	error_inc = error - PID_Struct->PreError;
	dTerm = error_inc * PID_Struct->Kd;
	PID_Struct->PreError = error;

	output = pTerm + PID_Struct->Integral + dTerm;

	/*limit output*/
	if (output >= PID_Struct->LimitOutput)
	{
		return (PID_Struct->LimitOutput);
	}
	else if (output < -1.0f * PID_Struct->LimitOutput)
	{
		return (-1.0f * PID_Struct->LimitOutput);
	}
	else
	{
		return output;
	}
}
