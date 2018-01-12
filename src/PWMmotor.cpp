/*
 * PWMmotor.cpp
 *
 *  Created on: Dec 26, 2017
 *      Author: mas2
 */

#include <PWMmotor.h>
#include "WPILib.h"

PWMmotor::PWMmotor(int number) {
	m_PWMmotor = new Victor(number);
	accumulatedError = 0.0;
	pComp = 0.0;
	iComp = 0.0;
	dComp = 0.0;
	fComp = 0.0;
	speedError = 0.0;
	spdErrorLast = 0.0;
	motorPctCmd = 0.0;
	motorPctRaw = 0.0;
}

void PWMmotor::SetP(double p) {
	P_GAIN = p;
}

void PWMmotor::SetI(double i) {
	I_GAIN = i;
}

void PWMmotor::SetD(double d) {
	D_GAIN = d;
}

void PWMmotor::SetF(double f) {
	F_GAIN = f;
}

double PWMmotor::GetPcomp() {
	return pComp;
}

double PWMmotor::GetIcomp() {
	return iComp;
}

double PWMmotor::GetDcomp() {
	return dComp;
}

double PWMmotor::GetFcomp() {
	return fComp;
}

double PWMmotor::GetSpeedError() {
	return speedError;
}

double PWMmotor::GetMotorPctCmd() {
	return motorPctCmd;
}

double PWMmotor::GetMotorPctRaw() {
	return motorPctRaw;
}

double PWMmotor::GetAccumulatedError() {
	return accumulatedError;
}

void PWMmotor::Disable(double speed) {
	accumulatedError = 0.0;
	pComp = 0.0;
	iComp = 0.0;
	dComp = 0.0;
	fComp = 0.0;
	spdErrorLast = 0.0;
	speedError = 0.0;
	motorPctCmd = 0.0;
	motorPctRaw = 0.0;

	/* Set motor speed */
	m_PWMmotor->Set(0.0);
}

void PWMmotor::MaintainPID(double speedCmd, double speedAct, double timeDelta) {
	/* Calculate error and accumulated error */
	speedError = speedCmd - speedAct;
	accumulatedError = accumulatedError + speedError;

	/* Calculated F, P, I and D components */
	fComp = speedCmd * F_GAIN;
	pComp = speedError * P_GAIN;
	iComp = accumulatedError * I_GAIN;
	dComp = ((speedError - spdErrorLast) / (timeDelta/1000.0)) * D_GAIN;

	/* Store Speed Error Last for next loops derivative calculation */
	spdErrorLast = speedError;

	/* Raw motor command = f + p + i + d */
	motorPctRaw = fComp + pComp + iComp + dComp;

	/* Limit Motor Command between 0.0 and 1.0 */
	if (motorPctRaw < 0.0) {
		motorPctCmd = 0.0;
	} else if (motorPctRaw > 1.0) {
		motorPctCmd = 1.0;
	} else {
		motorPctCmd = motorPctRaw;
	}

	/* Set motor speed on Victor in % */
	m_PWMmotor->Set(motorPctCmd);
}


PWMmotor::~PWMmotor() {

}

