/*
 * PWMmotor.h
 *
 *  Created on: Dec 26, 2017
 *      Author: mas2
 */

#ifndef SRC_PWMMOTOR_H_
#define SRC_PWMMOTOR_H_

#include <PWMmotor.h>
#include "WPILib.h"

class PWMmotor {
public:
	PWMmotor(int number);
	void SetP(double p);
	void SetI(double i);
	void SetD(double d);
	void SetF(double f);
	double GetPcomp();
	double GetIcomp();
	double GetDcomp();
	double GetFcomp();
	double GetSpeedError();
	double GetMotorPctCmd();
	double GetMotorPctRaw();
	double GetAccumulatedError();
	void MaintainPID(double speedCmd, double speedAct, double timeDelta);
	void Disable(double speed);
	virtual ~PWMmotor();
private:
	Victor* m_PWMmotor;
	double P_GAIN;
	double I_GAIN;
	double D_GAIN;
	double F_GAIN;
	double spdCmd;
	double speedError;
	double accumulatedError;
	double pComp, iComp, dComp, fComp;
	double spdErrorLast;
	double motorPctCmd;
	double motorPctRaw;
};

#endif /* SRC_PWMMOTOR_H_ */
