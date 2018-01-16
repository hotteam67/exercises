/*
 * CANmotor.h
 *
 *  Created on: Dec 26, 2017
 *      Author: mas2
 */

#ifndef SRC_CANMOTOR_H_
#define SRC_CANMOTOR_H_

#include <CANmotor.h>
#include "WPILib.h"
#include "ctre/Phoenix.h"

#define GAIN_P 0.0001
#define GAIN_I 0.0001
#define GAIN_D 0.000
#define GAIN_F 0.0339

/* To find F-Gain with Phoenix - measure RPM with 50% output commanded with ControlMode::PercentOutput
 * F-Gain = 75.0525 / RPM
 * No idea how the 75.0525 is calculated, but it was derived empirically.
 */

#define SLOT 0
#define PID_INDEX 0
#define TIMEOUT 10

class CANmotor {
public:
	CANmotor(int number);
	double GetSpeedRPM();
	double GetSpeedNative();
	double GetPosition();
	double GetOutputCurrent();
	double GetMotorOutput();
	double GetSpeedErrorRPM();
	void Enable(double spd);
	void Disable();
	void ResetPosition();
	virtual ~CANmotor();
private:
	TalonSRX* m_CANmotor;
};

#endif /* SRC_CANMOTOR_H_ */
