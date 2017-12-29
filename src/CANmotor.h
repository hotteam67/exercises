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
#include "RobotUtils/RobotUtils.h"
#include "ctrlib/CANTalon.h"
#include <chrono>

#define GAIN_P 0.05
#define GAIN_I 0.0001
#define GAIN_D 0.0
#define GAIN_F 0.0

class CANmotor {
public:
	CANmotor(int number);
	double GetSpeed();
	double GetOutputCurrent();
	void Enable(double spd);
	void Disable();
	virtual ~CANmotor();
private:
	CANTalon* m_CANmotor;
};

#endif /* SRC_CANMOTOR_H_ */
