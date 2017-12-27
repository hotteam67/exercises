/*
 * PWMmotor.cpp
 *
 *  Created on: Dec 26, 2017
 *      Author: mas2
 */

#include <PWMmotor.h>
#include "WPILib.h"
#include "RobotUtils/RobotUtils.h"

PWMmotor::PWMmotor(int number) {
	m_PWMmotor = new Victor(number);
}

void PWMmotor::Set(double speed) {
	m_PWMmotor->Set(speed);
}

PWMmotor::~PWMmotor() {

}

