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
	void Set(double spd);
	virtual ~PWMmotor();
private:
	Victor* m_PWMmotor;
};

#endif /* SRC_PWMMOTOR_H_ */
