/*
 * CANmotor.cpp
 *
 *  Created on: Dec 26, 2017
 *      Author: mas2
 */

#include <CANmotor.h>
#include "WPILib.h"
#include "RobotUtils/RobotUtils.h"
#include "ctrlib/CANTalon.h"
#include <chrono>

CANmotor::CANmotor(int number) {
	m_CANmotor = new CANTalon(number);
	m_CANmotor->SetControlMode(CANTalon::kSpeed);
	m_CANmotor->ConfigNeutralMode(CANTalon::NeutralMode::kNeutralMode_Coast);
	m_CANmotor->SetFeedbackDevice(CANTalon::CtreMagEncoder_Relative);
	m_CANmotor->ConfigEncoderCodesPerRev(4096);
	m_CANmotor->SetSensorDirection(true);
}

void CANmotor::Enable(double speed) {
	m_CANmotor->Set(speed);
	m_CANmotor->SetPID(GAIN_P,GAIN_I, GAIN_D);
	m_CANmotor->SetF(GAIN_F);
	m_CANmotor->Enable();
}

void CANmotor::Disable() {
	m_CANmotor->Disable();
}

double CANmotor::GetSpeed() {
	return m_CANmotor->GetSpeed();
}

double CANmotor::GetOutputCurrent() {
	return m_CANmotor->GetOutputCurrent();
}

CANmotor::~CANmotor() {

}

