/*
 * CANmotor.cpp
 *
 *  Created on: Dec 26, 2017
 *      Author: mas2
 */

#include <CANmotor.h>
#include "WPILib.h"
#include "ctre/Phoenix.h"

CANmotor::CANmotor(int number) {
	m_CANmotor = new TalonSRX(number);
	m_CANmotor->Set(ControlMode::PercentOutput, 0);
	m_CANmotor->SetNeutralMode(Coast);
	m_CANmotor->ConfigSelectedFeedbackSensor(FeedbackDevice::CTRE_MagEncoder_Relative, PID_INDEX, TIMEOUT);
	m_CANmotor->SetSensorPhase(true);
	m_CANmotor->ConfigNominalOutputForward(0, TIMEOUT);
	m_CANmotor->ConfigNominalOutputReverse(0, TIMEOUT);
	m_CANmotor->ConfigPeakOutputForward(1, TIMEOUT);
	m_CANmotor->ConfigPeakOutputReverse(-1, TIMEOUT);
	m_CANmotor->SetSelectedSensorPosition(0, PID_INDEX, TIMEOUT);
}

void CANmotor::Enable(double speed) {
	/* To find F-Gain with Phoenix - measure RPM with 50% output commanded with ControlMode::PercentOutput
	 * F-Gain = 75.0525 / RPM
	 * No idea how the 75.0525 is calculated, but it was derived empirically.
	 */
	m_CANmotor->Config_kF(SLOT, GAIN_F, TIMEOUT);
	m_CANmotor->Config_kP(SLOT, GAIN_P, TIMEOUT);
	m_CANmotor->Config_kI(SLOT, GAIN_I, TIMEOUT);
	m_CANmotor->Config_kD(SLOT, GAIN_D, TIMEOUT);
	m_CANmotor->Set(ControlMode::Velocity, (speed * (4096.0/600.0)));
}

void CANmotor::Disable() {
	m_CANmotor->Set(ControlMode::PercentOutput, 0);
	this->ResetPosition();
}

void CANmotor::ResetPosition() {
	m_CANmotor->SetSelectedSensorPosition(0, PID_INDEX, TIMEOUT);
}

double CANmotor::GetSpeedRPM() {
	return (m_CANmotor->GetSelectedSensorVelocity(PID_INDEX) * (600.0/4096.0));
}

double CANmotor::GetSpeedErrorRPM() {
	return (m_CANmotor->GetClosedLoopError(PID_INDEX) * (600.0/4096.0));
}

double CANmotor::GetMotorOutput() {
	return m_CANmotor->GetMotorOutputPercent();
}

double CANmotor::GetSpeedNative() {
	return (m_CANmotor->GetSelectedSensorVelocity(PID_INDEX));
}

double CANmotor::GetOutputCurrent() {
	return m_CANmotor->GetOutputCurrent();
}

double CANmotor::GetPosition() {
	return m_CANmotor->GetSelectedSensorPosition(PID_INDEX);
}

CANmotor::~CANmotor() {

}

