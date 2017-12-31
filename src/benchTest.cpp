#include "WPILib.h"
#include "RobotUtils/RobotUtils.h"
#include "PWMmotor.h"
#include "stdlib.h"
#include <chrono>

/* Exercise 06 uses the following I/O on the Sweet Bench:
 *      Analog Potentiometer (Analog0)
 *      Victor Motor Controller (PWM0)
 *      256 counts/rev encoder (DIO2 and DIO3)
 *      Pushbutton switch (DIO4)
 *      LED Ring (Relay 1)
 *
 * The intent of Exercise 06 is to create a class to implement a PIDF speed controller from the ground up.
 *
 *
 * Functional Requirements:
 * 1)  All functionality shall be done in TeleOp mode.
 * 2)  Create a new class, PWM motor which will implement PIDF control of using the PWM Victor motor controller.
 * 3)  The Analog Potentiometer shall be used to control the shaft speed, from 0 to 800 rpm.
 * 4)  The quadrature encoder on DIO2 and DIO3 shall be used to measure the shaft speed.
 * 5)  The pushbutton switch shall be used to enable/disable the speed control.
 * 6)  When the speed control is disabled, all variables, especially accumulated variables, shall be zeroed.
 * 7)  When enabled, the functions in the PWM motor class shall control to the target speed.
 * 8)  When the speed error (desired speed - actual speed) is within +/- 25 rpm, the LED ring shall be turned on.
 * 9)  All the P, I, D and F components of the speed command shall be output to the dashboard.
 * 10) The motor percent command, target speed, speed error, and accumulated error shall be output to the dashboard.
 *
 *
 *   800 rpm = 100% speed
 */

class benchTest: public HotBot {
private:

	AnalogPotentiometer* m_Analog0;
	DigitalInput* m_SwitchDIO4;
	Encoder* m_EncoderMtr5;
	PWMmotor* m_motor5;
	Relay* m_LED_Ring;

	float spdCmd;
	double potentio;
	bool switchState;
	bool switchStateOld = false;
	bool motorEnabled = false;
	double encoderTicksMtr5;
	double actSpdRPM;
	double deadBand = 25.0;
	std::chrono::time_point<std::chrono::high_resolution_clock> timeNow = std::chrono::high_resolution_clock::now();
	std::chrono::time_point<std::chrono::high_resolution_clock> timeLast = std::chrono::high_resolution_clock::now();
	double timeDelta = 0.0; /* milliseconds */



public:
	benchTest() {
		m_SwitchDIO4 = new DigitalInput(4);
		AnalogInput* ai = new AnalogInput(0);
		m_Analog0 = new AnalogPotentiometer(ai, 1, 0);
		m_EncoderMtr5 = new Encoder(2,3,true,Encoder::EncodingType::k4X);
		m_motor5 = new PWMmotor(0);
		m_LED_Ring = new Relay(1);
		m_motor5->SetP(0.0005);
		m_motor5->SetI(0.01);
		m_motor5->SetD(0.0);
		m_motor5->SetF(1.0 / 800.0);
	}
	void RobotInit() {
	}


	void AutonomousInit() {

	}

	void AutonomousPeriodic() {

	}

	void TeleopInit() {
		m_EncoderMtr5->SetSamplesToAverage(15);
		m_EncoderMtr5->SetDistancePerPulse(1.0 / 256.0); /* 1 rotation per 256 pulses */
	}

	void TeleopPeriodic() {

		/* Read inputs */
		potentio = m_Analog0->Get();
		switchState = !m_SwitchDIO4->Get();
		encoderTicksMtr5 = m_EncoderMtr5->Get();

		/* Read clock for loop time */
		timeNow = std::chrono::high_resolution_clock::now();
		timeDelta = std::chrono::duration_cast<std::chrono::milliseconds>(timeNow-timeLast).count();
		timeLast = timeNow;

		/* Calculate speed from potentiometer input */
		spdCmd = potentio * 800.0;

		/* Convert encoder rate to rpm */
		actSpdRPM = (m_EncoderMtr5->GetRate() * 60.0);


		/* Change motor enabled state if switch (DIO4) is pressed */
		if ((switchState) && (!switchStateOld)) {
			motorEnabled = !motorEnabled;
		}

		/* Command speed to motor */
		if (!motorEnabled) {
			m_motor5->Disable(actSpdRPM);
		} else {
			m_motor5->MaintainPID(spdCmd, actSpdRPM, timeDelta);
		}

		/* Is speed error within dead band, if yes - light LED Ring */
		if ((abs(m_motor5->GetSpeedError() < deadBand)) && (motorEnabled)) {
			m_LED_Ring->Set(Relay::Value::kForward);
		} else {
			m_LED_Ring->Set(Relay::Value::kOff);
		}

		/* remember old switch state */
		switchStateOld = switchState;

		/* Output values to the dashboard */
		DashboardOutput();
	}

	void DashboardOutput() {
		/* Writes variables to Dashboard */
		SmartDashboard::PutBoolean("switchState", switchState);
		SmartDashboard::PutBoolean("motorEnabled", motorEnabled);
		SmartDashboard::PutNumber("SpdCmd", spdCmd);
		SmartDashboard::PutNumber("potentio", potentio);
		SmartDashboard::PutNumber("actSpdRPM", actSpdRPM);
		SmartDashboard::PutNumber("encoderTicksMtr5", encoderTicksMtr5);
		SmartDashboard::PutNumber("timeDelta", timeDelta);
		SmartDashboard::PutNumber("speedError", m_motor5->GetSpeedError());
		SmartDashboard::PutNumber("accumulatedError", m_motor5->GetAccumulatedError());
		SmartDashboard::PutNumber("PID fComp", m_motor5->GetFcomp());
		SmartDashboard::PutNumber("PID pComp", m_motor5->GetPcomp());
		SmartDashboard::PutNumber("PID iComp", m_motor5->GetIcomp());
		SmartDashboard::PutNumber("PID dComp", m_motor5->GetDcomp());
		SmartDashboard::PutNumber("motorPctRaw", m_motor5->GetMotorPctRaw());
		SmartDashboard::PutNumber("motorPctCmd", m_motor5->GetMotorPctCmd());
	}


	void TestPeriodic() {
	}

};

START_ROBOT_CLASS(benchTest)
