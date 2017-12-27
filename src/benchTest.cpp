#include "WPILib.h"
#include "RobotUtils/RobotUtils.h"
#include "PWMmotor.h"

/* Exercise 04 uses the following I/O on the Sweet Bench:
 *      Analog Potentiometer
 *      Victor Motor Controller
 *      256 counts/rev encoder
 *      Pushbutton switch
 *
 * Functional Requirements:
 * 1)  All functionality shall be done in TeleOp mode.
 * 2)  Use potentiometer [AnalogInput(0)] to control magnitude of speed command.
 * 3)  Use pushbutton [DIO4] to trigger a motor direction change when the pushbutton is initially
 *     pressed (buttonState == true && buttonStateOld == false).
 * 4)  Only trigger a direction flip is the encoder speed [using inputs 2 and 3] is less than 50 rpm.
 * 5)  Use the encoder to measure the shaft speed in revs/minute.  Output speed to dashboard.
 * 6)  Create a new class, PWMmotor that accepts an integer arguement at initialization
 *     to specify the PWM channel to send to the Victor.  Additionally, create a "Set" function in
 *     the PWMmotor class that commands the motor speed.
 *
 */

class benchTest: public HotBot {
private:

	AnalogPotentiometer* m_Analog0;
	DigitalInput* m_SwitchDIO4;
	Encoder* m_EncoderMtr5;
	PWMmotor* m_motor5;

	float spdCmd;
	double potentio;
	bool switchState;
	bool switchStateOld = false;
	bool direction = true;
	double encoderTicksMtr5;
	double actSpdRPM;


public:
	benchTest() {
		m_SwitchDIO4 = new DigitalInput(4);
		AnalogInput* ai = new AnalogInput(0);
		m_Analog0 = new AnalogPotentiometer(ai, 360, 0);
		m_EncoderMtr5 = new Encoder(2,3,true,Encoder::EncodingType::k4X);
		m_motor5 = new PWMmotor(0);


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

		/* Calculate speed from potentiometer input */
		spdCmd = potentio / 360;

		/* Convert encoder rate to rpm */
		actSpdRPM = (m_EncoderMtr5->GetRate() * 60.0);


		/* Change motor direction if switch (DIO4) is pressed and
		 * motor speed is less than 50 rpm.  */
		if ((switchState) && (!switchStateOld) && (actSpdRPM < 50.0)) {
			direction = !direction;
		}

		/* Command speed to motor */
		if (direction) {
			m_motor5->Set(spdCmd);
		} else {
			m_motor5->Set(-spdCmd);
		}


		/* remember old switch state */
		switchStateOld = switchState;


		/* Output values to the dashboard */
		DashboardOutput();
	}

	void DashboardOutput() {
		/* Writes variables to Dashboard */
		SmartDashboard::PutBoolean("switchState", switchState);
		SmartDashboard::PutBoolean("direction", direction);
		SmartDashboard::PutNumber("SpdCmd", spdCmd);
		SmartDashboard::PutNumber("potentio", potentio);
		SmartDashboard::PutNumber("actSpdRPM", actSpdRPM);
		SmartDashboard::PutNumber("encoderTicksMtr5", encoderTicksMtr5);
	}


	void TestPeriodic() {
	}

};

START_ROBOT_CLASS(benchTest)
