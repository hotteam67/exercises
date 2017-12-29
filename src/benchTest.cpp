#include <CANmotor.h>
#include "WPILib.h"
#include "RobotUtils/RobotUtils.h"
#include "ctrlib/CANTalon.h"
#include <chrono>
#include <stdlib.h>

/* Exercise 05 uses the following I/O on the Sweet Bench:
 *      Analog Potentiometer
 *      Talon Motor Controller
 *      Magnetic encoder
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
	CANmotor* m_motor3;

	double spdCmd;
	double potentio;
	bool switchState;
	bool switchStateOld = false;
	bool controlMode = false;
	double actSpdRPM;
	double motorCurrent;


public:
	benchTest() {
		m_SwitchDIO4 = new DigitalInput(4);
		AnalogInput* ai = new AnalogInput(0);
		m_Analog0 = new AnalogPotentiometer(ai, 3600, 0);
		m_motor3 = new CANmotor(3);


	}
	void RobotInit() {
	}


	void AutonomousInit() {

	}

	void AutonomousPeriodic() {

	}

	void TeleopInit() {

	}

	void TeleopPeriodic() {


		/* Read inputs */
		potentio = m_Analog0->Get();
		switchState = !m_SwitchDIO4->Get();

		/* Calculate speed from potentiometer input */
		spdCmd = potentio;

		/* Get Shaft Speed and motor current */
		actSpdRPM = m_motor3->GetSpeed();
		motorCurrent = m_motor3->GetOutputCurrent();


		/* Change control mode if switch (DIO4) is pressed */
		if ((switchState) && (!switchStateOld)) {
			controlMode = !controlMode;
		}

		/* Command speed to motor */
		if (controlMode) {
			m_motor3->Enable(spdCmd);
		} else {
			m_motor3->Disable();
		}


		/* remember old switch state */
		switchStateOld = switchState;


		/* Output values to the dashboard */
		DashboardOutput();
	}

	void DashboardOutput() {
		/* Writes variables to Dashboard */
		SmartDashboard::PutBoolean("switchState", switchState);
		SmartDashboard::PutBoolean("Enabled", controlMode);
		SmartDashboard::PutNumber("SpdCmd", spdCmd);
		SmartDashboard::PutNumber("potentio", potentio);
		SmartDashboard::PutNumber("actSpdRPM", actSpdRPM);
		SmartDashboard::PutNumber("motorCurrent", motorCurrent);
	}


	void TestPeriodic() {
	}

};

START_ROBOT_CLASS(benchTest)
