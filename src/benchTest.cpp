#include <CANmotor.h>
#include "WPILib.h"
#include "RobotUtils/HotJoystick.h"
#include "ctre/Phoenix.h"
#include <stdlib.h>

/* Exercise 05 uses the following I/O on the Sweet Bench:
 *      Analog Potentiometer
 *      CANTalon Motor Controller
 *      Magnetic encoder
 *      Pushbutton switch
 *
 * Functional Requirements:
 * 1)  All functionality shall be done in TeleOp mode.
 * 2)  Use potentiometer [AnalogInput(0)] to control magnitude of speed command.  The analog input
 *     should scale the speed command between 0 and 3600 rpm.
 * 3)  Use pushbutton [DIO4] to enable/disable PID speed control of motor 3 when the pushbutton
 *     is initially pressed (buttonState == true && buttonStateOld == false).
 * 4)  The PID speed control shall be done using the CAN Talon PID library functions.  See documentation
 *     for CTR Electronics TALON SRX Software Reference Manual.
 * 5)  When enabling the robot, the PID speed control of motor 3 shall default to "disabled".
 * 6)  Use the magnetic encoder to measure the shaft speed in revs/minute.  Output speed to dashboard.
 * 7)  Create a new class, CANmotor that accepts an integer argument at initialization
 *     to specify the CAN channel to send to the Talon.  Additionally, create a "Set" function in
 *     the CANmotor class that commands the motor speed.
 * 8)  Tune P, I, D and F gains for responsive and accurate speed control.  Configure CANmotor
 *     class so that gain constants can be easily tweaked in the code.
 *
 */

class benchTest: public IterativeRobot {
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
	double rotations;
	double spdNative;

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
		actSpdRPM = m_motor3->GetSpeedRPM();
		motorCurrent = m_motor3->GetOutputCurrent();
		spdNative = m_motor3->GetSpeedNative();

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
		rotations = (m_motor3->GetPosition() / 4096.0);  /* divide by number of pulses/rev for total revolutions. */
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
		SmartDashboard::PutNumber("PositionTicks", m_motor3->GetPosition());
		SmartDashboard::PutNumber("Rotations", rotations);
		SmartDashboard::PutNumber("spdNative", spdNative);
		SmartDashboard::PutNumber("MotorOutputPct", m_motor3->GetMotorOutput());
		SmartDashboard::PutNumber("SpeedError", m_motor3->GetSpeedErrorRPM());
	}


	void TestPeriodic() {
	}

};

START_ROBOT_CLASS(benchTest)
