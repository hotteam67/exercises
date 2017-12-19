#include "WPILib.h"
#include "RobotUtils/RobotUtils.h"
#include "ctrlib/CANTalon.h"
#include "ctrlib/PigeonImu.h"

/* Exercise 01 is to use the joystick and motors (Talon SRX) 2, 3, 4 on the Sweet Bench
 *
 * Functional Requirements:
 * 1)  All functionality shall be done in TeleOp mode.
 * 2)  Use the joystick "A" button to cycle between controlling motor 2, motor 3, motor 4, or no motor.
 * 3)  Upon initialization, the control shall default to "no motor" for safety.
 * 4)  Button presses shall be interpreted as the transition from false to true.
 * 5)  The joystick left stick, x-axis shall control the speed and direction of the active motor.
 * 6)  The x-axis of the left stick should have a deadband from -0.2 to +0.2 to allow for
 *     joystick hysteresis (i.e. not quite returning to zero when released)
 * 7)  The lower "active" region of the joystick (-1.0 -> -0.2) shall command between -1.0 and 0.0 speed.
 * 8)  The upper "active" region of the joystick (+0.2 -> +1.0) shall command between 0.0 and +1.0 speed.
 * 9)  The A Button State, Commanded Speed, and Active Motor shall be output to the dashboard.
 *
 */

class benchTest: public HotBot {
private:

	HotJoystick* m_driver;
	CANTalon* m_CANmotor2;
	CANTalon* m_CANmotor3;
	CANTalon* m_CANmotor4;

	bool aButton;
	bool aButtonOld = false;
	bool motorStTransInProcess = false;
	float joystickRaw;
	float spdCmd;

	int motorSelect = 0;
	     /* 0: Off (no motor)
		    1: CANTalon2
			2: CANTalon3
			3: CANTalon4 */

public:
	benchTest() {
		m_driver = new HotJoystick(0);
		m_CANmotor2 = new CANTalon(2);
		m_CANmotor3 = new CANTalon(3);
		m_CANmotor4 = new CANTalon(4);
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

		 /* Read inputs from Joystick */
		 aButton = m_driver->ButtonA();
		 joystickRaw = m_driver->AxisLX();  /* Uses Left Stick, X axis */

		 /* Deadband input, determine joystickMod */
		 if(joystickRaw <= 0.2 && joystickRaw >= -0.2){
			 spdCmd = 0.0;
		 }else if(joystickRaw > 0.2){
			 spdCmd = (joystickRaw - 0.2) * 1.25;
		 }else{
			 spdCmd = (joystickRaw + 0.2) * 1.25;
		 }

		 /* Process button presses */
		 /* Look for aButton to transition from false to true */
		 if ((aButton == true) && (aButtonOld == false)) {
			 motorStTransInProcess = true;
		 } else {
			 motorStTransInProcess = false;
		 }

		 /* Use motorSelect to define speed commands to motors */
		 if (motorStTransInProcess == true) {
			 if (motorSelect < 3) {
				 motorSelect++;
			 } else {
				 motorSelect = 0;
			 }
		 }

		 /* Command speeds to motor controllers */
		 switch (motorSelect) {
		 case 0:
			 m_CANmotor2->Set(0.0);
			 m_CANmotor3->Set(0.0);
			 m_CANmotor4->Set(0.0);
			 break;
		 case 1:
			 m_CANmotor2->Set(spdCmd);
			 m_CANmotor3->Set(0.0);
			 m_CANmotor4->Set(0.0);
			 break;
		 case 2:
			 m_CANmotor2->Set(0.0);
			 m_CANmotor3->Set(spdCmd);
			 m_CANmotor4->Set(0.0);
			 break;
		 case 3:
			 m_CANmotor2->Set(0.0);
			 m_CANmotor3->Set(0.0);
			 m_CANmotor4->Set(spdCmd);
			 break;
		 }

		 /* Preserve knowledge of previous loop button state */
		 aButtonOld = aButton;

		 DashboardOutput();
	}

	void DashboardOutput() {
		/* Writes variables to Dashboard */
		SmartDashboard::PutBoolean("ButtonA", aButton);
		SmartDashboard::PutNumber("joystickRaw", joystickRaw);
		SmartDashboard::PutNumber("motorSelect", motorSelect);
		SmartDashboard::PutNumber("SpdCmd", spdCmd);
	}


	void TestPeriodic() {
	}

};

START_ROBOT_CLASS(benchTest)
