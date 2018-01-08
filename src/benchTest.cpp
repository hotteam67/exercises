#include "WPILib.h"
#include "RobotUtils/HotJoystick.h"


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

class benchTest: public IterativeRobot {
private:

	HotJoystick* m_driver;

public:
	benchTest() {
		m_driver = new HotJoystick(0);

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
		DashboardOutput();

	}

	void DashboardOutput() {
		SmartDashboard::PutBoolean("ButtonA", m_driver->ButtonA());
		SmartDashboard::PutBoolean("ButtonB", m_driver->ButtonB());
		SmartDashboard::PutBoolean("ButtonX", m_driver->ButtonX());
		SmartDashboard::PutBoolean("ButtonY", m_driver->ButtonY());
		SmartDashboard::PutBoolean("ButtonRB", m_driver->ButtonRB());
		SmartDashboard::PutBoolean("ButtonLB", m_driver->ButtonLB());

		SmartDashboard::PutNumber("JoystickLY", m_driver->AxisLY());
		SmartDashboard::PutNumber("JoystickRY", m_driver->AxisRY());
		SmartDashboard::PutNumber("JoystickLX", m_driver->AxisLX());
		SmartDashboard::PutNumber("JoystickRX", m_driver->AxisRX());

		SmartDashboard::PutNumber("TriggerL", m_driver->AxisLT());
		SmartDashboard::PutNumber("TriggerR", m_driver->AxisRT());
	}


	void TestPeriodic() {
	}

};

START_ROBOT_CLASS(benchTest)
