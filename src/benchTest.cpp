#include "WPILib.h"
#include "RobotUtils/HotJoystick.h"
#include "ctre/Phoenix.h"


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

	TalonSRX * m_motor2;
	TalonSRX * m_motor3;
	TalonSRX * m_motor4;

	PigeonIMU * m_pigeon;

	double angleIMU[3];

	double kP = 0.01;
	double kI = 0.00;
	double kD = 0.00;



public:
	benchTest() {
		m_driver = new HotJoystick(0);

		m_motor2 = new TalonSRX(2);
		m_motor3 = new TalonSRX(3);
		m_motor4 = new TalonSRX(4);

		m_pigeon = new PigeonIMU(m_motor4);

	}
	void RobotInit() {

		//m_motor4->Config_kF(0, 0.1097, 0);
		m_motor3->ConfigSelectedFeedbackSensor(FeedbackDevice::CTRE_MagEncoder_Relative, 0, -1500.0 * 4096 / 600);
		m_motor3->SetSensorPhase(true);

		m_motor3->ConfigNominalOutputForward(0, 0);
		m_motor3->ConfigNominalOutputReverse(0, 0);
		m_motor3->ConfigPeakOutputForward(12, 0);
		m_motor3->ConfigPeakOutputReverse(-12, 0);

		m_motor3->Config_kP(0, 0.01, 0);
		m_motor3->Config_kI(0, 0.0, 0);
		m_motor3->Config_kD(0, 0.0, 0);
	}


	void AutonomousInit() {

	}

	void AutonomousPeriodic() {

	}

	void TeleopInit() {

	}

	void TeleopPeriodic() {
		DashboardOutput();

		if (m_driver->ButtonA()){
			m_motor3->Set(ControlMode::PercentOutput, 0.25);
		}
		else if (m_driver->ButtonB()) {
			m_motor3->Set(ControlMode::Velocity, 10);
		}
		else {
			m_motor3->Set(ControlMode::PercentOutput, 0.0);
		}


		m_pigeon->GetYawPitchRoll(angleIMU);
	}

	void DashboardOutput() {

		//Joystick
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

		//TalonPID
			SmartDashboard::PutNumber("TalonPosition", m_motor3->GetSelectedSensorPosition(0));
			SmartDashboard::PutNumber("TalonVelocity", m_motor3->GetSelectedSensorVelocity(0));

		//Talon Info
			SmartDashboard::PutNumber("Motor Percent Output", m_motor3->GetMotorOutputPercent());

		//PigeonIMU Info
			SmartDashboard::PutNumber("Yaw", angleIMU[0]);
			SmartDashboard::PutNumber("Temperature", m_pigeon->GetTemp());
	}


	void TestPeriodic() {
	}

};

START_ROBOT_CLASS(benchTest)
