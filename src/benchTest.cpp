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
	TalonSRX* m_leftFront;
	TalonSRX* m_leftMiddle;
	TalonSRX* m_leftRear;
	TalonSRX* m_rightFront;
	TalonSRX* m_rightMiddle;
	TalonSRX* m_rightRear;

	bool aButton;
	bool aButtonOld = false;
	bool motorStTransInProcess = false;
	float joystickRaw;
	float spdCmd;
	//encoder position
	double leftPosition;
	double rightPosition;

	int motorSelect = 0;
	     /* 0: Off (no motor)
		    1: CANTalon2
			2: CANTalon3
			3: CANTalon4 */

public:
	benchTest() {
		//set it && forget it
		m_driver = new HotJoystick(0);
		m_leftFront = new TalonSRX(7);
		m_leftMiddle = new TalonSRX(5);
		m_leftRear = new TalonSRX(6);
		m_rightFront = new TalonSRX(2);
		m_rightMiddle = new TalonSRX(4);
		m_rightRear = new TalonSRX(3);


		m_leftFront->SetNeutralMode(Coast);
		m_leftMiddle->SetNeutralMode(Coast);
		m_leftRear->SetNeutralMode(Coast);
		m_rightFront->SetNeutralMode(Coast);
		m_rightMiddle->SetNeutralMode(Coast);
		m_rightRear->SetNeutralMode(Coast);

		m_leftMiddle->Set(ControlMode::Follower, 7);
		m_leftRear->Set(ControlMode::Follower, 7);
		m_rightMiddle->Set(ControlMode::Follower, 2);
		m_rightRear->Set(ControlMode::Follower, 2);

		m_rightFront->SetInverted(true);
		m_rightMiddle->SetInverted(true);
		m_rightRear->SetInverted(true);

		m_leftFront->ConfigSelectedFeedbackSensor(QuadEncoder, 0, 0);
		m_rightFront->ConfigSelectedFeedbackSensor(QuadEncoder, 0, 0);
		m_leftFront->SetSelectedSensorPosition(0, 0, 0);
		m_rightFront->SetSelectedSensorPosition(0, 0, 0);
		m_leftFront->SetSensorPhase(true);
		m_rightFront->SetSensorPhase(true);
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
		 joystickRaw = m_driver->AxisLY();  /* Uses Left Stick, Y axis */
		 //joystickRaw1 = m_driver->AxisRY(); //Uses Right Stick, Y axis

		 /* Deadband input, determine joystickMod */
		 if(joystickRaw <= 0.2 && joystickRaw >= -0.2){
			 spdCmd = 0.0;
		 }else if(joystickRaw > 0.2){
			 spdCmd = (joystickRaw - 0.2) * -1.25;
		 }else{
			 spdCmd = (joystickRaw + 0.2) * -1.25;
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
			 if (motorSelect < 6) {
				 motorSelect++;
			 } else {
				 motorSelect = 0;
			 }
		 }

		 /*encoders*/
		 leftPosition = m_leftFront->GetSelectedSensorPosition(0);
		 rightPosition = m_leftFront->GetSelectedSensorPosition(0);

		 /* Command speeds to motor controllers */
		 switch (motorSelect) {
		 case 0:
			 m_leftFront->Set(ControlMode::PercentOutput, spdCmd);
			 m_rightFront->Set(ControlMode::PercentOutput, spdCmd);
			 break;
		 case 1:
			 m_leftFront->Set(ControlMode::PercentOutput, spdCmd);
			 m_leftMiddle->Set(ControlMode::PercentOutput, 0.0);
			 m_leftRear->Set(ControlMode::PercentOutput, 0.0);
			 m_rightFront->Set(ControlMode::PercentOutput, 0.0);
			 m_rightMiddle->Set(ControlMode::PercentOutput, 0.0);
			 m_rightRear->Set(ControlMode::PercentOutput, 0.0);
			 break;
		 case 2:
			 m_leftFront->Set(ControlMode::PercentOutput, 0.0);
			 m_leftMiddle->Set(ControlMode::PercentOutput, spdCmd);
			 m_leftRear->Set(ControlMode::PercentOutput, 0.0);
			 m_rightFront->Set(ControlMode::PercentOutput, 0.0);
			 m_rightMiddle->Set(ControlMode::PercentOutput, 0.0);
			 m_rightRear->Set(ControlMode::PercentOutput, 0.0);
			 break;
		 case 3:
			 m_leftFront->Set(ControlMode::PercentOutput, 0.0);
			 m_leftMiddle->Set(ControlMode::PercentOutput, 0.0);
			 m_leftRear->Set(ControlMode::PercentOutput, spdCmd);
			 m_rightFront->Set(ControlMode::PercentOutput, 0.0);
			 m_rightMiddle->Set(ControlMode::PercentOutput, 0.0);
			 m_rightRear->Set(ControlMode::PercentOutput, 0.0);
			 break;
		 case 4:
			 m_leftFront->Set(ControlMode::PercentOutput, 0.0);
			 m_leftMiddle->Set(ControlMode::PercentOutput, 0.0);
			 m_leftRear->Set(ControlMode::PercentOutput, 0.0);
			 m_rightFront->Set(ControlMode::PercentOutput, spdCmd);
			 m_rightMiddle->Set(ControlMode::PercentOutput, 0.0);
			 m_rightRear->Set(ControlMode::PercentOutput, 0.0);
			 break;
		 case 5:
			 m_leftFront->Set(ControlMode::PercentOutput, 0.0);
			 m_leftMiddle->Set(ControlMode::PercentOutput, 0.0);
			 m_leftRear->Set(ControlMode::PercentOutput, 0.0);
			 m_rightFront->Set(ControlMode::PercentOutput, 0.0);
			 m_rightMiddle->Set(ControlMode::PercentOutput, spdCmd);
			 m_rightRear->Set(ControlMode::PercentOutput, 0.0);
			 break;
		 case 6:
			 m_leftFront->Set(ControlMode::PercentOutput, 0.0);
			 m_leftMiddle->Set(ControlMode::PercentOutput, 0.0);
			 m_leftRear->Set(ControlMode::PercentOutput, 0.0);
			 m_rightFront->Set(ControlMode::PercentOutput, 0.0);
			 m_rightMiddle->Set(ControlMode::PercentOutput, 0.0);
			 m_rightRear->Set(ControlMode::PercentOutput, spdCmd);
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
		SmartDashboard::PutNumber("LeftEncoderPosition", leftPosition);
		SmartDashboard::PutNumber("RightEncoderPosition", rightPosition);
	}


	void TestPeriodic() {
	}

};

START_ROBOT_CLASS(benchTest)
