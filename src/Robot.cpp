/**

 */

#include "WPILib.h"
#include "MotionProfileExample.h"
#include "ctre/Phoenix.h"
#include "RobotUtils/HotJoystick.h"

class Robot: public IterativeRobot {
public:
	/** The Talon we want to motion profile. */
	TalonSRX _talon;

	/** some example logic on how one can manage an MP */
	MotionProfileExample _example;

	/** joystick for testing */
	HotJoystick _joy;

	bool aButton;
	bool aButtonOld = false;
	bool bButton;
	bool bButtonOld = false;
	double rotations = 0.0;
	double velocityRPM;
	double targPos;
	double targVel;
	double topBufferCnt;
	double topBufferRem;
	double btmBufferCnt;
	bool isLast;
	MotionProfileStatus profileStatus;

	Robot() :
			_talon(3), _example(_talon), _joy(0) {
	}

	/** run once after booting/enter-disable */
	void DisabledInit() {
		_talon.ConfigSelectedFeedbackSensor(FeedbackDevice::CTRE_MagEncoder_Relative, 0, 10);
		_talon.SetSensorPhase(true);
		_talon.SetNeutralMode(Brake);
		_talon.ConfigNeutralDeadband(0.05, 10);
		_talon.Config_kF(0, 0.0515, 10);
		_talon.Config_kP(0, 0.0004, 10);
		_talon.Config_kI(0, 0.000001, 10);
		_talon.Config_kD(0, 0.0, 10);

		_talon.ConfigMotionProfileTrajectoryPeriod(10, 10); //Our profile uses 10 ms timing
		/* status 10 provides the trajectory target for motion profile AND motion magic */
		_talon.SetStatusFramePeriod(StatusFrameEnhanced::Status_10_MotionMagic, 10, 10);
	}
	/**  function is called periodically during operator control */
	void TeleopPeriodic() {
		/* get buttons */
		aButton = _joy.ButtonA();
		bButton = _joy.ButtonB();

		/* call this periodically, and catch the output.  Only apply it if user wants to run MP. */
		_example.control();
		_example.PeriodicTask();

		/* display variables about motion profile in Talon */
		_talon.GetMotionProfileStatus(profileStatus);
		topBufferCnt = profileStatus.topBufferCnt;
		topBufferRem = profileStatus.topBufferRem;
		btmBufferCnt = profileStatus.btmBufferCnt;
		isLast = profileStatus.isLast;
		targPos = (_talon.GetActiveTrajectoryPosition() / 4096.0);          /* Convert to Revolutions */
		targVel = (_talon.GetActiveTrajectoryVelocity() * (600.0/4096.0));  /* Convert to RPM */

		if (aButton == false) { /* Check aButton to enter Motion Profile Mode */
			/* If it's not being pressed, just turn off motor. */

			_talon.Set(ControlMode::PercentOutput, 0.0);

			_example.reset();
		} else {
			/* ButtonA is held down so switch to motion profile control mode => This is done in MotionProfileControl.
			 * When we transition from no-press to press,
			 * pass a "true" once to MotionProfileControl.
			 */

			SetValueMotionProfile setOutput = _example.getSetValue();

			_talon.Set(ControlMode::MotionProfile, setOutput);

			/* if bButton is pressed and was not pressed last time,
			 * In other words we just detected the on-press event.
			 * This will signal the robot to start a MP */
			if ((bButton == true) && (bButtonOld == false)) {
				/* user just tapped bButton */

				//------------ We could start an MP if MP isn't already running ------------//
				_example.start();
			}
		}

		/* save buttons states for on-press detection */
		aButtonOld = aButton;
		bButtonOld = bButton;

		/* Writes variables to Dashboard */
		rotations = (_talon.GetSelectedSensorPosition(0) / 4096.0);
		velocityRPM = (_talon.GetSelectedSensorVelocity(0) * (600.0/4096.0));
		DashboardOutput();
	}



	void DashboardOutput() {
		/* Writes variables to Dashboard */
		SmartDashboard::PutBoolean("ButtonA", aButton);
		SmartDashboard::PutBoolean("ButtonB", bButton);
		SmartDashboard::PutBoolean("isLast", isLast);
		SmartDashboard::PutNumber("MotorOutputPct", _talon.GetMotorOutputPercent());
		SmartDashboard::PutNumber("rotationsRevs", rotations);
		SmartDashboard::PutNumber("velocityRPM", velocityRPM);
		SmartDashboard::PutNumber("topBufferCnt", topBufferCnt);
		SmartDashboard::PutNumber("topBufferRem", topBufferRem);
		SmartDashboard::PutNumber("btmBufferCnt", btmBufferCnt);
		SmartDashboard::PutNumber("targPos", targPos);
		SmartDashboard::PutNumber("targVel", targVel);
	}


	/**  function is called periodically during disable */
	void DisabledPeriodic() {
		/* it's generally a good idea to put motor controllers back
		 * into a known state when robot is disabled.  That way when you
		 * enable the robot doesn't just continue doing what it was doing before.
		 * BUT if that's what the application/testing requires than modify this accordingly */
		_talon.Set(ControlMode::PercentOutput, 0);
		/* clear our buffer and put everything into a known state */
		_example.reset();
	}
};

START_ROBOT_CLASS(Robot)
