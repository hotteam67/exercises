/**
 * This C++ FRC robot application is meant to demonstrate an example using the Motion Profile control mode
 * in Talon SRX.  The CANTalon class gives us the ability to buffer up trajectory points and execute them
 * as the roboRIO streams them into the Talon SRX.
 * 
 * There are many valid ways to use this feature and this example does not sufficiently demonstrate every possible
 * method.  Motion Profile streaming can be as complex as the developer needs it to be for advanced applications,
 * or it can be used in a simple fashion for fire-and-forget actions that require precise timing.
 * 
 * This application is an IterativeRobot project to demonstrate a minimal implementation not requiring the command 
 * framework, however these code excerpts could be moved into a command-based project.
 * 
 * 
 * Logitech Gamepad mapping, use left y axis to drive Talon normally.  
 * Press and hold top-left-shoulder-button5 to put Talon into motion profile control mode.
 * This will start sending Motion Profile to Talon while Talon is neutral. 
 * 
 * While holding top-left-shoulder-button5, tap top-right-shoulder-button6.
 * This will signal Talon to fire MP.  When MP is done, Talon will "hold" the last setpoint position
 * and wait for another button6 press to fire again.
 * 
 * Release button5 to allow OpenVoltage control with left y axis.
 */

#include "WPILib.h"
#include "MotionProfileExample.h"
#include "ctre/Phoenix.h"
#include "Constants.h"
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
	double velNative;
	double posNative;
	double targPos;
	double targVel;
	double topBufferCnt;
	double topBufferRem;
	double btmBufferCnt;
	bool isLast;
	MotionProfileStatus profileStatus;

	Robot() :
			_talon(Constants::kTalonID), _example(_talon), _joy(0) {
	}

	/** run once after booting/enter-disable */
	void DisabledInit() {
		_talon.ConfigSelectedFeedbackSensor(FeedbackDevice::CTRE_MagEncoder_Relative, 0,
				kTimeoutMs);
		_talon.SetSensorPhase(true);
		_talon.SetNeutralMode(Brake);
		_talon.ConfigNeutralDeadband(Constants::kNeutralDeadbandPercent * 0.01,
				Constants::kTimeoutMs);

		_talon.Config_kF(0, 0.0515, kTimeoutMs);
		_talon.Config_kP(0, 0.0004, kTimeoutMs);
		_talon.Config_kI(0, 0.000001, kTimeoutMs);
		_talon.Config_kD(0, 0.0, kTimeoutMs);

		_talon.ConfigMotionProfileTrajectoryPeriod(10, Constants::kTimeoutMs); //Our profile uses 10 ms timing
		/* status 10 provides the trajectory target for motion profile AND motion magic */
		_talon.SetStatusFramePeriod(StatusFrameEnhanced::Status_10_MotionMagic,
				10, Constants::kTimeoutMs);
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
		velNative = _talon.GetSelectedSensorVelocity(0);
		posNative = _talon.GetSelectedSensorPosition(0);
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
