/**
 * Example demonstrating the velocity closed-loop servo.
 * Tested with Logitech F350 USB Gamepad inserted into Driver Station]
 *
 * Be sure to select the correct feedback sensor using SetFeedbackDevice() below.
 *
 * After deploying/debugging this to your RIO, first use the left Y-stick
 * to throttle the Talon manually.  This will confirm your hardware setup.
 * Be sure to confirm that when the Talon is driving forward (green) the
 * position sensor is moving in a positive direction.  If this is not the cause
 * flip the boolean input to the SetSensorDirection() call below.
 *
 * Once you've ensured your feedback device is in-phase with the motor,
 * use the button shortcuts to servo to target velocity.
 *
 * Tweak the PID gains accordingly.
 */
#include "WPILib.h"
#include "RobotUtils/HotJoystick.h"
#include "ctre/Phoenix.h"
#include "Constants.h"

class Robot: public IterativeRobot {
private:
	TalonSRX * _talon = new TalonSRX(3);
	HotJoystick * m_joy = new HotJoystick(0);
	std::string _sb;
	int _loops = 0;

	void RobotInit() {
        /* first choose the sensor */
		_talon->ConfigSelectedFeedbackSensor(FeedbackDevice::CTRE_MagEncoder_Relative, 0, 10);
		_talon->SetSensorPhase(true);

		/* set the peak and nominal outputs, 12V means full */
		_talon->ConfigNominalOutputForward(0, kTimeoutMs);
		_talon->ConfigNominalOutputReverse(0, kTimeoutMs);
		_talon->ConfigPeakOutputForward(1, kTimeoutMs);
		_talon->ConfigPeakOutputReverse(-1, kTimeoutMs);
		/* set closed loop gains in slot0 */
		_talon->Config_kF(kPIDLoopIdx, 0.0509, kTimeoutMs); //0.1097
		_talon->Config_kP(kPIDLoopIdx, 0.0001, kTimeoutMs);
		_talon->Config_kI(kPIDLoopIdx, 0.0001, kTimeoutMs);
		_talon->Config_kD(kPIDLoopIdx, 0.0, kTimeoutMs);
	}
	/**
	 * This function is called periodically during operator control
	 */
	void TeleopPeriodic() {
		/* get gamepad axis */
		double leftYstick = m_joy->AxisLY();
		double motorOutput = _talon->GetMotorOutputPercent();
		/* prepare line to print */
		_sb.append("\tout:");
		_sb.append(std::to_string(motorOutput));
		_sb.append("\tspd:");
		_sb.append(std::to_string(_talon->GetSelectedSensorVelocity(kPIDLoopIdx)));

		SmartDashboard::PutNumber("SensorVelocity", (_talon->GetSelectedSensorVelocity(kPIDLoopIdx)/4096.0 * 600.0));

		/* while button1 is held down, closed-loop on target velocity */
		if (m_joy->ButtonA()) {
        	/* Speed mode */
			/* 1500 RPM * 4096 units/rev / 600 100ms/min in either direction: velocity control is units/100ms */
			double targetSpeed = 300.0 * 4096 / 600;
        	_talon->Set(ControlMode::Velocity, targetSpeed); /* 1500 RPM in either direction */

			/* append more signals to print when in speed mode. */
			_sb.append("\terrNative:");
			_sb.append(std::to_string(_talon->GetClosedLoopError(kPIDLoopIdx)));
			_sb.append("\ttrg:");
			_sb.append(std::to_string(targetSpeed));
        } else {
			/* Percent voltage mode */
			_talon->Set(ControlMode::PercentOutput, leftYstick);
		}
		/* print every ten loops, printing too much too fast is generally bad for performance */
		if (++_loops >= 10) {
			_loops = 0;
			printf("%s\n",_sb.c_str());
		}
		_sb.clear();
	}
};

START_ROBOT_CLASS(Robot)
