#include "WPILib.h"
#include "ctre/Phoenix.h"
#include "stdlib.h"
#include <chrono>

/* Exercise 07 uses the PigeonIMU and the DIO(4) on the Sweet Bench
 * The goal of this exercise is to create a position estimator using the measured accelerations
 * from the Pigeon IMU and integrating to find velocity and position.
 *
 * Functional Requirements:
 * 1)  All functionality shall be done in TeleOp mode.
 * 2)  The x, y and z accelerations shall be continuously measured in in/sec^2
 * 3)  Using the accelerations in all 3 axes, the x, y, and z velocities shall be calculated in in/sec
 * 4)  Using the velocities in all 3 axes, the x, y, and z positions shall be calculated in inches
 * 5)  Pressing the DIO pushbutton will effectively "zero" the Pigeon and reset the velocities and positions.
 *
 */

class benchTest: public IterativeRobot {
private:
	PigeonIMU* m_pigey;
	DigitalInput* m_SwitchDIO4;
	TalonSRX* m_CANmotor4;

	bool switchSignal;
	bool switchSignalOld = false;
	double zeroAccel[3];
	double currentAccel[3];
	double velocity[3] = {0.0, 0.0, 0.0};
	double position[3] = {0.0, 0.0, 0.0};
	int16_t accelerometer[3];
	double timeDelta = 0.0; /* milliseconds */
	std::chrono::time_point<std::chrono::high_resolution_clock> timeNow = std::chrono::high_resolution_clock::now();
	std::chrono::time_point<std::chrono::high_resolution_clock> timeLast = std::chrono::high_resolution_clock::now();

public:
	benchTest() {
        m_CANmotor4 = new TalonSRX(4);
		m_pigey = new PigeonIMU(m_CANmotor4);  /* Pigeon installed on CANTalon(4) */
		m_SwitchDIO4 = new DigitalInput(4);
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

		 /* Read input from DIO4 - pushbutton switch */
		 switchSignal = !m_SwitchDIO4->Get();

		 /* Read the Pigeon IMU for Acceleration */
		 m_pigey->GetBiasedAccelerometer(accelerometer);

		 /* Check to see if switch is has been pressed to trigger accelerometer reset */
		 /* Note:  1 unit of acceleration from the Pigeon = (1/42.43521) in/sec^2 of acceleration */
		 if ((switchSignal == true) && (switchSignalOld == false)) {
			 for (int i = 0; i < 3; i++) {
				 zeroAccel[i] = (double)accelerometer[i] / 42.43521;
				 velocity[i] = 0.0;
				 position[i] = 0.0;
			 }
		 }

		 /* Measure the acceleration and compensate for zeroing */
		 currentAccel[0] = ((double)accelerometer[0] / 42.43521) - zeroAccel[0];
		 currentAccel[1] = ((double)accelerometer[1] / 42.43521) - zeroAccel[1];
		 currentAccel[2] = ((double)accelerometer[2] / 42.43521) - zeroAccel[2];

		 /* Read clock for loop time. */
		 timeNow = std::chrono::high_resolution_clock::now();
		 timeDelta = std::chrono::duration_cast<std::chrono::milliseconds>(timeNow-timeLast).count();
		 timeLast = timeNow;

		 /* Integrate to find velocity and position for each axis of acceleration */
		 for (int i = 0; i < 3; i++) {
			 velocity[i] += currentAccel[i] * (timeDelta/1000.0);
			 position[i] += velocity[i] * (timeDelta/1000.0);
		 }

		 /* Remember the switch state for next loop */
		 switchSignalOld = switchSignal;

		 /* Output to dashboard */
		 DashboardOutput();
	}

	void DashboardOutput() {
		/* Writes variables to Dashboard */
		SmartDashboard::PutNumber("timeDelta", timeDelta);
		SmartDashboard::PutNumber("raw x accel", (double)accelerometer[0] / 42.43521);
		SmartDashboard::PutNumber("raw y accel", (double)accelerometer[1] / 42.43521);
		SmartDashboard::PutNumber("raw z accel", (double)accelerometer[2] / 42.43521);
		SmartDashboard::PutNumber("zeroed x accel", currentAccel[0]);
		SmartDashboard::PutNumber("zeroed y accel", currentAccel[1]);
		SmartDashboard::PutNumber("zeroed z accel", currentAccel[2]);
		SmartDashboard::PutNumber("velocity x", velocity[0]);
		SmartDashboard::PutNumber("velocity y", velocity[1]);
		SmartDashboard::PutNumber("velocity z", velocity[2]);
		SmartDashboard::PutNumber("position x", position[0]);
		SmartDashboard::PutNumber("position y", position[1]);
		SmartDashboard::PutNumber("position z", position[2]);
	}

	void TestPeriodic() {
	}

};

START_ROBOT_CLASS(benchTest)
