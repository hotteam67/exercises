#include "WPILib.h"
#include "RobotUtils/HotJoystick.h"
#include "ctre/Phoenix.h"
#include <chrono>

/* Exercise 03 uses the Potentiometer (AnalogIn0), Pushbutton Switch (DIO4),
 * light sensor (DIO5) and motor 1 (CANTalon1) on the Sweet Bench.
 * It is also necessary to use the <chrono> library to precisely measure the
 * time period of the pulses detected from the light sensor.
 *
 * It should be noted that the wheel on top of motor 1 (which is the target of
 * the light sensor) has two openings.  Therefore, the light sensor should see
 * a transition from FALSE to TRUE two times per revolution.
 *
 * Functional Requirements:
 * 1)  All functionality shall be done in TeleOp mode.
 * 2)  Use the pushbutton switch (DIO4) to select the state of the motor.  This state indicates
 *     OFF, FORWARD or REVERSE.
 * 3)  The motor state shall start in the safe state, OFF.
 * 4)  The potentiometer (AnalogIn0) shall control the speed of motor 1.
 * 5)  The motor speed command shall range from 0 (off) to 1 (100%) never crossing either limit.
 * 6)  The light sensor shall be used to calculate the time between false->true transitions observed.
 * 7)  The time between transitions shall be used to estimate the motor speed in rev/min.  NOTE: This
 *     calculation can be easily corrupted if the motor is spinning fast.  This is because loop times
 *     for the RoboRIO are around 20 ms.  In can be reasoned that the fastest that can be "accurately"
 *     measured is if the light sensor changes state every loop - meaning that all 4 states of the
 *     target wheel are observed each rotation.  This means that rotations that finish in less
 *     than (20 ms * 4), or 80 ms may be corrupted.  1 rotation every 80ms equates to 750 rpm.
 * 8)  Output time between transitions, estimated speed (rpm), motor state and commanded speed to the
 *     dashboard.
 *
 */

class benchTest: public IterativeRobot {
private:

	TalonSRX* m_CANmotor1;
	AnalogInput* m_Analog0;
	DigitalInput* m_SwitchDIO4;
	DigitalInput* m_LightSensor;

	bool switchSignal;
	bool switchSignalOld = false;
	bool lightSignal;
	bool lightSignalOld;
	float pot1;
	float spdCmd;
	std::chrono::time_point<std::chrono::high_resolution_clock> timeNow = std::chrono::high_resolution_clock::now();
	std::chrono::time_point<std::chrono::high_resolution_clock> timeLast = std::chrono::high_resolution_clock::now();
	double timeDelta = 0.0; /* milliseconds */
	double speedRPM = 0.0; /* motor speed calculation in rpm */

	int motorState = 0;
	      /* motorState = 0 : motor OFF
	       * motorState = 1 : FORWARD
	       * motorState = 2 : REVERSE
	       */


public:
	benchTest() {
		m_CANmotor1 = new TalonSRX(1);
		m_Analog0 = new AnalogInput(0);  /* Analog Input Channel 0 */
		m_SwitchDIO4 = new DigitalInput(4);
		m_LightSensor = new DigitalInput(5);
		lightSignalOld = m_LightSensor->Get();

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

		 /* Read input from DIO5 - light sensor */
		 lightSignal = m_LightSensor->Get();;

		 /* Read potentiometer inputs from Analog0 */
		 pot1 = m_Analog0->GetValue();
		 spdCmd = (pot1 - 15.0) / 3900.0;
		 if (spdCmd < 0.0) {
			 spdCmd = 0.0;
		 } else if (spdCmd > 1.0) {
			 spdCmd = 1.0;
		 }

		 /* Read clock for loop time. */
		 timeNow = std::chrono::high_resolution_clock::now();
		 timeDelta = std::chrono::duration_cast<std::chrono::milliseconds>(timeNow-timeLast).count();


		 /* Look for switchSignal (DIO4) to transition from false to true */
		 if ((switchSignal == true) && (switchSignalOld == false)) {
			 if (motorState == 2) {
				 motorState = 0;
			 } else {
				 motorState++;
			 }

		 }

		 /* Look for lightSignal (DIO5) to transition from false to true */
		 /* Calculate speed based on knowledge that the variable timeDelta will measure
		  * the number of milliseconds it takes for the motor to spin 1/2 revolution.
		  * Therefore, the speed in revs/min = 30,000/timeDelta.  Note - this calculation can
		  * really be corrupted if motor speeds are fast.  Knowing the loop rate of the
		  * RoboRIO is 20 ms, the fasted possible speed to detect is 750 rpm.  Anything over
		  * that will be corrupted.
		  */
		 if ((lightSignal == true) && (lightSignalOld == false)) {
			 timeNow = std::chrono::high_resolution_clock::now();
			 timeDelta = std::chrono::duration_cast<std::chrono::milliseconds>(timeNow-timeLast).count();
			 timeLast = timeNow;
			 if (timeDelta > 0.0) {
				 speedRPM = (30000.0 / timeDelta);
			 } else {
				 speedRPM = 0.0;
			 }
		 }

		 /* Send speed command to motor based on motorState variable */
		 if (motorState == 0) {
			 m_CANmotor1->Set(ControlMode::PercentOutput, 0.0);
		 } else if (motorState == 1 ){
			 m_CANmotor1->Set(ControlMode::PercentOutput, spdCmd);
		 } else {
			 m_CANmotor1->Set(ControlMode::PercentOutput, -spdCmd);
		 }

		 /* Preserve knowledge of previous loop button state */
		 switchSignalOld = switchSignal;
		 lightSignalOld = lightSignal;

		 DashboardOutput();
	}

	void DashboardOutput() {
		/* Writes variables to Dashboard */
		SmartDashboard::PutBoolean("switchSignal", switchSignal);
		SmartDashboard::PutBoolean("lightSignal", lightSignal);
        SmartDashboard::PutNumber("timeDelta", timeDelta);
        SmartDashboard::PutNumber("speedRPM", speedRPM);
		SmartDashboard::PutNumber("motorState", motorState);
		SmartDashboard::PutNumber("SpdCmd", spdCmd);
		SmartDashboard::PutNumber("Pot1", pot1);
	}


	void TestPeriodic() {
	}
};

START_ROBOT_CLASS(benchTest)
