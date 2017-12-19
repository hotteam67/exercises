#include "WPILib.h"
#include "RobotUtils/RobotUtils.h"
#include "ctrlib/CANTalon.h"
#include "ctrlib/PigeonImu.h"
#include "stdlib.h"

/* Exercise 02 is to use the PigeonIMU, DIO, analog potentiometer and the LED light ring on the Sweet Bench
 * The goal of this exercise is to create a motion detection algorithm that will light the LED
 * ring if the IMU moves in any direction.  The motion detection tolerance will be defined by the
 * potentiometer, and the Pigeon angles can be zeroed with the DIO pushbutton.
 *
 * Functional Requirements:
 * 1)  All functionality shall be done in TeleOp mode.
 * 2)  The yaw, pitch and roll of the Pigeon IMU shall be continuously monitored.
 * 3)  If the change in yaw, pitch or roll of the Pigeon moves outside a deadband (from
 *     the zero point) in any direction, an alarm (LED ring) shall be triggered.
 * 4)  The size of the deadband can be calibrated using the potentiometer with the full
 *     scale of the analog potentiometer representing a deadband of 90 degrees.
 * 5)  Pressing the DIO pushbutton will effectively "zero" the Pigeon.
 *
 */

class benchTest: public HotBot {
private:
	PigeonImu* m_pigey;
	Relay* m_LED_Ring;
	DigitalInput* m_SwitchDIO4;
	AnalogInput* m_Analog0;
	CANTalon* m_CANmotor4;

	bool switchSignal;
	bool switchSignalOld = false;
	bool alarmActive;
	double deadband;
	double angle[3];
	double zeroAngle[3] = {0.0, 0.0, 0.0};
	double angleIMU[3];  /* yaw, pitch and roll angles in degrees */

	float pot1;

public:
	benchTest() {
        m_CANmotor4 = new CANTalon(4);
		m_pigey = new PigeonImu(m_CANmotor4);  /* Pigeon installed on CANTalon(4) */
		m_Analog0 = new AnalogInput(0);  /* Analog Input Channel 0 */
		m_LED_Ring = new Relay(1);
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

		 /* Read potentiometer inputs from Analog0 and calculate deadband */
		 pot1 = m_Analog0->GetValue();  /* 0 - 3939 counts */
		 deadband = (pot1 / 3939) * 90;  /* Full range of Pot1 = 90 degrees */

		 /* Read the Pigeon IMU for Yaw, Pitch, Roll */
		 m_pigey->GetYawPitchRoll(angleIMU);

		 /* Check to see if switch is has been pressed to trigger gyro reset */
		 if ((switchSignal == true) && (switchSignalOld == false)) {
			 zeroAngle[0] = angleIMU[0];
			 zeroAngle[1] = angleIMU[1];
			 zeroAngle[2] = angleIMU[2];
		 }

		 /* Measure the rotation from the zeroAngle */
		 angle[0] = angleIMU[0] - zeroAngle[0];
         angle[1] = angleIMU[1] - zeroAngle[1];
         angle[2] = angleIMU[2] - zeroAngle[2];

         /* Determine if Pigeon has moved more than the deadband. */
         if ((abs(angle[0]) > deadband) ||
             (abs(angle[1]) > deadband) ||
			 (abs(angle[2]) > deadband)) {
        	 alarmActive = true;
         } else {
        	 alarmActive = false;
         }

		 /* Send LED command to relay */
         if (alarmActive) {
        	 m_LED_Ring->Set(Relay::Value::kForward);
         } else {
        	 m_LED_Ring->Set(Relay::Value::kOff);
         }

		 /* Remember the switch state for next loop */
		 switchSignalOld = switchSignal;

		 /* Output to dashboard */
		 DashboardOutput();
	}

	void DashboardOutput() {
		/* Writes variables to Dashboard */
		SmartDashboard::PutBoolean("switchSignal", switchSignal);
		SmartDashboard::PutBoolean("alarmActive", alarmActive);
		SmartDashboard::PutNumber("Angle0 (Yaw)", angle[0]);
		SmartDashboard::PutNumber("Angle1 (Pitch)", angle[1]);
		SmartDashboard::PutNumber("Angle2 (Roll)", angle[2]);
		SmartDashboard::PutNumber("Deadband", deadband);
	}

	void TestPeriodic() {
	}

};

START_ROBOT_CLASS(benchTest)
