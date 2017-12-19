#include "WPILib.h"
#include "RobotUtils/RobotUtils.h"
#include "ctrlib/CANTalon.h"
#include "ctrlib/PigeonImu.h"

class benchTest: public HotBot {
private:

	HotJoystick* m_driver;

	CANTalon* m_CANmotor1;
	CANTalon* m_CANmotor2;
	CANTalon* m_CANmotor3;
	CANTalon* m_CANmotor4;
	PigeonImu * m_pigey;
	AnalogInput* m_Analog0;
	Relay* m_LED_Ring;

	bool aButton;
	bool aButtonOld = false;
	bool bButton;
	bool bButtonOld = false;
	bool motorStTransInProcess = false;
	bool ledTransInProcess = false;

	float joystickRaw;
	float joystickMod;
	float spdCmd;
	double angle[3];
	float pot1;
	float pot2;

	bool ledSelect = false;
	     /* false: LED off
	        true:  LED on */

	int motorSelect = 4;
	     /* 0: CANTalon1
		    1: CANTalon2
			2: CANTalon3
			3: CANTalon4
			4: No Motor Selected */

public:
	benchTest() {
		m_driver = new HotJoystick(0);

		m_CANmotor1 = new CANTalon(1);
		m_CANmotor2 = new CANTalon(2);
		m_CANmotor3 = new CANTalon(3);
		m_CANmotor4 = new CANTalon(4);
		m_pigey = new PigeonImu(m_CANmotor4);  /* Pigeon installed on CANTalon(4) */
		m_Analog0 = new AnalogInput(0);  /* Analog Input Channel 0 */
		m_LED_Ring = new Relay(1);

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
		 bButton = m_driver->ButtonB();
		 joystickRaw = m_driver->AxisLX();  /* Uses Left Stick, X axis */

		 /* Read potentiometer inputs from Analog0 and CANmotor2 */
		 pot1 = m_Analog0->GetValue();
		 pot2 = m_CANmotor2->GetAnalogIn();

		 /* Write Relay for LED ring */
		 m_LED_Ring->Set(Relay::Value::kOff);

		 /* Deadband input, determine joystickMod */
		 if(joystickRaw <= 0.2 && joystickRaw >= -0.2){
			 joystickMod = 0.0;
		 }else if(joystickRaw > 0.2){
			 joystickMod = (joystickRaw - 0.2) * 1.25;
		 }else{
			 joystickMod = (joystickRaw + 0.2) * 1.25;
		 }
		 spdCmd = joystickMod;

		 /* Process button presses */
		 /* Look for aButton to transition from false to true */
		 if ((aButton == true) && (aButtonOld == false)) {
			 motorStTransInProcess = true;
		 } else {
			 motorStTransInProcess = false;
		 }

		 /* Look for bButton to transition from false to true */
		 if ((bButton == true) && (bButtonOld == false)) {
			  ledTransInProcess = true;
		 } else {
		 	  ledTransInProcess = false;
		 }

		 /* Use motorSelect to define speed commands to motors */
		 if (motorStTransInProcess == true) {
			 if (motorSelect < 4) {
				 motorSelect++;
			 } else {
				 motorSelect = 0;
			 }
		 }

		 /* Change LED state when ledTransInProccess == true */
		 if (ledTransInProcess == true) {
			 ledSelect = !ledSelect;
		 }

		 if (ledSelect == true) {
			 m_LED_Ring->Set(Relay::Value::kForward);
		 } else {
			 m_LED_Ring->Set(Relay::Value::kOff);
		 }

		 /* Command speeds to motor controllers */
		 switch (motorSelect) {
		 case 0:
			 m_CANmotor1->Set(spdCmd);
			 m_CANmotor2->Set(0.0);
			 m_CANmotor3->Set(0.0);
			 m_CANmotor4->Set(0.0);
			 break;
		 case 1:
			 m_CANmotor1->Set(0.0);
			 m_CANmotor2->Set(spdCmd);
			 m_CANmotor3->Set(0.0);
			 m_CANmotor4->Set(0.0);
			 break;
		 case 2:
			 m_CANmotor1->Set(0.0);
			 m_CANmotor2->Set(0.0);
			 m_CANmotor3->Set(spdCmd);
			 m_CANmotor4->Set(0.0);
			 break;
		 case 3:
			 m_CANmotor1->Set(0.0);
			 m_CANmotor2->Set(0.0);
			 m_CANmotor3->Set(0.0);
			 m_CANmotor4->Set(spdCmd);
			 break;
		 case 4:
			 m_CANmotor1->Set(0.0);
			 m_CANmotor2->Set(0.0);
			 m_CANmotor3->Set(0.0);
			 m_CANmotor4->Set(0.0);
			 break;
		 }

		 /* Preserve knowledge of previous loop button state */
		 aButtonOld = aButton;
		 bButtonOld = bButton;

		 /* Read the Pigeon IMU for Yaw, Pitch, Roll */
		 m_pigey->GetYawPitchRoll(angle);

		 DashboardOutput();
	}

	void DashboardOutput() {
		/* Writes variables to Dashboard */
		SmartDashboard::PutBoolean("ButtonA", aButton);
		SmartDashboard::PutBoolean("ButtonB", bButton);
		SmartDashboard::PutNumber("joystickRaw", joystickRaw);
		SmartDashboard::PutNumber("joystickMod", joystickMod);
		SmartDashboard::PutNumber("motorSelect", motorSelect);
		SmartDashboard::PutBoolean("ledSelect", ledSelect);
		SmartDashboard::PutNumber("SpdCmd", spdCmd);
		SmartDashboard::PutNumber("Angle0 (Yaw)", angle[0]);
		SmartDashboard::PutNumber("Angle1 (Pitch)", angle[1]);
		SmartDashboard::PutNumber("Angle2 (Roll)", angle[2]);
		SmartDashboard::PutNumber("Pot1", pot1);
		SmartDashboard::PutNumber("Pot2", pot2);
	}


	void TestPeriodic() {
	}

};

START_ROBOT_CLASS(benchTest)
