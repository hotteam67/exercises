#include "WPILib.h"
#include "RobotUtils/RobotUtils.h"
#include "ctrlib/CANTalon.h"
#include "ctrlib/PigeonImu.h"

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
	DigitalInput *m_SwitchDIO4;

	bool aButton;
	bool aButtonOld = false;
	bool motorStTransInProcess = false;
	bool ledTransInProcess = false;
	bool switchSignal;
	bool switchSignalOld = false;

	float joystickRaw;
	float joystickMod;
	float spdCmd;
	double angle[3];
	float pot1;
	float pot2;

	bool ledSelect = false;
	     /* false: LED off
	        true:  LED on */

	int motorSelect = 0;
	     /* 0: Off (no motor)
		    1: CANTalon2
			2: CANTalon3
			3: CANTalon4 */

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

		 /* Read inputs from Joystick */
		 aButton = m_driver->ButtonA();
		 joystickRaw = m_driver->AxisLX();  /* Uses Left Stick, X axis */

		 /* Read input from DIO4 - pushbutton switch */
		 switchSignal = !m_SwitchDIO4->Get();

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
		 if ((switchSignal == true) && (switchSignalOld == false)) {
			  ledTransInProcess = true;
		 } else {
		 	  ledTransInProcess = false;
		 }

		 /* Use motorSelect to define speed commands to motors */
		 if (motorStTransInProcess == true) {
			 if (motorSelect < 3) {
				 motorSelect++;
			 } else {
				 motorSelect = 0;
			 }
		 }

		 /* Change LED state when ledTransInProccess == true */
		 if (ledTransInProcess == true) {
			 ledSelect = !ledSelect;
		 }

		 /* Send LED command to relay */
		 if (ledSelect == true) {
			 m_LED_Ring->Set(Relay::Value::kForward);
		 } else {
			 m_LED_Ring->Set(Relay::Value::kOff);
		 }

		 /* Command speeds to motor controllers */
		 switch (motorSelect) {
		 case 0:
			 m_CANmotor2->Set(0.0);
			 m_CANmotor3->Set(0.0);
			 m_CANmotor4->Set(0.0);
			 break;
		 case 1:
			 m_CANmotor2->Set(spdCmd);
			 m_CANmotor3->Set(0.0);
			 m_CANmotor4->Set(0.0);
			 break;
		 case 2:
			 m_CANmotor2->Set(0.0);
			 m_CANmotor3->Set(spdCmd);
			 m_CANmotor4->Set(0.0);
			 break;
		 case 3:
			 m_CANmotor2->Set(0.0);
			 m_CANmotor3->Set(0.0);
			 m_CANmotor4->Set(spdCmd);
			 break;
		 }

		 /* Preserve knowledge of previous loop button state */
		 aButtonOld = aButton;
		 switchSignalOld = switchSignal;

		 /* Read the Pigeon IMU for Yaw, Pitch, Roll */
		 m_pigey->GetYawPitchRoll(angle);

		 DashboardOutput();
	}

	void DashboardOutput() {
		/* Writes variables to Dashboard */
		SmartDashboard::PutBoolean("ButtonA", aButton);
		SmartDashboard::PutBoolean("switchSignal", switchSignal);
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
