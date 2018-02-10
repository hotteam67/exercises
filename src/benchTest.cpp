#include "WPILib.h"
#include "RobotUtils/HotJoystick.h"
#include "ctre/Phoenix.h"
#include <chrono>
#include <iostream>



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

using namespace std;

enum State {
	Reset,
	Prep,
	Test
};

class benchTest: public IterativeRobot {
private:

	TalonSRX* Motor3;
	TalonSRX* Motor1;
	TalonSRX* Motor2;
	Encoder* Encode;

	AnalogInput* RawUltra;
	DigitalInput* m_SwitchDIO4;
	PigeonIMU * _pidgey;
	TalonSRX* PigionTalon;
	HotJoystick* m_driver;

	double current;
	double Xaxis[2];
	double Yaxis[3];
	double Cal[2];
	double CalB[50];
	double CalAv;
	int16_t ba_xyz[3];
	bool button;
	bool Runonce;
	int TestCase=0;
	State state;
	int MotorTest;
	int count=0;
	double encoder;
	double NormalValue;




	int motorState = 0;
	/* motorState = 0 : motor OFF
	 * motorState = 1 : FORWARD
	 * motorState = 2 : REVERSE
	 */
	BuiltInAccelerometer accel;


public:
	benchTest() {
		Motor3 = new TalonSRX(3);
		Motor1 = new TalonSRX(1);
		Motor2 = new TalonSRX(2);
		PigionTalon = new TalonSRX(1);
		RawUltra = new AnalogInput(1);  /* Analog Input Channel 0 */
		m_SwitchDIO4 = new DigitalInput(4);
		_pidgey = new PigeonIMU(PigionTalon); /* Pigeon is ribbon cabled to the specified CANTalon. */
		m_driver = new HotJoystick(0);



	}
	void RobotInit() {

	}


	void AutonomousInit() {

	}

	void AutonomousPeriodic() {

	}

	void TeleopInit() {

		Cal[0] = 0;

	}

	void TeleopPeriodic() {






		Motor3->Set(ControlMode::PercentOutput, -0.2);
		current = Motor1->GetOutputCurrent();
		SmartDashboard::PutNumber("Current",current);



		if(current <= 0.750){
			cout<<"Motor #3 Passed"<<endl;
		}else{
			cout<<"Warning: Fault in motor #3 "<<endl;
		}

		/**
		_pidgey->GetBiasedAccelerometer(ba_xyz);

		if(m_driver->ButtonB()){

		for(int i=0; i<50; i++){
			CalB[i] = ba_xyz[0];
		}
		for(int i=0; i<50; i++){
			CalAv = CalAv+CalB[i];
		}
		Cal[0] = CalAv/50;
		}
		Xaxis[1] = ((accel.GetX())-Cal[1])*386.0885826772;
		Xaxis[0] = ((ba_xyz[0])-Cal[0])/42.43521;




		cout<<"Cool it worked"<<endl;
		 */



		SmartDashboard::PutNumber("Current2",CalAv);


		void DashboardOutput();

		/*
		SmartDashboard::PutNumber("acelXRIO",Xaxis[1]);
		SmartDashboard::PutNumber("acelXPIDGY",Xaxis[0]);
		SmartDashboard::PutNumber("CalXPIDGY",Cal[0]);
		 */


	}

	void DashboardOutput() {



	}



	void TestInit(){

		state = Reset;
		TestCase = 0;
	}

	void EncReset(){

	}

	void TestPeriodic() {


		switch(state) {
		case Reset:

			Motor1->Set(ControlMode::PercentOutput, 0);
			Motor3->Set(ControlMode::PercentOutput, 0);


			count = count+1;
			if(count>150){
				count = 0;
				state = Prep;
				TestCase = TestCase+1;
				EncReset();
			}
			break;
		case Prep:

			if(TestCase == 1) Motor1->Set(ControlMode::PercentOutput, -0.2);
			if(TestCase == 3) Motor3->Set(ControlMode::PercentOutput, -0.2);

			count = count+1;
			if(count>100){
				count = 0;
				state = Test;
			}
			break;
		case Test:

			switch(TestCase){
			case 1:
			{
				//Read Motor current here
				current = Motor1->GetOutputCurrent();
				//Read encoder value here
				encoder = Motor1->GetSelectedSensorPosition(0);
				//Normalvalue is just the number put for safe operating current and is a placeholder
				NormalValue = 0.750;
				if(current == 0 && encoder <= 100){
					cout << "Warning: Motor1 Power Disconnect likely"<<endl;
				}else if(current >= NormalValue+50 && encoder <= 100){
					cout << "Warning: Probable Motor Stall Detected In Motor1 Disconnect Motor1 Immediately"<<endl;
				}else if(current <= NormalValue+50 && current >= NormalValue-50 && encoder <=100){
					cout << "Motor1 Passed Test"<<endl;
				}else if(current <= NormalValue+50 && current >= NormalValue-50 && encoder >=100){
					cout << "Motor1 Passed Test"<<endl;
				}else{
					//This should never happen but if it does I want to know
					cout << "Warning:Testing Error Motor# Did Not Meet Any Testing Criteria"<<endl;
				}
				state = Reset;
				break;
			}
			case 2:
			{
				state = Reset;
				break;
			}
			break;
			case 3:
			{
				//Read Motor current here
				current = Motor3->GetOutputCurrent();
				//Read encoder value here
				encoder = Motor3->GetSelectedSensorPosition(0);
				//Normalvalue is just the number put for safe operating current and is a placeholder
				NormalValue = 0.750;
				if(current == 0 && encoder <= 100){
					cout << "Warning: Motor3 Power Disconnect likely"<<endl;
				}else if(current >= NormalValue+50 && encoder <= 100){
					cout << "Warning: Probable Motor Stall Detected In Motor3 Disconnect Motor3 Immediately"<<endl;
				}else if(current <= NormalValue+50 && current >= NormalValue-50 && encoder <=100){
					cout << "Warning: Encoder Error Detected In Motor3"<<endl;
				}else if(current <= NormalValue+50 && current >= NormalValue-50 && encoder >=100){
					cout << "Motor3 Passed Test"<<endl;
				}else{
					//This should never happen but if it does I want to know
					cout << "Warning:Testing Error Motor# Did Not Meet Any Testing Criteria"<<endl;
				}

				state = Reset;
				break;
			}


		}

		}
	}


	//for opmode

};

START_ROBOT_CLASS(benchTest)
