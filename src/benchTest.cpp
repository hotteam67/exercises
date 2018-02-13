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


	TalonSRX* MotorR1;
	TalonSRX* MotorR2;
	TalonSRX* MotorR3;
	TalonSRX* MotorL1;
	TalonSRX* MotorL2;
	TalonSRX* MotorL3;
	Encoder* Encode;
	TalonSRX* LeftEnc;
	TalonSRX* RightEnc;

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
	double encoder=0;

	double NormalValue;
	double Range;




	int motorState = 0;
	/* motorState = 0 : motor OFF
	 * motorState = 1 : FORWARD
	 * motorState = 2 : REVERSE
	 */
	BuiltInAccelerometer accel;


public:
	benchTest() {
		MotorR1 = new TalonSRX(2);
		MotorR2 = new TalonSRX(4);
		MotorR3 = new TalonSRX(3);
		MotorL1 = new TalonSRX(7);
		MotorL2 = new TalonSRX(5);
		MotorL3 = new TalonSRX(10);

		LeftEnc = new TalonSRX(7);
		RightEnc = new TalonSRX(2);
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






		//MotorR1->Set(ControlMode::PercentOutput, 1);
		MotorR2->Set(ControlMode::PercentOutput, 1);
		//MotorR3->Set(ControlMode::PercentOutput, 1);
		//MotorL1->Set(ControlMode::PercentOutput, 1);
		//MotorL2->Set(ControlMode::PercentOutput, 1);
		//MotorL3->Set(ControlMode::PercentOutput, 1);

		current = MotorL2->GetOutputCurrent();
		encoder = LeftEnc->GetSelectedSensorPosition(0);
		SmartDashboard::PutNumber("Current",current);
		SmartDashboard::PutNumber("LEncoder",encoder);



	}



	void TestInit(){

		state = Reset;
		TestCase = 0;
		for(int i=0;i<10;i++){
			cout<<" "<<endl;

		}
	}


	void MotorStop(){
		MotorR1->SetNeutralMode(Brake);
		MotorR2->SetNeutralMode(Brake);
		MotorR3->SetNeutralMode(Brake);
		MotorL1->SetNeutralMode(Brake);
		MotorL2->SetNeutralMode(Brake);
		MotorL3->SetNeutralMode(Brake);
	}

	void MotorCoast(){
		MotorR1->SetNeutralMode(Coast);
		MotorR2->SetNeutralMode(Coast);
		MotorR3->SetNeutralMode(Coast);
		MotorL1->SetNeutralMode(Coast);
		MotorL2->SetNeutralMode(Coast);
		MotorL3->SetNeutralMode(Coast);
	}
	void EncoderReset(){
		RightEnc->SetSelectedSensorPosition(0,0,0);
		LeftEnc->SetSelectedSensorPosition(0,0,0);
		encoder = 0;
	}

	void TestPeriodic() {


		switch(state) {
		case Reset:

			MotorStop();

			MotorR1->Set(ControlMode::PercentOutput, 0);
			MotorR2->Set(ControlMode::PercentOutput, 0);
			MotorR3->Set(ControlMode::PercentOutput, 0);
			MotorL1->Set(ControlMode::PercentOutput, 0);
			MotorL2->Set(ControlMode::PercentOutput, 0);
			MotorL3->Set(ControlMode::PercentOutput, 0);


			count = count+1;
			if(count>75){
				count = 0;
				state = Prep;
				TestCase = TestCase+1;
				SmartDashboard::PutNumber("Encoder",encoder);
				SmartDashboard::PutNumber("Current",current);
				EncoderReset();
			}
			break;
		case Prep:
			MotorCoast();
			if(TestCase == 1) MotorR1->Set(ControlMode::PercentOutput, 1);
			if(TestCase == 2) MotorL1->Set(ControlMode::PercentOutput, 1);
			if(TestCase == 3) MotorR2->Set(ControlMode::PercentOutput, 1);
			if(TestCase == 4) MotorL2->Set(ControlMode::PercentOutput, -1);
			if(TestCase == 5) MotorR3->Set(ControlMode::PercentOutput, -1);
			if(TestCase == 6) MotorL3->Set(ControlMode::PercentOutput, -1);

			count = count+1;
			if(count>175){
				count = 0;
				state = Test;
			}
			break;
		case Test:

			switch(TestCase){
			case 1:
			{
				//Test motor R1
				//Read Motor current here
				current = MotorR1->GetOutputCurrent();
				//Read encoder value here
				encoder = RightEnc->GetSelectedSensorPosition(0);
				//Normalvalue is just the number put for safe operating current and is a placeholder
				NormalValue = 16;
				Range = 0.5;
				if(current == 0 && encoder <= 100){
					cout << "Warning: Motor#R1 Power Disconnect likely"<<endl;
				}else if(current > NormalValue+Range && encoder <= 10){
					cout << "Warning: Probable Motor Stall Detected In Motor#R1 Disconnect Motor#R1 Immediately"<<endl;
				}else if(current < NormalValue+Range && current > NormalValue-Range && encoder <=10){
					cout << "Warning: Encoder Error Detected In Motor#R1"<<endl;
				}else if(current < NormalValue+Range && current > NormalValue-Range && encoder >=10){
					cout << "Motor#R1 Passed Test"<<endl;
				}else if(current == 0 && encoder > 10){
				cout<<"Warning: Current Read Error Motor#R1"<<endl;
				}else{
					//This should never happen but if it does I want to know
					cout << "Warning:Testing Error Motor#R1 Did Not Meet Any Testing Criteria"<<endl;
				}

				state = Reset;
				break;

			}
			case 2:
			{
				//Read Motor current here
				current = MotorL1->GetOutputCurrent();
				//Read encoder value here
				encoder = LeftEnc->GetSelectedSensorPosition(0);
				//Normalvalue is just the number put for safe operating current and is a placeholder
				NormalValue = 16;
				Range = 0.5;
				if(current == 0 && encoder <= 10){
					cout << "Warning: Motor#3 Power Disconnect likely"<<endl;
				}else if(current > NormalValue+Range && encoder <= 10){
					cout << "Warning: Probable Motor Stall Detected In Motor#L1 Disconnect Motor#L1 Immediately"<<endl;
				}else if(current < NormalValue+Range && current > NormalValue-Range && encoder <=10){
					cout << "Warning: Encoder Error Detected In Motor#L1"<<endl;
				}else if(current < NormalValue+Range && current > NormalValue-Range && encoder >=10){
					cout << "Motor#L1 Passed Test"<<endl;
				}else if(current == 0 && encoder >= 10){
				cout<<"Warning: Current Read Error Motor#L1"<<endl;

				}else{
					//This should never happen but if it does I want to know
					cout << "Warning:Testing Error Motor#L1 Did Not Meet Any Testing Criteria"<<endl;
				}

				state = Reset;
				break;
			}
			break;
			case 3:
			{
				//Read Motor current here
				//Read Motor current here
				current = MotorR2->GetOutputCurrent();
				//Read encoder value here
				encoder = RightEnc->GetSelectedSensorPosition(0);
				//Normalvalue is just the number put for safe operating current and is a placeholder
				NormalValue = 12;
				Range = 0.5;
				if(current == 0 && encoder <= 10){
					cout << "Warning: Motor#R2 Power Disconnect likely"<<endl;
				}else if(current > NormalValue+Range && encoder <= 10){
					cout << "Warning: Probable Motor Stall Detected In Motor#R2 Disconnect Motor#R2 Immediately"<<endl;
				}else if(current < NormalValue+Range && current > NormalValue-Range && encoder <=10){
					cout << "Warning: Encoder Error Detected In Motor#R2"<<endl;
				}else if(current < NormalValue+Range && current > NormalValue-Range && encoder >=10){
					cout << "Motor#R2 Passed Test"<<endl;
				}else if(current == 0 && encoder >= 10){
					cout<<"Warning: Current Read Error Motor#L1"<<endl;
				}else{
					//This should never happen but if it does I want to know
					cout << "Warning:Testing Error Motor#R2 Did Not Meet Any Testing Criteria"<<endl;
				}

				state = Reset;
				break;
			}
			case 4:
			{
				//Read Motor current here
				current = MotorL2->GetOutputCurrent();
				//Read encoder value here
				encoder = LeftEnc->GetSelectedSensorPosition(0);
				//Normalvalue is just the number put for safe operating current and is a placeholder
				NormalValue = 16;
				Range = 0.5;
				if(current == 0 && encoder <= 10){
					cout << "Warning: Motor#L2 Power Disconnect likely"<<endl;
				}else if(current > NormalValue+Range && encoder <= 10){
					cout << "Warning: Probable Motor Stall Detected In Motor#L2 Disconnect Motor#L2 Immediately"<<endl;
				}else if(current < NormalValue+Range && current > NormalValue-Range && encoder <=10){
					cout << "Warning: Encoder Error Detected In Motor#L2"<<endl;
				}else if(current < NormalValue+Range && current > NormalValue-Range && encoder >=10){
					cout << "Motor#L2 Passed Test"<<endl;
				}else if(current == 0 && encoder > 10){
					cout<<"Warning: Current Read Error Motor#R2"<<endl;
				}else{
					//This should never happen but if it does I want to know
					cout << "Warning:Testing Error Motor#L2 Did Not Meet Any Testing Criteria"<<endl;
				}

				state = Reset;
				break;
			}

			case 5:
			{
				//Read Motor current here
				current = MotorR3->GetOutputCurrent();
				//Read encoder value here
				encoder = RightEnc->GetSelectedSensorPosition(0);
				//Normalvalue is just the number put for safe operating current and is a placeholder
				NormalValue = 16;
				Range = 0.5;
				if(current == 0 && encoder <= 100){
					cout << "Warning: Motor#R3 Power Disconnect likely"<<endl;
				}else if(current > NormalValue+Range && encoder <= 10){
					cout << "Warning: Probable Motor Stall Detected In Motor#R3 Disconnect Motor#R3 Immediately"<<endl;
				}else if(current < NormalValue+Range && current > NormalValue-Range && encoder <=10){
					cout << "Warning: Encoder Error Detected In Motor#R3"<<endl;
				}else if(current < NormalValue+Range && current > NormalValue-Range && encoder >=10){
					cout << "Motor#R3 Passed Test"<<endl;

				}else if(current == 0 && encoder > 10){
					cout<<"Warning: Current Read Error Motor#R3"<<endl;
				}else{
					//This should never happen but if it does I want to know
					cout << "Warning:Testing Error Motor#R3 Did Not Meet Any Testing Criteria"<<endl;
				}

				state = Reset;
				break;

			}
			case 6:
			{
				//Read Motor current here
				current = MotorL3->GetOutputCurrent();
				//Read encoder value here
				encoder = LeftEnc->GetSelectedSensorPosition(0);
				//Normalvalue is just the number put for safe operating current and is a placeholder
				NormalValue = 16;
				Range = 0.5;
				if(current == 0 && encoder <= 10){
					cout << "Warning: Motor#L3 Power Disconnect likely"<<endl;
				}else if(current > NormalValue+Range && encoder <= 10){
					cout << "Warning: Probable Motor Stall Detected In Motor#L3 Disconnect Motor#L3 Immediately"<<endl;
				}else if(current < NormalValue+Range && current > NormalValue-Range && encoder <=10){
					cout << "Warning: Encoder Error Detected In Motor#L3"<<endl;
				}else if(current < NormalValue+Range && current > NormalValue-Range && encoder >=10){
					cout << "Motor#L3 Passed Test"<<endl;
				}else if(current == 0 && encoder >= 10){
					cout<<"Warning: Current Read Error Motor#L3"<<endl;
				}else{
					//This should never happen but if it does I want to know
					cout << "Warning:Testing Error Motor#L3 Did Not Meet Any Testing Criteria"<<endl;
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
