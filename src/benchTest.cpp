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
	Menu,
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
	int Lencoder;
	int Rencoder;
	int Tencoder;
	double NormVal[10];
	bool menu;
	double NormalValue;
	double Range;
	double MultiFalure[3];




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



	}

	void TeleopPeriodic() {






		//MotorR1->Set(ControlMode::PercentOutput, 1);
		MotorR2->Set(ControlMode::PercentOutput, 1);
		//MotorR3->Set(ControlMode::PercentOutput, 1);
		//MotorL1->Set(ControlMode::PercentOutput, 1);
		//MotorL2->Set(ControlMode::PercentOutput, 1);
		//MotorL3->Set(ControlMode::PercentOutput, 1);

		//current = MotorL2->GetOutputCurrent();
		//Lencoder = LeftEnc->GetSelectedSensorPosition(0);
		SmartDashboard::PutNumber("Current",current);
		SmartDashboard::PutNumber("LEncoder",Lencoder);



	}






	void MotorBrake(){
		MotorR1->SetNeutralMode(Brake);
		MotorR2->SetNeutralMode(Brake);
		MotorR3->SetNeutralMode(Brake);
		MotorL1->SetNeutralMode(Brake);
		MotorL2->SetNeutralMode(Brake);
		MotorL3->SetNeutralMode(Brake);
	}

	void MotorStop(){
		MotorR1->Set(ControlMode::PercentOutput, 0);
		MotorR2->Set(ControlMode::PercentOutput, 0);
		MotorR3->Set(ControlMode::PercentOutput, 0);
		MotorL1->Set(ControlMode::PercentOutput, 0);
		MotorL2->Set(ControlMode::PercentOutput, 0);
		MotorL3->Set(ControlMode::PercentOutput, 0);
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
		Lencoder = 0;
		Rencoder = 0;
	}
	void EncRead(){
		Rencoder = RightEnc->GetSelectedSensorPosition(0);
		Lencoder = LeftEnc->GetSelectedSensorPosition(0);
	}

	void TestInit(){
		MotorR2->Set(ControlMode::PercentOutput, 1);
		MotorStop();
		state = Menu;
		TestCase = 0;
		for(int i=0;i<10;i++){
			cout<<" "<<endl;
		}
		NormVal[1] = 12;
		NormVal[2] = 14;
		NormVal[3] = 12;
		NormVal[4] = 15;
		NormVal[5] = 12;
		NormVal[6] = 15;
		Range = 1.75;

		cout<<"Thank you for using ALPHA Self Test V1.0"<<endl;
		cout<<"Please Stand Back and Make Sure Robot Is On Stilts"<<endl;
		cout<<"Press Y To Start To Test"<<endl;
	}


	void TestPeriodic() {
		EncRead();
		SmartDashboard::PutNumber("LEncoder",Lencoder);
		SmartDashboard::PutNumber("REncoder",Rencoder);
		SmartDashboard::PutNumber("Current",current);


		if(m_driver->ButtonPressedA() && menu == false){
			state = Menu;
			cout<<"Test Paused"<<endl;
			TestCase = TestCase-1;
		}


		switch(state) {
		case Menu:
			MotorStop();
			menu = true;

			if(m_driver->ButtonPressedX() == true && TestCase > 0){
				TestCase = TestCase-1;
				cout<<"TestCase="<<TestCase+1<<endl;
			}
			if(m_driver->ButtonY() == true) {
				state = Reset;
				cout<<"Starting Test"<<endl;
			}
			if(m_driver->ButtonPressedB() == true){
				TestCase++;
				cout<<"Test Skipped: Currently on TestCase="<<TestCase+1<<endl;
			}





			break;
		case Reset:
			menu = false;
			MotorStop();
			MotorBrake();

			count = count+1;
			if(count>15){
				count = 0;
				state = Prep;
				TestCase = TestCase+1;
				EncoderReset();
			}

			break;
		case Prep:
			MotorCoast();
			if(TestCase == 1) MotorR1->Set(ControlMode::PercentOutput, -1);
			if(TestCase == 2) MotorL1->Set(ControlMode::PercentOutput, -1);
			if(TestCase == 3) MotorR2->Set(ControlMode::PercentOutput, -1);
			if(TestCase == 4) MotorL2->Set(ControlMode::PercentOutput, -1);
			if(TestCase == 5) MotorR3->Set(ControlMode::PercentOutput, -1);
			if(TestCase == 6) MotorL3->Set(ControlMode::PercentOutput, -1);

			count = count+1;
			if(count>30){
				count = 0;
				state = Test;
				SmartDashboard::PutNumber("Current",current);
			}
			break;
		case Test:
			EncRead();
			switch(TestCase){
			case 1:
			{
				current = MotorR1->GetOutputCurrent();
				NormalValue = NormVal[TestCase];
				Tencoder = Rencoder;

				if((current < NormalValue+Range && current > NormalValue-Range) && Tencoder >=100){
					cout<<"Motor"<<TestCase<<" Passed Test"<<endl;
				}else if(current == 0 && Tencoder <= 100){

					cout<<"Warning: Motor"<<TestCase<<" Is Probably Disconnected"<<endl;
				}else if((current < NormalValue+Range && current > NormalValue-Range) && Tencoder == 0){

					cout<<"Warning: Motor"<<TestCase<<" Likely Encoder Register Error"<<endl;
					MultiFalure[1]++;
				}else if(current > NormalValue+Range && Tencoder == 0){

					cout<<"Warning: Motor"<<TestCase<<" Stall Detected"<<endl;
				}else if(current > NormalValue+Range && Tencoder > 100){

					cout<<"Warning: Motor"<<TestCase<<" OverDraw Detected"<<endl;
				}else if(current < NormalValue-Range && Tencoder > 100){

					cout<<"Motor"<<TestCase<<" Passed Test Recommend ReCalabration"<<endl;
				}else if((current < NormalValue+Range && current > NormalValue-Range) && Tencoder <-100){
					cout<<"Warning: Motor"<<TestCase<<" Inverted Polarity Detected"<<endl;
				}else{

					cout<<"Warning: This case failed miserably"<<endl;
				}

				state = Reset;
				break;
			}

			case 2:
			{
				current = MotorL1->GetOutputCurrent();
				NormalValue = NormVal[TestCase];
				Tencoder = Lencoder;

				if((current < NormalValue+Range && current > NormalValue-Range) && Tencoder >=100){
					cout<<"Motor"<<TestCase<<" Passed Test"<<endl;
				}else if(current == 0 && Tencoder <= 100){

					cout<<"Warning: Motor"<<TestCase<<" Is Probably Disconnected"<<endl;
				}else if((current < NormalValue+Range && current > NormalValue-Range) && Tencoder == 0){

					cout<<"Warning: Motor"<<TestCase<<" Likely Encoder Register Error"<<endl;
					MultiFalure[2]++;
				}else if(current > NormalValue+Range && Tencoder == 0){

					cout<<"Warning: Motor"<<TestCase<<" Stall Detected"<<endl;
				}else if(current > NormalValue+Range && Tencoder < 100){

					cout<<"Warning: Motor"<<TestCase<<" OverDraw Detected"<<endl;
				}else if(current < NormalValue-Range && Tencoder > 100){

					cout<<"Motor"<<TestCase<<" Passed Test Recommend ReCalabration"<<endl;
				}else if((current < NormalValue+Range && current > NormalValue-Range) && Tencoder <-100){
					cout<<"Warning: Motor"<<TestCase<<" Inverted Polarity Detected"<<endl;
				}else{

					cout<<"Warning: This case failed miserably"<<endl;
				}

				state = Reset;
				break;
			}
			break;
			case 3:
			{
				current = MotorR2->GetOutputCurrent();
				NormalValue = NormVal[TestCase];
				Tencoder = Rencoder;

				if((current < NormalValue+Range && current > NormalValue-Range) && Tencoder >=100){
					cout<<"Motor"<<TestCase<<" Passed Test"<<endl;
				}else if(current == 0 && Tencoder <= 100){

					cout<<"Warning: Motor"<<TestCase<<" Is Probably Disconnected"<<endl;
				}else if((current < NormalValue+Range && current > NormalValue-Range) && Tencoder == 0){

					cout<<"Warning: Motor"<<TestCase<<" Likely Encoder Register Error"<<endl;
					MultiFalure[1]++;
				}else if(current > NormalValue+Range && Tencoder == 0){

					cout<<"Warning: Motor"<<TestCase<<" Stall Detected"<<endl;
				}else if(current > NormalValue+Range && Tencoder > 100){

					cout<<"Warning: Motor"<<TestCase<<" OverDraw Detected"<<endl;
				}else if(current < NormalValue-Range && Tencoder > 100){

					cout<<"Motor"<<TestCase<<" Passed Test Recommend ReCalabration"<<endl;
				}else if((current < NormalValue+Range && current > NormalValue-Range) && Tencoder <-100){
					cout<<"Warning: Motor"<<TestCase<<" Inverted Polarity Detected"<<endl;
				}else{

					cout<<"Warning: This case failed miserably"<<endl;
				}

				state = Reset;
				break;

			}
			case 4:
			{
				current = MotorL2->GetOutputCurrent();
				NormalValue = NormVal[TestCase];
				Tencoder = Lencoder;

				if((current < NormalValue+Range && current > NormalValue-Range) && Tencoder >=100){
					cout<<"Motor"<<TestCase<<" Passed Test"<<endl;
				}else if(current == 0 && Tencoder <= 100){

					cout<<"Warning: Motor"<<TestCase<<" Is Probably Disconnected"<<endl;
				}else if((current < NormalValue+Range && current > NormalValue-Range) && Tencoder == 0){

					cout<<"Warning: Motor"<<TestCase<<" Likely Encoder Register Error"<<endl;
					MultiFalure[2]++;
				}else if(current > NormalValue+Range && Tencoder == 0){

					cout<<"Warning: Motor"<<TestCase<<" Stall Detected"<<endl;
				}else if(current > NormalValue+Range && Tencoder > 100){

					cout<<"Warning: Motor"<<TestCase<<" OverDraw Detected"<<endl;
				}else if(current < NormalValue-Range && Tencoder > 100){

					cout<<"Motor"<<TestCase<<" Passed Test Recommend ReCalabration"<<endl;
				}else if((current < NormalValue+Range && current > NormalValue-Range) && Tencoder <-100){
					cout<<"Warning: Motor"<<TestCase<<" Inverted Polarity Detected"<<endl;
				}else{

					cout<<"Warning: This case failed miserably"<<endl;
				}

				state = Reset;
				break;

			}

			case 5:
			{
				current = MotorR3->GetOutputCurrent();
				NormalValue = NormVal[TestCase];
				Tencoder = Rencoder;

				if((current < NormalValue+Range && current > NormalValue-Range) && Tencoder >=100){
					cout<<"Motor"<<TestCase<<" Passed Test"<<endl;
				}else if(current == 0 && Tencoder <= 100){

					cout<<"Warning: Motor"<<TestCase<<" Is Probably Disconnected"<<endl;
				}else if((current < NormalValue+Range && current > NormalValue-Range) && Tencoder == 0){

					cout<<"Warning: Motor"<<TestCase<<" Likely Encoder Register Error"<<endl;
					MultiFalure[1]++;
				}else if(current > NormalValue+Range && Tencoder == 0){

					cout<<"Warning: Motor"<<TestCase<<" Stall Detected"<<endl;
				}else if(current > NormalValue+Range && Tencoder > 100){

					cout<<"Warning: Motor"<<TestCase<<" OverDraw Detected"<<endl;
				}else if(current < NormalValue-Range && Tencoder > 100){

					cout<<"Motor"<<TestCase<<" Passed Test Recommend ReCalabration"<<endl;
				}else if((current < NormalValue+Range && current > NormalValue-Range) && Tencoder <-100){
					cout<<"Warning: Motor"<<TestCase<<" Inverted Polarity Detected"<<endl;
				}else{

					cout<<"Warning: This case failed miserably"<<endl;
				}

				state = Reset;
				break;


			}
			case 6:
			{
				current = MotorL3->GetOutputCurrent();
				NormalValue = NormVal[TestCase];
				Tencoder = Lencoder;

				if((current < NormalValue+Range && current > NormalValue-Range) && Tencoder >=100){
					cout<<"Motor"<<TestCase<<" Passed Test"<<endl;
				}else if(current == 0 && Tencoder <= 100){

					cout<<"Warning: Motor"<<TestCase<<" Is Probably Disconnected"<<endl;
				}else if((current < NormalValue+Range && current > NormalValue-Range) && Tencoder == 0){

					cout<<"Warning: Motor"<<TestCase<<" Likely Encoder Register Error"<<endl;
					MultiFalure[2]++;
				}else if(current > NormalValue+Range && Tencoder == 0){

					cout<<"Warning: Motor"<<TestCase<<" Stall Detected"<<endl;
				}else if(current > NormalValue+Range && Tencoder > 100){

					cout<<"Warning: Motor"<<TestCase<<" OverDraw Detected"<<endl;
				}else if(current < NormalValue-Range && Tencoder > 100){

					cout<<"Motor"<<TestCase<<" Passed Test Recommend ReCalabration"<<endl;
				}else if((current < NormalValue+Range && current > NormalValue-Range) && Tencoder <-100){
					cout<<"Warning: Motor"<<TestCase<<" Inverted Polarity Detected"<<endl;
				}else{

					cout<<"Warning: This case failed miserably"<<endl;
				}

				state = Reset;
				break;
			}
			case 7:
			{
				cout<<"Test Complete and Paused"<<endl;
				if(MultiFalure[1] <= 3){
					cout<<"Right Encoder Failure"<<endl;
				}
				if(MultiFalure[2] <= 3){
					cout<<"Left Encoder Failure"<<endl;
				}
				state = Menu;
				break;
			}

			}

		}
	}


	//for opmode

};

START_ROBOT_CLASS(benchTest)
