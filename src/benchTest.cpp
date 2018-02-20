#include "WPILib.h"
#include "RobotUtils/HotJoystick.h"
#include "ctre/Phoenix.h"

/*
 */

class benchTest: public TimedRobot {
private:

	WPI_TalonSRX* m_CANmotor1;
	WPI_TalonSRX* m_CANmotor3;
	AnalogInput* m_Analog0;
	DigitalInput* m_SwitchDIO4;
	Encoder* m_EncoderMtr1;
	PowerDistributionPanel* pdp;

	bool motorStTransInProcess = false;
	bool switchSignal;
	bool switchSignalOld = false;
	double pot1;
	int testMode = 0;
	double current1;
	double current3;
	double encoder3;
	double actSpdRPM3;
	double voltage1;
	double voltage3;
	double percent1;
	double percent3;
	double pdpCurr1;
	double pdpCurr3;

	double currentLimit = 0.0;

	double speedCommandPID = 0.0;

	double encoderTicksMtr1;
	double actSpdRPM1;

	double spdCmd;

	double pdpVoltage;

public:
	benchTest() {
		m_Analog0 = new AnalogInput(0);  /* Analog Input Channel 0 */
		m_SwitchDIO4 = new DigitalInput(4);

		m_CANmotor1 = new WPI_TalonSRX(1);
		m_CANmotor1->ConfigOpenloopRamp(8.0, 0);
		m_CANmotor1->ConfigClosedloopRamp(8.0, 0);

		m_CANmotor3 = new WPI_TalonSRX(3);
		m_CANmotor3->ConfigSelectedFeedbackSensor(FeedbackDevice::CTRE_MagEncoder_Relative, 0, 0);
		m_CANmotor3->SetInverted(true);
		m_CANmotor3->SetSensorPhase(true);
		m_CANmotor3->SetSelectedSensorPosition(0, 0, 0);
		m_CANmotor3->ConfigOpenloopRamp(0.5, 0);
		m_CANmotor3->ConfigClosedloopRamp(0.5, 0);

		m_EncoderMtr1 = new Encoder(2,3,true,Encoder::EncodingType::k4X);
		m_EncoderMtr1->SetSamplesToAverage(15);
		m_EncoderMtr1->SetDistancePerPulse(1.0 / 256.0); /* 1 rotation per 256 pulses */
		m_EncoderMtr1->Reset();

		pdp = new PowerDistributionPanel();
	}
	void RobotInit() {
	}


	void AutonomousInit() {

	}

	void AutonomousPeriodic() {

	}

	void TeleopInit() {
		testMode = 0;
		speedCommandPID = 0.0;
		m_CANmotor3->Config_kF(0, 0.0339, 0);
		m_CANmotor3->Config_kP(0, 0.0001, 0);
		m_CANmotor3->Config_kI(0, 0.0001, 0);
		m_CANmotor3->Config_kD(0, 0.0000, 0);
	}

	void TeleopPeriodic() {

		 /* Read inputs from DIO4 */
		 switchSignal = !m_SwitchDIO4->Get();

		 /* Read potentiometer inputs from Analog0 */
		 pot1 = m_Analog0->GetValue();
		 spdCmd = (pot1 - 15.0) / 3900.0;
		 if (spdCmd < 0.0) {
			 spdCmd = 0.0;
		 } else if (spdCmd > 1.0) {
			 spdCmd = 1.0;
		 }

		 /* Read Motor Parameters */
		 current1 = m_CANmotor1->GetOutputCurrent();
		 current3 = m_CANmotor3->GetOutputCurrent();
		 voltage1 = m_CANmotor1->GetMotorOutputVoltage();
		 voltage3 = m_CANmotor3->GetMotorOutputVoltage();
		 percent1 = m_CANmotor1->GetMotorOutputPercent();
		 percent3 = m_CANmotor3->GetMotorOutputPercent();
		 encoder3 = (m_CANmotor3->GetSelectedSensorPosition(0) / 4096.0);
		 actSpdRPM3 = m_CANmotor3->GetSelectedSensorVelocity(0) * (600.0/4096.0);

		 pdpCurr1 = pdp->GetCurrent(0);
		 pdpCurr3 = pdp->GetCurrent(2);
		 pdpVoltage = pdp->GetVoltage();

		 /* Convert encoder rate to rpm */
		 encoderTicksMtr1 = m_EncoderMtr1->Get();
		 actSpdRPM1 = (m_EncoderMtr1->GetRate() * 60.0);

		 /* Process button presses */
		 /* Look for aButton to transition from false to true */
		 if ((switchSignal == true) && (switchSignalOld == false)) {
			 motorStTransInProcess = true;
		 } else {
			 motorStTransInProcess = false;
		 }

		 /* Use motorSelect to define speed commands to motors */
		 if (motorStTransInProcess == true) {
			 if (testMode < 2) {
				 testMode++;
			 } else {
				 testMode = 0;
			 }
		 }

		 /* Command speeds to motor controllers */
		 switch (testMode) {
		 case 0:
			 m_EncoderMtr1->Reset();
			 m_CANmotor3->SetSelectedSensorPosition(0, 0, 0);
			 m_CANmotor1->Set(ControlMode::PercentOutput, 0.0);
			 m_CANmotor3->Set(ControlMode::PercentOutput, 0.0);
			 speedCommandPID = 0.0;
			 currentLimit = 0.0;
			 break;
		 case 1:
			 m_CANmotor1->Set(ControlMode::PercentOutput, spdCmd);
			 m_CANmotor3->Set(ControlMode::PercentOutput, spdCmd);
			 speedCommandPID = actSpdRPM3;
			 break;
		 case 2:
			 m_CANmotor3->Set(ControlMode::Velocity, (speedCommandPID * (4096.0/600.0)));
			 m_CANmotor1->Follow(*m_CANmotor3);
			 currentLimit = current3;
			 break;
		 }

		 /* Preserve knowledge of previous loop button state */
		 switchSignalOld = switchSignal;

		 DashboardOutput();
	}

	void DashboardOutput() {
		/* Writes variables to  Dashboard */
		SmartDashboard::PutBoolean("switchSignal", switchSignal);
		SmartDashboard::PutNumber("SpdCmd", spdCmd);
		SmartDashboard::PutNumber("testMode", testMode);
		SmartDashboard::PutNumber("current1", current1);
		SmartDashboard::PutNumber("current3", current3);
		SmartDashboard::PutNumber("encoder3", encoder3);
		SmartDashboard::PutNumber("actSpdRPM1", actSpdRPM1);
		SmartDashboard::PutNumber("actSpdRPM3", actSpdRPM3);
		SmartDashboard::PutNumber("speedCommandPID", speedCommandPID);
		SmartDashboard::PutNumber("voltage1", voltage1);
		SmartDashboard::PutNumber("voltage3", voltage3);
		SmartDashboard::PutNumber("percent1", percent1);
		SmartDashboard::PutNumber("percent3", percent3);
		SmartDashboard::PutNumber("currentLimit", currentLimit);
		SmartDashboard::PutNumber("pdpCurr1", pdpCurr1);
		SmartDashboard::PutNumber("pdpCurr3", pdpCurr3);
		SmartDashboard::PutNumber("pdpVoltage", pdpVoltage);

	}


	void TestPeriodic() {
	}

};

START_ROBOT_CLASS(benchTest)
