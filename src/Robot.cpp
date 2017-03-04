#include "WPIlib.h"
#include "Commands/Command.h"
#include "Commands/ExampleCommand.h"
#include "CommandBase.h"
#include "CANTalon.h"
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>
#include <AHRS.h>

/*
 * I/O map
 * Talons:
 * robotDriveFront CANTalon 0
 * robotDriveFront CANTalon 3
 * robotDriveRear CANTalon 2
 * robotDriveRear CANTalon 5
 * midwheelRight CANTalon 1
 * midwheelLeft  CANTalon 4
 * winchControl CANTalon
 * winchFollow CANTalon
 * shooterControl CANTalon
 * shooterFollow CANTalon
 * leftdriveenc GND, Empty, A, +, B
 * rightdriveenc GND, Empty, A, +, B
 * gearSolenoid SingleSolenoid - 0
 * gearIntake DoubleSolenoid - 0, 1
 * Shift DoubleSolenoid- 2, 3
 * PCM 0 - 24V
 * PCM 1 - 12V
 */

// Once Robot is wired, double-check what things are named as

class Robot: public IterativeRobot {

	CANTalon talon0;
	CANTalon talon1;
	CANTalon talon2;
	CANTalon talon5;
	CANTalon midWheelLeft;
	CANTalon midWheelRight;
	CANTalon winchControl;
	CANTalon winchFollow;
	CANTalon shooterControl;
	CANTalon shooterFollow;
	RobotDrive robotDriveFront;
	RobotDrive robotDriveRear;
	//Encoder leftdriveenc;
	//Encoder rightdriveenc;
	Compressor cps;
	Solenoid gearSolenoid;
	DoubleSolenoid gearIntake;
	DoubleSolenoid Shift;
	Joystick jsL;
	Joystick jsR;
	Joystick jsG;
	Timer timer;
	cs::UsbCamera cam0;
	cs::UsbCamera cam1;
	cs::UsbCamera cam2;

public:
	Robot() :
		talon0(0),
		talon1(2),
		talon2(3),
		talon5(5),
		midWheelLeft(4),
		midWheelRight(1),
		winchControl(6),
		winchFollow(7),
		shooterControl(8),
		shooterFollow(9),
		robotDriveFront(talon0 , talon2),
		robotDriveRear(talon1, talon5),
		//leftdriveenc(0, 1, false, Encoder::EncodingType::k2X),
		//rightdriveenc(2, 3, false, Encoder::EncodingType::k2X),
		cps(),
		gearSolenoid(1, 0),
		gearIntake(0, 0, 1),
		Shift(0, 2, 3),
		jsL(0),
		jsR(1),
		jsG(2){
	}


private:
	void VisionThread() {
		cam0 = CameraServer::GetInstance()->StartAutomaticCapture(0);
		cam0.SetResolution(640,480);

		cs::CvSink cvSink = CameraServer::GetInstance()->GetVideo();
		cs::CvSource outputStreamStd = CameraServer::GetInstance()->PutVideo("Gray", 640, 480);
		cv::Mat source;
		cv::Mat output;

		while(1) {
			//grip::GripPipeline gp;
		}
	}

	void RobotInit() {
		talon0.SetControlMode(CANSpeedController::kPercentVbus);
		talon1.SetControlMode(CANSpeedController::kPercentVbus);
		talon2.SetControlMode(CANSpeedController::kPercentVbus);
		talon5.SetControlMode(CANSpeedController::kPercentVbus);

		midWheelLeft.SetControlMode(CANSpeedController::kPercentVbus);
		midWheelLeft.SetFeedbackDevice(CANTalon::QuadEncoder);
		midWheelRight.SetControlMode(CANSpeedController::kPercentVbus);
		midWheelRight.SetFeedbackDevice(CANTalon::QuadEncoder);

		winchControl.SetControlMode(CANSpeedController::kPercentVbus);
		winchFollow.SetControlMode(CANSpeedController::kPercentVbus);


		winchFollow.SetControlMode(CANSpeedController::kFollower);
		winchFollow.Set(6);

		shooterControl.SetControlMode(CANSpeedController::kPercentVbus);
		shooterFollow.SetControlMode(CANSpeedController::kPercentVbus);

		shooterFollow.SetControlMode(CANSpeedController::kFollower);
		shooterFollow.Set(8);

		/*//Initializes gearbox encoder on left side of drive train
		leftdriveenc.Reset();
		leftdriveenc.SetMaxPeriod(.1);
		leftdriveenc.SetMinRate(10);
		leftdriveenc.SetDistancePerPulse(2.8125);
		leftdriveenc.SetReverseDirection(false);
		leftdriveenc.SetSamplesToAverage(7);

		//Initializes gearbox encoder on right side of drive train
		rightdriveenc.Reset();
		rightdriveenc.SetMaxPeriod(.1);
		rightdriveenc.SetMinRate(10);
		rightdriveenc.SetDistancePerPulse(2.8125);
		rightdriveenc.SetReverseDirection(false);
		rightdriveenc.SetSamplesToAverage(7); */

		cam0 = CameraServer::GetInstance()->StartAutomaticCapture(0);
		cam1 = CameraServer::GetInstance()->StartAutomaticCapture(1);
	}

	int pointer = 0;
	int index = 0;

	bool isRedAlliance = false;
	bool isBlueAlliance = false;
	bool driveForward = false;
	bool gearAuton = false;
	bool lowGoalAuton = false;
	bool nothing = false;

	void EnableClosedLoop(double targetPosition, double rightmotorDirection, double leftmotorDirection) {
		midWheelLeft.SetVoltageRampRate(0);
		midWheelRight.SetVoltageRampRate(0);

		midWheelLeft.SetControlMode(CANTalon::kPosition);
		midWheelRight.SetControlMode(CANTalon::kPosition);

		talon1.SetControlMode(CANSpeedController::kFollower);
		talon2.SetControlMode(CANSpeedController::kFollower);
		talon0.SetControlMode(CANSpeedController::kFollower);
		talon5.SetControlMode(CANSpeedController::kFollower);

		talon1.Set(4);
		talon2.Set(1);
		talon0.Set(4);
		talon5.Set(1);

		midWheelRight.Set(rightmotorDirection * targetPosition);
		midWheelLeft.Set(leftmotorDirection * targetPosition);
	}

	void StopRobot() {
		robotDriveFront.SetLeftRightMotorOutputs(0, 0); //stops robot
		robotDriveRear.SetLeftRightMotorOutputs(0, 0);
		midWheelLeft.Set(0);
		midWheelRight.Set(0);
	}

	void Gear(double timeTwo) {
		float time = timer.Get();

		timer.Reset();
		time = timer.Get();
		while (time < timeTwo) {
			gearSolenoid.Set(false);
		}
	}

	void AutonomousInit() {
		if (jsG.GetRawButton(2)) {
			isRedAlliance = true;
			isBlueAlliance = false;
		}
		if (jsG.GetRawButton(3)) {
			isRedAlliance = false;
			isBlueAlliance = true;
		}
		if (jsG.GetRawButton(4)) {
			driveForward = true;
			gearAuton = false;
			lowGoalAuton = false;
			nothing = false;
		} else if (jsG.GetRawButton(6)) {
			gearAuton = true;
			driveForward = false;
			lowGoalAuton = false;
			nothing = false;
		} else if (jsG.GetRawButton(7)) {
			gearAuton = false;
			driveForward = false;
			lowGoalAuton = true;
			nothing = false;
		} else if (jsG.GetRawButton(5)) {
			nothing = true;
			gearAuton = false;
			driveForward = false;
			lowGoalAuton = false;
			nothing = false;
		}

		cps.Start();

		timer.Reset();
		timer.Start();
	}

	void AutonomousPeriodic() {
		/*if (isRedAlliance && !isBlueAlliance) {
			driveTime = 5.0;
			turnTime = 1.0;
			timeTwo = 1.0;
			timeThree = 1.0;
			timeFour = 1.0;
		} else if (isBlueAlliance && !isRedAlliance) {
			driveTime = 5.0;
			turnTime = 1.0;
			timeTwo = 1.0;
			timeThree = 1.0;
			timeFour = 1.0;
		} else {
			driveTime = 7.0;
			turnTime = 0.0;
			timeTwo = 0.0;
			timeThree = 0.0;
			timeFour = 0.0;
		} */

		auto grip = NetworkTable::GetTable("grip");

		auto Xcenter = grip->GetNumberArray("ContoursReport/centerX", 1);
		auto Ycenter = grip->GetNumberArray("ContoursReport/centerY", 1);

		float time = timer.Get();

		double lcount = midWheelLeft.GetEncPosition();
		double rcount = midWheelRight.GetEncPosition();

		//leftdriveenc.Reset();
		//rightdriveenc.Reset();

		if (nothing) {
			StopRobot();

		}

		if (gearAuton) {
				// MPH - is the gearshift default correct for autonomous?
			midWheelLeft.SetPosition(0);
			midWheelRight.SetPosition(0);

			EnableClosedLoop(3000, 1, 1);

			midWheelLeft.SetPosition(0);
			midWheelRight.SetPosition(0);

			if (isRedAlliance) {
				EnableClosedLoop(3000, 1, -1);
			} else if (isBlueAlliance) {
				EnableClosedLoop(3000, -1, 1);
			}

			EnableClosedLoop(1000, 1, 1);

			Gear(2.0);

			gearSolenoid.Set(true);

			EnableClosedLoop(1000, -1, -1);

			StopRobot();
			//printf("{%.2f},\n", rcount);
			printf("{%.2f},\n", time);
		}

		if (driveForward) {
			EnableClosedLoop(10000, 1, 1);
		}

		if (lowGoalAuton) {
			midWheelLeft.SetPosition(0);
			midWheelRight.SetPosition(0);

			EnableClosedLoop(3000, 1, 1);

			midWheelLeft.SetPosition(0);
			midWheelRight.SetPosition(0);

			if (isRedAlliance) {
				EnableClosedLoop(3000, 1, -1);
			} else if (isBlueAlliance) {
				EnableClosedLoop(3000, -1, 1);
			}

			EnableClosedLoop(1000, 1, 1);


			EnableClosedLoop(1000, -1, -1);

			StopRobot();
			//printf("{%.2f},\n", rcount);
			printf("{%.2f},\n", time);

		}
	}

	void TeleopInit() {
		cps.Start();

		talon0.SetControlMode(CANSpeedController::kPercentVbus);
		talon1.SetControlMode(CANSpeedController::kPercentVbus);
		talon2.SetControlMode(CANSpeedController::kPercentVbus);
		talon5.SetControlMode(CANSpeedController::kPercentVbus);
		midWheelLeft.SetControlMode(CANSpeedController::kPercentVbus);
		midWheelRight.SetControlMode(CANSpeedController::kPercentVbus);

		Shift.Set(DoubleSolenoid::kOff);
	}

	void Drive() {
		float leftInput = 1 * jsL.GetY();
		float rightInput = 1 * jsR.GetY();
		float leftMidwheelInput = 1 * jsL.GetY();
		float rightMidwheelInput = 1* jsR.GetY();
		robotDriveFront.SetLeftRightMotorOutputs(leftInput, rightInput);
		robotDriveRear.SetLeftRightMotorOutputs(leftInput, rightInput);
		midWheelLeft.Set(leftMidwheelInput);
		midWheelRight.Set(-1 * rightMidwheelInput);

	}

	void Gears() {
		if (jsG.GetRawButton(2)) {
			gearSolenoid.Set(false);
		//}
		} else {
			gearSolenoid.Set(true);
		}

		if (jsG.GetRawButton(4)) {
			gearIntake.Set(DoubleSolenoid::kForward);
		}
		else if (jsG.GetRawButton(5)) {
			gearIntake.Set(DoubleSolenoid::kReverse);
		} else {
			gearIntake.Set(DoubleSolenoid::kOff);
		}
	}

	void Balls() {

		if (jsG.GetRawButton(1)) {
			shooterControl.Set(1);
		}
	}

	void Gearshift() { //drive train gearshift (high to low gear)
		if (jsL.GetRawButton(2)) { //Close
			Shift.Set(DoubleSolenoid::kForward);
		} else if (jsL.GetRawButton(3)) { //Open
			Shift.Set(DoubleSolenoid::kReverse);
		} else {
			Shift.Set(DoubleSolenoid::kOff);
		}
	}

	void Winch() {
		float winchInput = jsG.GetY();
		winchControl.Set(-1 * abs(winchInput));
	}

	void TeleopPeriodic() {
		this->Drive();
		this->Gearshift();
		this->Gears();
		this->Balls();
		this->Winch();

	}

	void TestInit() {
			cps.Start();
		}

	void AutoDrive(double leftInput, double rightInput) {
		float time = timer.Get();

		timer.Reset();
		timer.Start();
		time = timer.Get();
		while (time < 2) {

			robotDriveFront.SetLeftRightMotorOutputs(leftInput, rightInput);
			robotDriveRear.SetLeftRightMotorOutputs(leftInput, rightInput);
			midWheelLeft.Set(leftInput);
			midWheelRight.Set(-1 * rightInput);

			time = timer.Get();
			printf("drive wheel", leftInput, "speed test");
		}
	}

	void TestPeriodic() {

		AutoDrive(0.10, 0.10);

		AutoDrive(0.50, 0.50);

		AutoDrive(1.00, 1.00);

		AutoDrive(1.00, -1.00);

		AutoDrive(-1.00, -1.00);

		timer.Reset();
		timer.Start();
		double time = timer.Get();
		while (time < 3) {
			double leftInput = 1.00;
			double rightInput = 1.00;

			robotDriveFront.SetLeftRightMotorOutputs(leftInput, rightInput);
			robotDriveRear.SetLeftRightMotorOutputs(leftInput, rightInput);
			midWheelLeft.Set(leftInput);
			midWheelRight.Set(-1 * rightInput);

			Shift.Set(DoubleSolenoid::kForward);

			while (time < 3 && time > 2) {
				double leftInput = 1.00;
				double rightInput = 1.00;

				robotDriveFront.SetLeftRightMotorOutputs(leftInput, rightInput);
				robotDriveRear.SetLeftRightMotorOutputs(leftInput, rightInput);
				midWheelLeft.Set(leftInput);
				midWheelRight.Set(-1 * rightInput);

				Shift.Set(DoubleSolenoid::kReverse);
			}

			Shift.Set(DoubleSolenoid::kOff);

			time = timer.Get();
			printf("drive wheel shift test");
		}

		timer.Reset();
		timer.Start();
		time = timer.Get();
		while (time < 2) {
			double winchInput = 1.0;

			winchControl.Set(winchInput);

			time = timer.Get();
			printf("winch test");
		}

		timer.Reset();
		timer.Start();
		time = timer.Get();
		while (time < 2) {
			gearIntake.Set(DoubleSolenoid::kForward);

			while (time < 2 && time > 1) {
				gearIntake.Set(DoubleSolenoid::kReverse);
			}

			time = timer.Get();
			printf("gear intake test");
		}
		gearIntake.Set(DoubleSolenoid::kOff);

		timer.Reset();
		timer.Start();
		time = timer.Get();
		while (time < 2) {
			gearSolenoid.Set(true);
			printf("gear outtake test");
		}
		gearSolenoid.Set(false);

	}

};

START_ROBOT_CLASS(Robot)
