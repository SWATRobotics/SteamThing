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
 * robotDriveFront CANTalon xxx
 * robotDriveFront CANTalon xxx
 * robotDriveRear CANTalon xxx
 * midwheelRight CANTalon xxx
 * midwheelLeft  CANTalon xxx
 *
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
		midWheelRight.SetControlMode(CANSpeedController::kPercentVbus);

		winchControl.SetControlMode(CANSpeedController::kPercentVbus);
		winchFollow.SetControlMode(CANSpeedController::kPercentVbus);


		winchFollow.SetControlMode(CANSpeedController::kFollower);
		winchFollow.Set(6);

		shooterControl.SetControlMode(CANSpeedController::kPercentVbus);
		shooterFollow.SetControlMode(CANSpeedController::kPercentVbus);

		shooterFollow.SetControlMode(CANSpeedController::kFollower);
		shooterFollow.Set(8);

		cam0 = CameraServer::GetInstance()->StartAutomaticCapture(0);
		cam1 = CameraServer::GetInstance()->StartAutomaticCapture(1);
	}

	int pointer = 0;
	int index = 0;

	bool isRedAlliance = false;
	bool isBlueAlliance = false;


	double driveTime = 0.0;
	double turnTime = 0.0;
	double timeTwo = 0.0;
	double timeThree = 0.0;
	double timeFour = 0.0;

	void AutonomousInit() {
		if (jsG.GetRawButton(2)) {
			isRedAlliance = true;
			isBlueAlliance = false;
		}
		if (jsG.GetRawButton(3)) {
			isRedAlliance = false;
			isBlueAlliance = true;
		}

		cps.Start();

		timer.Reset();
		timer.Start();
	}


	void AutonomousPeriodic() {
		if (isRedAlliance && !isBlueAlliance) {
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
		}

		auto grip = NetworkTable::GetTable("grip");

		auto Xcenter = grip->GetNumberArray("ContoursReport/centerX", 1);
		auto Ycenter = grip->GetNumberArray("ContoursReport/centerY", 1);

		float time = timer.Get();

		if (index == 0) {
				// MPH - is the gearshift default correct for autonomous?
			float leftInput = -0.37; //EGH - sets left and right voltage input of drive train
			float rightInput = -0.37;

			timer.Reset();
			time = timer.Get();
			while (time < driveTime) {
				robotDriveFront.SetLeftRightMotorOutputs(leftInput, rightInput); // MPH - maybe 0.25, 0.25 ? why is Right negative?  it's positive on TeleOp?
				robotDriveRear.SetLeftRightMotorOutputs(leftInput, rightInput); // maybe 0.25, 0.25 ? why is Right negative?  it's positive on TeleOp?
				midWheelLeft.Set(leftInput);
				midWheelRight.Set(-1 * rightInput);
			}

			timer.Reset();
			time = timer.Get();
			while (time < turnTime) {
				if (isRedAlliance) {
					robotDriveFront.SetLeftRightMotorOutputs(leftInput, -rightInput); // MPH - maybe 0.25, 0.25 ? why is Right negative?  it's positive on TeleOp?
					robotDriveRear.SetLeftRightMotorOutputs(leftInput, -rightInput); // maybe 0.25, 0.25 ? why is Right negative?  it's positive on TeleOp?
					midWheelLeft.Set(leftInput);
					midWheelRight.Set(1 * rightInput);
				}
				if (isBlueAlliance) {
					robotDriveFront.SetLeftRightMotorOutputs(-leftInput, rightInput); // MPH - maybe 0.25, 0.25 ? why is Right negative?  it's positive on TeleOp?
					robotDriveRear.SetLeftRightMotorOutputs(-leftInput, rightInput); // maybe 0.25, 0.25 ? why is Right negative?  it's positive on TeleOp?
					midWheelLeft.Set(-leftInput);
					midWheelRight.Set(-1 * rightInput);
				}
			}

			timer.Reset();
			time = timer.Get();
			while (time < timeTwo) {
				gearSolenoid.Set(false);
			}

			gearSolenoid.Set(true);
			timer.Reset();
			time = timer.Get();
			while (time < timeThree) {
				robotDriveFront.SetLeftRightMotorOutputs(-leftInput, -rightInput); // MPH - maybe 0.25, 0.25 ? why is Right negative?  it's positive on TeleOp?
				robotDriveRear.SetLeftRightMotorOutputs(-leftInput, -rightInput); // maybe 0.25, 0.25 ? why is Right negative?  it's positive on TeleOp?
				midWheelLeft.Set(-leftInput);
				midWheelRight.Set(1 * rightInput);
			}

			timer.Reset();
			time = timer.Get();
			while (time < timeFour) {

			}

			robotDriveFront.SetLeftRightMotorOutputs(0, 0); //stops robot
			robotDriveRear.SetLeftRightMotorOutputs(0, 0);
			midWheelLeft.Set(0);
			midWheelRight.Set(0);

			//printf("{%.2f},\n", rcount);
			printf("{%.2f},\n", time);
		}
	}

	void TeleopInit() {
		cps.Start();

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

	void TestPeriodic() {

		float time = timer.Get();

		timer.Reset();
		timer.Start();
		time = timer.Get();
		while (time < 2) {
			double leftInput = 0.10;
			double rightInput = 0.10;

			robotDriveFront.SetLeftRightMotorOutputs(leftInput, rightInput);
			robotDriveRear.SetLeftRightMotorOutputs(leftInput, rightInput);
			midWheelLeft.Set(leftInput);
			midWheelRight.Set(-1 * rightInput);

			time = timer.Get();
			printf("drive wheel 1/10 speed test");
		}

		timer.Reset();
		timer.Start();
		time = timer.Get();
		while (time < 2) {
			double leftInput = 0.50;
			double rightInput = 0.50;

			robotDriveFront.SetLeftRightMotorOutputs(leftInput, rightInput);
			robotDriveRear.SetLeftRightMotorOutputs(leftInput, rightInput);
			midWheelLeft.Set(leftInput);
			midWheelRight.Set(-1 * rightInput);

			time = timer.Get();
			printf("drive wheel 1/2 speed test");
		}

		timer.Reset();
		timer.Start();
		time = timer.Get();
		while (time < 2) {
			double leftInput = 1.00;
			double rightInput = 1.00;

			robotDriveFront.SetLeftRightMotorOutputs(leftInput, rightInput);
			robotDriveRear.SetLeftRightMotorOutputs(leftInput, rightInput);
			midWheelLeft.Set(leftInput);
			midWheelRight.Set(-1 * rightInput);

			time = timer.Get();
			printf("drive wheel full speed test");
		}

		timer.Reset();
		timer.Start();
		time = timer.Get();
		while (time < 2) {
			double leftInput = 1.00;
			double rightInput = -1.00;

			robotDriveFront.SetLeftRightMotorOutputs(leftInput, rightInput);
			robotDriveRear.SetLeftRightMotorOutputs(leftInput, rightInput);
			midWheelLeft.Set(leftInput);
			midWheelRight.Set(-1 * rightInput);

			time = timer.Get();
			printf("drive wheel turn speed test");
		}

		timer.Reset();
		timer.Start();
		time = timer.Get();
		while (time < 2) {
			double leftInput = -1.00;
			double rightInput = -1.00;

			robotDriveFront.SetLeftRightMotorOutputs(leftInput, rightInput);
			robotDriveRear.SetLeftRightMotorOutputs(leftInput, rightInput);
			midWheelLeft.Set(leftInput);
			midWheelRight.Set(-1 * rightInput);

			time = timer.Get();
			printf("drive wheel backwards test");
		}

		timer.Reset();
		timer.Start();
		time = timer.Get();
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
