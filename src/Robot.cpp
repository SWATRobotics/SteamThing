#include "WPIlib.h"
//#include "Command/Command.h"
//#include "Command/ExampleCommand.h"
#include "CommandBase.h"

/*
 * I/O map
 * Talons:
 * robotDriveFront
 * robotDriveFront
 * robotDriveRear
 * midwheelRight
 * midwheelLeft
 *
 */

// Once Robot is wired, double-check what things are named as

class Robot: public IterativeRobot {

	CANTalon talon0;
	CANTalon talon1;
	CANTalon talon2;
	CANTalon talon3;
	CANTalon talon4;
	CANTalon talon5;
	RobotDrive robotDriveFront;
	RobotDrive robotDriveRear;
	Joystick jsL;
	Joystick jsR;
	Joystick jsG;

public:
	Robot() :
		talon0(0),
		talon1(1),
		talon2(2),
		talon3(3),
		talon4(4),
		talon5(5),
		robotDriveFront(talon0 , talon2),
		robotDriveRear(talon3, talon5),
		jsL(0),
		jsR(1),
		jsG(2) {
	}


private:
	void RobotInit() {
		talon0.SetControlMode(CANSpeedController::kPercentVbus);
		talon1.SetControlMode(CANSpeedController::kPercentVbus);
		talon2.SetControlMode(CANSpeedController::kPercentVbus);
		talon3.SetControlMode(CANSpeedController::kPercentVbus);
		talon4.SetControlMode(CANSpeedController::kPercentVbus);
		talon5.SetControlMode(CANSpeedController::kPercentVbus);
	}

	void TeleopInit() {
		//this->cps.Start();
	}

	void Drive() {
		float leftInput = jsL.GetY();
		float rightInput = jsR.GetY();
		float leftMidwheelInput = jsL.GetY();
		float rightMidwheelInput = jsR.GetY();
		robotDriveFront.SetLeftRightMotorOutputs(leftInput, rightInput);
		robotDriveRear.SetLeftRightMotorOutputs(leftInput, rightInput);
		talon1.Set(leftMidwheelInput);
		talon4.Set(rightMidwheelInput);

	}

	void TeleopPeriodic() {
		this->Drive();
	}

	void TestPeriodic() {

	}

};

START_ROBOT_CLASS(Robot)
