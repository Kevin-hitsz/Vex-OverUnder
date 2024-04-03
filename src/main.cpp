#include "main.h"
#include "RopoApi.hpp"
#include "RopoChassis.hpp"
#include "RopoController.hpp"
#include "RopoDevice.hpp"
#include "RopoMath/Misc.hpp"
#include "pros/llemu.hpp"
#include "pros/misc.h"
#include "pros/rtos.hpp"
#include <cmath>

namespace ControllerModule{
}

void initialize() {
	RopoDevice::DeviceIni();
	pros::lcd::initialize();
	pros::delay(10);
}

namespace RopoFunction{
	void ShooterPneumaticTest(){
		RopoDevice::ThreeWire::ShooterPneumatic.set_value(true);
	}
	void Intake(){
		RopoDevice::Motors::IntakeMotor.move_velocity(-600);
	}
	void Outtake(){
		RopoDevice::Motors::IntakeMotor.move_velocity(600);
	}
	void StopIn(){
		RopoDevice::Motors::IntakeMotor.move_velocity(0);
	}


	void ExternLeft(){
		RopoDevice::ThreeWire::LExternPneumatic.set_value(true);
	}
	void RecycleLeft(){
		RopoDevice::ThreeWire::LExternPneumatic.set_value(false);
	}
	void ExternRight(){
		RopoDevice::ThreeWire::RExternPneumatic.set_value(true);
	}
	void RecycleRight(){
		RopoDevice::ThreeWire::RExternPneumatic.set_value(false);
	}
	

	void ClimberUp(){
		RopoDevice::ThreeWire::ClimberPneumatic.set_value(true);
		
	}
	void ClimberRecycle(){
		RopoDevice::ThreeWire::ClimberPneumatic.set_value(false);
	}


	void IntakerUp(){
		RopoDevice::ThreeWire::IntakerPneumatic.set_value(true);
	}
	void IntakerDown(){
		RopoDevice::ThreeWire::IntakerPneumatic.set_value(false);
	}


	void ShooterInit(){
		RopoDevice::Motors::LShooterMotor.move_velocity(-30);
		RopoDevice::Motors::RShooterMotor.move_velocity(30);
	}
	void ShooterStopInit(){
		RopoDevice::Motors::LShooterMotor.move_velocity(0);
		RopoDevice::Motors::RShooterMotor.move_velocity(0);
	}
	bool DetectLoader(){
		if(RopoDevice::Motors::LShooterMotor.get_torque() < 0.2 && RopoDevice::Motors::RShooterMotor.get_torque() < 0.2) return false;
		else return true;
	}
	void ReLoad(){
		RopoDevice::Motors::LShooterMotor.move_voltage(-12000);
		RopoDevice::Motors::RShooterMotor.move_voltage(12000);
		pros::delay(200);
		while (RopoDevice::Motors::RShooterMotor.get_actual_velocity() > 5) pros::delay(5);
		ShooterStopInit();
	}
	void Shoot(){
		RopoDevice::Motors::LShooterMotor.move_voltage(12000);
		RopoDevice::Motors::RShooterMotor.move_voltage(-12000);
		pros::delay(200);
		while (RopoDevice::Motors::LShooterMotor.get_actual_velocity() > 2) pros::delay(5);
		ShooterStopInit();
	}
	void Hit(){
		RopoDevice::Motors::HitterMotor.move_voltage(-12000);
		pros::delay(200);
		while (std::abs(RopoDevice::Motors::HitterMotor.get_actual_velocity()) > 5) pros::delay(5);
		RopoDevice::Motors::HitterMotor.move_voltage(0);
	}
	void HitterReset(){
		RopoDevice::Motors::HitterMotor.move_voltage(12000);
		pros::delay(200);
		while (RopoDevice::Motors::HitterMotor.get_actual_velocity() > 5) pros::delay(5);
		RopoDevice::Motors::HitterMotor.move_voltage(0);
	}
	void Test(){
		RopoDevice::Chassis.AutoSetPosition(0,0,90 * RopoMath::Pi / 180.0,2000);
	}
	void RelativeControlMode(){
		RopoDevice::Chassis.SetRelativeMode();
	}
	void AbsoluteControlMode(){
		RopoDevice::Chassis.SetAbsoluteMode();
	}
	void ChangeControlMode(){
		static bool ControlMode = true;
		if(ControlMode){
			AbsoluteControlMode();
		}else{
			RelativeControlMode();
		}
		ControlMode = !ControlMode;
	}
	void Import(){
		Shoot();
		pros::delay(100);
		Hit();
		int i = 0;
		for(;i<=4;i++){
			pros::delay(200);
			HitterReset();
			pros::delay(100);
			ReLoad();
			pros::delay(800);
			Shoot();
			pros::delay(200);
			Hit();
		}
		pros::delay(200);
		HitterReset();
	}
}






void autonomous_A1(){
	

	/*RopoDevice::Chassis.OpSetAimStatus(XInput, YInput, WInput);
	RopoDevice::Chassis.AutoSetPosition();*/
	float x,y,theta;
	RopoFunction::ReLoad();
	RopoFunction::HitterReset();

	pros::delay(1000);


	RopoFunction::Intake();
 	RopoDevice::Chassis.AutoSetPosition(0,5,0,100);
	RopoFunction::StopIn();
	RopoFunction::ExternRight();
	RopoDevice::Chassis.AutoSetPosition(0,-40,0,100);
	RopoDevice::Chassis.AutoSetPosition(0,-40,45,100);
	RopoFunction::ExternLeft();
	RopoDevice::Chassis.AutoSetPosition(5,-45,45,100);
	RopoDevice::Chassis.AutoSetPosition(5,-45,90,100);
	RopoDevice::Chassis.AutoSetAimStatus(0, -20, 0);
	RopoDevice::Chassis.AutoSetAimStatus(0, 10, 0);
	RopoDevice::Chassis.AutoSetAimStatus(0, -20, 0);
	RopoDevice::Chassis.AutoSetAimStatus(0, 10, 0);

	x = RopoDevice::Chassis.GetX();
	y = RopoDevice::Chassis.GetY();
	RopoDevice::Chassis.AutoSetPosition(x,y,-90,100);
	
	RopoDevice::Chassis.AutoSetAimStatus(0, 20, 0);




















}






void disabled() {}

void competition_initialize() {}

void skill() {
	
}

void autonomous(){
}

void opcontrol() {
	pros::Controller MasterController(pros::E_CONTROLLER_MASTER);
	RopoController::ButtonTaskLine ButtonDetectLine(MasterController);
	FloatType VelocityMax = 2.3;
	FloatType RopoWcLimit = 3.5;

	/*FloatType LastXInput = 0;
	FloatType LastYInput = 0;
	FloatType LastWInput = 0;
	FloatType XInput = 0;
	FloatType YInput = 0;
	FloatType WInput = 0;*/
	
	RopoController::AxisValueCast XVelocityInput(MasterController,pros::E_CONTROLLER_ANALOG_LEFT_Y,RopoController::Linear);
	RopoController::AxisValueCast YVelocityInput(MasterController,pros::E_CONTROLLER_ANALOG_LEFT_X,RopoController::Linear);
	RopoController::AxisValueCast WVelocityInput(MasterController,pros::E_CONTROLLER_ANALOG_RIGHT_X,RopoController::Linear);
	Vector Velocity(RopoMath::ColumnVector,2),ResVelocity;
	MasterController.clear();

	//ButtonDetectLine.AddButtonDetect(pros::E_CONTROLLER_DIGITAL_A , RopoController::Rising, autonomous_A1);

	ButtonDetectLine.AddButtonDetect(pros::E_CONTROLLER_DIGITAL_R1, RopoController::Rising, RopoFunction::Intake);
	//ButtonDetectLine.AddButtonDetect(pros::E_CONTROLLER_DIGITAL_R1, RopoController::Rising, RopoFunction::Outtake);
	ButtonDetectLine.AddButtonDetect(pros::E_CONTROLLER_DIGITAL_R1, RopoController::Falling, RopoFunction::StopIn);
	//ButtonDetectLine.AddButtonDetect(pros::E_CONTROLLER_DIGITAL_R1, RopoController::Falling, RopoFunction::StopIn);

	ButtonDetectLine.AddButtonDetect(pros::E_CONTROLLER_DIGITAL_L2, RopoController::Rising, RopoFunction::ExternLeft);
	ButtonDetectLine.AddButtonDetect(pros::E_CONTROLLER_DIGITAL_L2, RopoController::Falling, RopoFunction::RecycleLeft);
	ButtonDetectLine.AddButtonDetect(pros::E_CONTROLLER_DIGITAL_L1, RopoController::Rising, RopoFunction::ExternRight);
	ButtonDetectLine.AddButtonDetect(pros::E_CONTROLLER_DIGITAL_L1, RopoController::Falling, RopoFunction::RecycleRight);

	ButtonDetectLine.AddButtonDetect(pros::E_CONTROLLER_DIGITAL_UP, RopoController::Rising, RopoFunction::ClimberUp);
	ButtonDetectLine.AddButtonDetect(pros::E_CONTROLLER_DIGITAL_UP, RopoController::Falling, RopoFunction::ClimberRecycle);

	ButtonDetectLine.AddButtonDetect(pros::E_CONTROLLER_DIGITAL_Y, RopoController::Rising, RopoFunction::IntakerUp);
	ButtonDetectLine.AddButtonDetect(pros::E_CONTROLLER_DIGITAL_Y, RopoController::Falling, RopoFunction::IntakerDown);


	ButtonDetectLine.AddButtonDetect(pros::E_CONTROLLER_DIGITAL_RIGHT, RopoController::Rising, RopoFunction::ShooterInit);
	ButtonDetectLine.AddButtonDetect(pros::E_CONTROLLER_DIGITAL_RIGHT, RopoController::Falling, RopoFunction::ShooterStopInit);
	ButtonDetectLine.AddButtonDetect(pros::E_CONTROLLER_DIGITAL_B, RopoController::Rising, RopoFunction::ReLoad);
	ButtonDetectLine.AddButtonDetect(pros::E_CONTROLLER_DIGITAL_X, RopoController::Rising, RopoFunction::Shoot);

	ButtonDetectLine.AddButtonDetect(pros::E_CONTROLLER_DIGITAL_LEFT, RopoController::Rising, RopoFunction::Hit);
	ButtonDetectLine.AddButtonDetect(pros::E_CONTROLLER_DIGITAL_LEFT, RopoController::Falling, RopoFunction::HitterReset);
	ButtonDetectLine.AddButtonDetect(pros::E_CONTROLLER_DIGITAL_DOWN, RopoController::Rising, RopoFunction::ShooterPneumaticTest);
	ButtonDetectLine.AddButtonDetect(pros::E_CONTROLLER_DIGITAL_A, RopoController::Rising, RopoFunction::Test);
	ButtonDetectLine.AddButtonDetect(pros::E_CONTROLLER_DIGITAL_R2, RopoController::Rising, RopoFunction::Import);
	ButtonDetectLine.AddButtonDetect(pros::E_CONTROLLER_DIGITAL_X, RopoController::DoubleClick, RopoFunction::ChangeControlMode);
	ButtonDetectLine.Enable();
	RopoDevice::Chassis.Operator();
	while (true) {

		/*XInput = RopoMath::LowPassFilter(3 * XVelocityInput.GetAxisValue(),LastXInput,50,1000);
		YInput = RopoMath::LowPassFilter(3 * YVelocityInput.GetAxisValue(),LastYInput,50,1000);
		WInput = RopoMath::LowPassFilter(-5 * WVelocityInput.GetAxisValue(),LastWInput,50,1000);*/



		FloatType XInput =  3 * XVelocityInput.GetAxisValue();
		FloatType YInput =  3 * YVelocityInput.GetAxisValue();
		FloatType WInput = -15 * WVelocityInput.GetAxisValue();

		/*FloatType YInput = 1;
		FloatType XInput = 0;
		FloatType WInput = 0;*/

		if(RopoDevice::Chassis.IsOpcontrol()) RopoDevice::Chassis.OpSetAimStatus(XInput, YInput, WInput);

		/*LastXInput = XInput;
		LastYInput = YInput;
		LastWInput = WInput;*/

		//MasterController.print(0,0,"%.2f, %.2f, %.2f", RopoDevice::Chassis.GetX(), RopoDevice::Chassis.GetY(), RopoDevice::Chassis.GetTheta() * 180 / RopoMath::Pi);
		//MasterController.print(0,0,"%.2f, %.2f", 180 * RopoDevice::LF.get_Angle() / RopoMath::Pi, 180 * RopoDevice::LB.get_Angle() / RopoMath::Pi);
		//MasterController.print(1,0,"%.2f, %.2f", 180 * RopoDevice::RF.get_Angle() / RopoMath::Pi, 180 * RopoDevice::RB.get_Angle() / RopoMath::Pi);
		//MasterController.print(0,0,"%.2f, %.2f, %.2f", RopoDevice::Chassis.GetSwerveAimStatus(2,1) * 180 / RopoMath::Pi,RopoDevice::Chassis.GetSwerveAimStatus(4,1) * 180 / RopoMath::Pi,RopoDevice::Chassis.GetSwerveAimStatus(6,1) * 180 / RopoMath::Pi);
		MasterController.print(0,0,"%.2f, %.2f, %d", RopoDevice::Chassis.GetSwerveAimStatus(2,1) * 180 /RopoMath::Pi,RopoDevice::LF.x * 180 /RopoMath::Pi);
		//MasterController.print(0,0,"%.2f",RopoDevice::LF.x);
		//MasterController.print(0,0,"%.2f, %.2f, %.2f", RopoDevice::LF.GetStatusError(1),RopoDevice::LF.GetStatusError(2),RopoDevice::LF.GetStatusError(3));
		pros::delay(5);
	}
}