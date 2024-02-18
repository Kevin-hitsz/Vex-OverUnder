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
	void Intake(){
		RopoDevice::Motors::IntakeMotor.move_velocity(600);
	}
	void Outtake(){
		RopoDevice::Motors::IntakeMotor.move_velocity(-600);
	}
	void StopIn(){
		RopoDevice::Motors::IntakeMotor.move_velocity(0);
	}
	void Extern(){
		RopoDevice::ThreeWire::ExternPneumatic.set_value(true);
	}
	void Recycle(){
		RopoDevice::ThreeWire::ExternPneumatic.set_value(false);
	}
	void ClimberUp(){
		RopoDevice::Motors::ClimberMotor1.move_velocity(600);
		RopoDevice::Motors::ClimberMotor2.move_velocity(600);
	}
	void ClimberDown(){
		RopoDevice::Motors::ClimberMotor1.move_velocity(-600);
		RopoDevice::Motors::ClimberMotor2.move_velocity(-600);
	}
	void ClimberStop(){
		RopoDevice::Motors::ClimberMotor1.move_velocity(0);
		RopoDevice::Motors::ClimberMotor2.move_velocity(0);
	}
	bool DetectLoader(){
		if(RopoDevice::Motors::ShooterMotor.get_torque() < 0.4) return false;
		else return true;
	}
	void ReLoad(){
		while(1){
			if(DetectLoader() == false){
				RopoDevice::Motors::ShooterMotor.move_velocity(30);
			}
			else {
				RopoDevice::Motors::ShooterMotor.move_velocity(30); 
				pros::delay(3000);
				RopoDevice::Motors::ShooterMotor.brake();
				break;
			}
		}
	}
	void Shoot(){
		RopoDevice::Motors::ShooterMotor.move_velocity(30);
		pros::delay(1000);
		RopoDevice::Motors::ShooterMotor.move_velocity(0);
	}
}

void disabled() {}

void competition_initialize() {}

void autonomous(){
	RopoDevice::Chassis.AutoStart();
	pros::Task * BackgroundTaskPtr = new pros::Task(RopoDevice::PositionControl);
	RopoDevice::SetPosition(0.32,0.40,41.7976 / 180 * RopoMath::Pi,2000);
	RopoFunction::Extern();
	RopoDevice::SetPosition(100, 100, 18.62 / 180 * RopoMath::Pi,600);
	RopoFunction::Recycle();
	RopoDevice::SetPosition(0.39, 0.37, -54.66 / 180 * RopoMath::Pi, 1500);
	RopoDevice::SetPosition(0.78, 0.62, -90.21 / 180 * RopoMath::Pi, 2000);
	RopoDevice::SetPosition(0.23, 0.33, -47.07 / 180 * RopoMath::Pi, 1500);


	RopoDevice::Chassis.Autoend();
	delete BackgroundTaskPtr;
}

void opcontrol() {
	pros::Controller MasterController(pros::E_CONTROLLER_MASTER);
	RopoController::ButtonTaskLine ButtonDetectLine(MasterController);
	FloatType VelocityMax = 2.3;
	FloatType RopoWcLimit = 3.5;
	
	RopoController::AxisValueCast XVelocityInput(MasterController,pros::E_CONTROLLER_ANALOG_LEFT_Y,RopoController::Linear);
	RopoController::AxisValueCast YVelocityInput(MasterController,pros::E_CONTROLLER_ANALOG_LEFT_X,RopoController::Linear);
	RopoController::AxisValueCast WVelocityInput(MasterController,pros::E_CONTROLLER_ANALOG_RIGHT_X,RopoController::Linear);
	Vector Velocity(RopoMath::ColumnVector,2),ResVelocity;
	MasterController.clear();
	ButtonDetectLine.AddButtonDetect(pros::E_CONTROLLER_DIGITAL_A , RopoController::Rising, autonomous);
	ButtonDetectLine.AddButtonDetect(pros::E_CONTROLLER_DIGITAL_R1, RopoController::Rising, RopoFunction::Intake);
	ButtonDetectLine.AddButtonDetect(pros::E_CONTROLLER_DIGITAL_R2, RopoController::Rising, RopoFunction::Outtake);
	ButtonDetectLine.AddButtonDetect(pros::E_CONTROLLER_DIGITAL_R1, RopoController::Falling, RopoFunction::StopIn);
	ButtonDetectLine.AddButtonDetect(pros::E_CONTROLLER_DIGITAL_R2, RopoController::Falling, RopoFunction::StopIn);
	ButtonDetectLine.AddButtonDetect(pros::E_CONTROLLER_DIGITAL_L1, RopoController::Rising, RopoFunction::Extern);
	ButtonDetectLine.AddButtonDetect(pros::E_CONTROLLER_DIGITAL_L1, RopoController::Falling, RopoFunction::Recycle);
	ButtonDetectLine.AddButtonDetect(pros::E_CONTROLLER_DIGITAL_L2, RopoController::Rising, RopoFunction::ClimberDown);
	ButtonDetectLine.AddButtonDetect(pros::E_CONTROLLER_DIGITAL_L2, RopoController::Falling, RopoFunction::ClimberStop);
	ButtonDetectLine.AddButtonDetect(pros::E_CONTROLLER_DIGITAL_UP, RopoController::Rising, RopoFunction::ClimberUp);
	ButtonDetectLine.AddButtonDetect(pros::E_CONTROLLER_DIGITAL_UP, RopoController::Falling, RopoFunction::ClimberStop);
	ButtonDetectLine.AddButtonDetect(pros::E_CONTROLLER_DIGITAL_X, RopoController::Rising, RopoFunction::ReLoad);
	ButtonDetectLine.AddButtonDetect(pros::E_CONTROLLER_DIGITAL_Y, RopoController::Rising, RopoFunction::Shoot);
	ButtonDetectLine.Enable();

	while (true) {
		FloatType XInput =  2 * XVelocityInput.GetAxisValue();
		FloatType YInput =  2 * YVelocityInput.GetAxisValue();
		FloatType WInput = -3 * WVelocityInput.GetAxisValue();	
		if(RopoDevice::Chassis.IsOpcontrol()) RopoDevice::Chassis.SetAimStatus(XInput, YInput, WInput);
		MasterController.print(0,0,"%.2f, %.2f, %.2f", RopoDevice::Sensors::GetPosition()[1][1], RopoDevice::Sensors::GetPosition()[2][1], RopoDevice::Sensors::GetPosition()[3][1] / RopoMath::Pi * 180.0);
		pros::delay(20);
	}
}