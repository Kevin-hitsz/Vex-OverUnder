#include "main.h"
#include "RopoApi.hpp"
#include "RopoChassis.hpp"
#include "RopoController.hpp"
#include "RopoDevice.hpp"
#include "RopoDiffySwerve.hpp"
#include "RopoMath/Misc.hpp"
#include "pros/llemu.hpp"
#include "pros/misc.h"
#include "pros/rtos.hpp"
#include <cmath>

namespace ControllerModule{
}

void initialize() {
	pros::lcd::initialize();
	pros::delay(10);
}

void disabled() {}

void competition_initialize() {}

void autonomous(){}

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
	ButtonDetectLine.Enable();

	
	

	//pros::lcd::print(1, "111");
	RopoDevice::LF.Initialize();
	RopoDevice::LF.Start();

	while (true) {
		FloatType XInput =  1.8 * XVelocityInput.GetAxisValue();
		FloatType YInput =  1.8 * YVelocityInput.GetAxisValue();
		FloatType WInput = -2 * WVelocityInput.GetAxisValue();	

		RopoDevice::Chassis.SetAimStatus(XInput, YInput, WInput);

		// FloatType AimSpeed = sqrt(pow(XInput,2) + pow(YInput,2));
		// FloatType AimAngle = atan2(YInput,XInput);

		// if(AimSpeed == 0 && AimAngle == 0){
		// 	AimAngle = RopoMath::Pi /4;
		// }

		// RopoDevice::LF.SetAimStatus(AimSpeed, AimAngle);

		// pros::lcd::print(1, "%f,%f,%f", RopoDiffySwerve::V1,RopoDiffySwerve::V2,RopoDiffySwerve::v_error);
		//pros::lcd::print(2, "%f,%f,%f",RopoDevice::LF.A,RopoDevice::LF.A_,RopoDevice::LF.V);
		// MasterController.print(1,1,"%f,%f",RopoChassis::V2,RopoChassis::O2);

		// pros::lcd::print(1,"%.3f,%.3f",RopoDevice::LF.angle_error,RopoDevice::LF.v_error);
		// pros::lcd::print(2,"%.3f,%.3f",RopoDevice::LB.angle_error,RopoDevice::LB.v_error);
		// pros::lcd::print(3,"%.3f,%.3f",RopoDevice::RF.angle_error,RopoDevice::RF.v_error);
		// pros::lcd::print(4,"%.3f,%.3f",RopoDevice::RB.angle_error,RopoDevice::RB.v_error);

		// pros::lcd::print(1,"%.3f,%.3f",RopoDevice::LF.Get_1,RopoDevice::LF.Get_2);
		// pros::lcd::print(2,"%.3f,%.3f",RopoDevice::LB.Get_1,RopoDevice::LB.Get_2);
		// pros::lcd::print(3,"%.3f,%.3f",RopoDevice::RF.Get_1,RopoDevice::RF.Get_2);
		// pros::lcd::print(4,"%.3f,%.3f",RopoDevice::RB.Get_1,RopoDevice::RB.Get_2);

		pros::lcd::print(1,"%.3f,%.3f",RopoDevice::LF.A,RopoDevice::LF.V);
		pros::lcd::print(2,"%.3f,%.3f",RopoDevice::LB.A,RopoDevice::LB.V);
		pros::lcd::print(3,"%.3f,%.3f",RopoDevice::RF.A,RopoDevice::RF.V);
		pros::lcd::print(4,"%.3f,%.3f",RopoDevice::RB.A,RopoDevice::RB.V);
		
		pros::lcd::print(5,"%.3f,%.3f,%.3f",XInput,YInput,WInput);

		pros::delay(20);
	}
}
