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

	
	//RopoDevice::LF.Initialize();

	pros::lcd::print(1, "111");

	while (true) {
		FloatType XInput =  XVelocityInput.GetAxisValue();
		FloatType YInput =  YVelocityInput.GetAxisValue();
		FloatType WInput = -WVelocityInput.GetAxisValue();	

		// RopoDevice::Chassis.SetAimStatus(XInput, YInput, WInput);
		// RopoDevice::LB.SetAimStatus(0.5, RopoMath::Pi / 3);
		// RopoDevice::LF.SetAimStatus(0.6, RopoMath::Pi/3 );
		// pros::lcd::print(1, "%f,%f,%f", RopoDiffySwerve::V1,RopoDiffySwerve::V2,RopoDiffySwerve::v_error);
		// pros::lcd::print(2, "%f,%f,%f", RopoDiffySwerve::A,RopoDiffySwerve::A_,RopoDiffySwerve::V);
		// MasterController.print(1,1,"%f,%f",RopoChassis::V2,RopoChassis::O2);

		pros::delay(20);
	}
}
