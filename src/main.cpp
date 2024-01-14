#include "main.h"
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
	bool ChassisMove = false;
	
	RopoController::AxisValueCast XVelocityInput(MasterController,pros::E_CONTROLLER_ANALOG_LEFT_Y,RopoController::Linear);
	RopoController::AxisValueCast WVelocityInput(MasterController,pros::E_CONTROLLER_ANALOG_RIGHT_X,RopoController::Linear);
	Vector Velocity(RopoMath::ColumnVector,2),ResVelocity;
	MasterController.clear();
	ButtonDetectLine.Enable();

	
	RopoDevice::LF.Initialize();

	while (true) {
		FloatType XInput =  XVelocityInput.GetAxisValue();
		FloatType WInput = -WVelocityInput.GetAxisValue();
		FloatType RopoWc = RopoWcLimit-XInput*0.6;			

		// if (fabs(XInput) <= 0.06 && fabs(WInput) <= 0.06 ) {
		// 	Velocity[1] = Velocity[2] = 0;
		// 	if(ChassisMove)
		// 		RopoDevice::Chassis.MoveVelocity(Velocity);
		// 	ChassisMove = false;
		// } else {
		// 	Velocity[1] = XInput * VelocityMax;
		// 	Velocity[2] = WInput * RopoWc;
		// 	RopoDevice::Chassis.MoveVelocity(Velocity);
		// 	ChassisMove = true;
		// }
		RopoDevice::LF.SetAimStatus(0.6, RopoMath::Pi/3 );
		// pros::lcd::print(1, "%f,%f,%f", RopoDiffySwerve::V1,RopoDiffySwerve::V2,RopoDiffySwerve::v_error);
		// pros::lcd::print(2, "%f,%f,%f", RopoDiffySwerve::A,RopoDiffySwerve::A_,RopoDiffySwerve::V);

		pros::delay(500);
	}
}
