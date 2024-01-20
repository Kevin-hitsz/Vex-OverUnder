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
		
		FloatType XInput =  1.5 * XVelocityInput.GetAxisValue();
		FloatType YInput =  1.5 * YVelocityInput.GetAxisValue();
		FloatType WInput = -2 * WVelocityInput.GetAxisValue();	

		RopoDevice::Chassis.SetAimStatus(XInput, YInput, WInput);
		
		pros::lcd::print(5,"%.3f,%.3f,%.3f",XInput,YInput,WInput);

		pros::delay(20);
	}
}
