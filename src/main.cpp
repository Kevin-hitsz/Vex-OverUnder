#include "main.h"
#include "RopoController.hpp"
#include "RopoDevice.hpp"
#include "RopoPosition.hpp"
#include "pros/misc.h"
#include "pros/rtos.hpp"

namespace ControllerModule{

	void BoolSwitch(void * Parameter){
		bool *p = static_cast<bool *>(Parameter);
		(*p) ^= 1;
	}

	void Push(){
		RopoDevice::ThreeWire::ExternPneumatic.set_value(true);
	}

	void Pull(){
		RopoDevice::ThreeWire::ExternPneumatic.set_value(false);
	}

	bool externFlag = false;
	void Switch(){
		externFlag ^= 1; 
		RopoDevice::ThreeWire::ExternPneumatic.set_value(externFlag);
	}

	bool locktag = false;
	void ChangeCatch(){
		locktag ^= 1;
		RopoDevice::ThreeWire::CatchPneumatic.set_value(locktag);
	}

	void RumbleMe(){
		pros::Controller MasterController1(pros::E_CONTROLLER_MASTER);
		MasterController1.rumble("-.-.-");
		FloatType aa = pros::millis();
		while(pros::millis()-aa<45000){
			pros::delay(100);
		}
		MasterController1.rumble("-.-.-");
		while(pros::millis()-aa<65000){
			pros::delay(100);
		}
		MasterController1.rumble("-.-.-");
	}
	int catch_1 = 0;
	void Hold(){
		RopoDevice::LiftMotors.Hold();
		catch_1 = 1;
	}

	void Lift(){
		RopoDevice::LiftMotors.Wait();
		catch_1 = 2;
	}

	void Hide(){
		RopoDevice::LiftMotors.Hide();
		catch_1 = 0;
	}

	void ChangeLift(){
		if (catch_1 == 1) {
			Hide();
		} else {
			Hold();
		}
	}

	// void ControllerPrint(){
	// 	while(true) {
	// 		pros::Controller MasterController(pros::E_CONTROLLER_MASTER);
	// 		MasterController.print(0,1,"degree: %.1lf",-RopoDevice::Sensors::Inertial.get_yaw());
	// 		pros::delay(10); 
	// 		MasterController.print(1,1,"X: %.2lf Y:%.2lf",RopoDevice::Position_Motor::MyPosition.Get_X(),RopoDevice::Position_Motor::MyPosition.Get_Y());
	// 		pros::delay(10); 
	// 		MasterController.print(2,1,"%.2lf  %d",RopoDevice::LiftMotors.GetLifterPosition(), RopoDevice::LiftMotors.GetLifterStatus());
	// 		pros::delay(10);
	// 	}
	// }
}

void initialize() {
	pros::lcd::initialize();
	
	pros::delay(50);
	RopoDevice::DeviceInit();
	RopoDevice::MotorsInit();
	RopoDevice::Position_Motor::MyPosition.initial();
}

void disabled() {}

void competition_initialize() {}

void autonomous(){
	
}

void opcontrol()
{

		

	pros::Task *RumbleTask = new pros::Task(ControllerModule::RumbleMe);
	// pros::Task *PrintTask = new pros::Task(ControllerModule::ControllerPrint);
	pros::Controller MasterController(pros::E_CONTROLLER_MASTER);
	RopoController::ButtonTaskLine ButtonDetectLine(MasterController);
	FloatType VelocityMax = 2.3;
	FloatType RopoWcLimit = 3.5;
	bool ChassisMove = false;
	
	RopoController::AxisValueCast XVelocityInput(MasterController,pros::E_CONTROLLER_ANALOG_LEFT_Y,RopoController::Linear);
	RopoController::AxisValueCast WVelocityInput(MasterController,pros::E_CONTROLLER_ANALOG_RIGHT_X,RopoController::Linear);

	Vector Velocity(RopoMath::ColumnVector,2),ResVelocity;

	MasterController.clear();

	ButtonDetectLine.AddButtonDetect(pros::E_CONTROLLER_DIGITAL_L1, RopoController::Rising, ControllerModule::ChangeLift);
	ButtonDetectLine.AddButtonDetect(pros::E_CONTROLLER_DIGITAL_L2, RopoController::Rising, ControllerModule::Lift);
	ButtonDetectLine.AddButtonDetect(pros::E_CONTROLLER_DIGITAL_R1,RopoController::Rising,ControllerModule::ChangeCatch);
	ButtonDetectLine.AddButtonDetect(pros::E_CONTROLLER_DIGITAL_B, RopoController::Rising, ControllerModule::Push);
	ButtonDetectLine.AddButtonDetect(pros::E_CONTROLLER_DIGITAL_B, RopoController::Falling, ControllerModule::Pull);
	ButtonDetectLine.AddButtonDetect(pros::E_CONTROLLER_DIGITAL_R2, RopoController::Rising, ControllerModule::Switch);
	ButtonDetectLine.Enable();

	while (true) {
		FloatType XInput =  XVelocityInput.GetAxisValue();
		FloatType WInput = -WVelocityInput.GetAxisValue();
		FloatType RopoWc = RopoWcLimit-XInput*0.6;			

		if (fabs(XInput) <= 0.06 && fabs(WInput) <= 0.06 ) {
			Velocity[1] = Velocity[2] = 0;
			if(ChassisMove)
				RopoDevice::Chassis.MoveVelocity(Velocity);
			ChassisMove = false;
		} else {
			Velocity[1] = XInput * VelocityMax;
			Velocity[2] = WInput * RopoWc;
			RopoDevice::Chassis.MoveVelocity(Velocity);
			ChassisMove = true;
		}
		pros::delay(4);
	}
}

