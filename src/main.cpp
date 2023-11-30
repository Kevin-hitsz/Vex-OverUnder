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
		RopoDevice::ThreeWire::RightExternPneumatic.set_value(true);
		RopoDevice::ThreeWire::LeftExternPneumatic.set_value(true);
	}

	void Pull(){
		RopoDevice::ThreeWire::RightExternPneumatic.set_value(false);
		RopoDevice::ThreeWire::LeftExternPneumatic.set_value(false);
	}

	void Catch(){
		RopoDevice::ThreeWire::CatchPneumatic.set_value(true);
	}

	void Lock(){
		RopoDevice::ThreeWire::LockPneumatic.set_value(true);
	}
	
	void RumbleMe(){
		pros::Controller MasterController1(pros::E_CONTROLLER_MASTER);
		MasterController1.rumble("-.-.-");
		RopoApi::FloatType aa = pros::millis();
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

	// static bool LifterR1Tag = false;
	// static bool LifterR2Tag = false;

	// void LifterStatusUpdate(){
	// 	if (LifterR1Tag && !LifterR2Tag)
	// 		RopoDevice::Motors::LiftMotor.move_velocity(100);
	// 	else if (LifterR2Tag && !LifterR1Tag)
	// 		RopoDevice::Motors::LiftMotor.move_velocity(-70);
	// 	else
	// 		RopoDevice::Motors::LiftMotor.move_velocity(0);			// 逻辑有待优化！！！！！
	// 	pros::delay(20);
	// }

	// void LifterBoolSwitch(void *param){
	// 	bool *p = static_cast<bool *>(param);
	// 	(*p) ^= 1;
	// 	LifterStatusUpdate();
	// }

	void ControllerPrint(){
		while(true) {
			pros::Controller MasterController(pros::E_CONTROLLER_MASTER);
			MasterController.print(0,1,"degree: %.1lf",-RopoDevice::Sensors::Inertial.get_yaw());
			pros::delay(10); 
			MasterController.print(1,1,"X: %.2lf Y:%.2lf",RopoDevice::Position_Motor::MyPosition.Get_X(),RopoDevice::Position_Motor::MyPosition.Get_Y());
			pros::delay(10); 
			MasterController.print(2,1,"%.2lf  %d",RopoDevice::LiftMotors.GetLifterPosition(), RopoDevice::LiftMotors.GetLifterStatus());
			pros::delay(10);
		}
	}
}

void initialize() {
	pros::lcd::initialize();
	pros::delay(10);
	RopoDevice::MotorsInit();
	RopoDevice::Position_Motor::MyPosition.initial();
	RopoDevice::DeviceInit();
}

void disabled() {}

void competition_initialize() {}

void autonomous() {
	// ------- Stage 1 - Catch the triball under the lift bar -------
	RopoDevice::LiftMotors.Hold();
	pros::delay(300);
	RopoDevice::Chassis.MoveVelocity(1.5,0);
	pros::delay(550);
	
	RopoDevice::Chassis.MoveVelocity(1.9,0.9);
	pros::delay(1100);
	RopoDevice::Chassis.MoveVelocity(0,0);
	pros::delay(700);
	RopoDevice::Chassis.MoveVelocity(-1,0);
	pros::delay(300);
	RopoDevice::LiftMotors.Hide();
	RopoDevice::Chassis.MoveVelocity(0,0);
	pros::delay(300);
	// RopoDevice::Chassis.AutoRotateAbs(-60.0);
	RopoDevice::Chassis.AutoMovePosAbs(1.63,0.303,-50.0);
	while (!RopoDevice::Chassis.IfDegArrived()){
		pros::delay(20);
	}
	RopoDevice::Chassis.MoveVelocity(0.4,0);
	pros::delay(500);
	RopoDevice::Chassis.MoveVelocity(0,0);
	pros::delay(100);
	RopoDevice::LiftMotors.Wait();
	pros::delay(600);
	RopoDevice::LiftMotors.Hold();
	RopoDevice::Chassis.MoveVelocity(-0.8,0);
	pros::delay(625);
	RopoDevice::Chassis.AutoMovePosAbsBack(0.92,1.43,-3);
	while (!RopoDevice::Chassis.IfDegArrived()){
		pros::delay(20);
	}
	// RopoDevice::Chassis.MoveVelocity(2,1.2);
	// pros::delay(600);

	// RopoDevice::Chassis.AutoRotateAbs(90.0);
	// while (!RopoDevice::Chassis.IfDegArrived()){
	// 	pros::delay(20);
	// }
	// RopoDevice::Chassis.MoveVelocity(1.7,0);
	// pros::delay(550);

	// // ------- Stage 2 - Catch the union triball -------
	// RopoDevice::Chassis.MoveVelocity(-0.8,0);
	// pros::delay(550);
	// RopoDevice::Chassis.MoveVelocity(-1,-0.6);
	// pros::delay(550);
	// ControllerModule::Hide();
	// pros::delay(500);
	// RopoDevice::Chassis.AutoMovePosAbs(1.4,0.2,-45);
	// while (!RopoDevice::Chassis.IfArrived()){
	//  	pros::delay(20);
	// }
	// RopoDevice::Chassis.MoveVelocity(0.4,0);
	// pros::delay(200);
	// ControllerModule::Hold();
	// pros::delay(900);
	// RopoDevice::Chassis.MoveVelocity(-1,0);
	// pros::delay(500);
	// RopoDevice::Chassis.MoveVelocity(0,0);
	// pros::delay(200);

	// RopoDevice::Chassis.AutoMovePosAbsBack(0.6,1.2,0);
	// while (!RopoDevice::Chassis.IfArrived()){
	//  	pros::delay(20);
	// }
	// RopoDevice::Chassis.MoveVelocity(1,0);
	// pros::delay(600);
	// RopoDevice::Chassis.MoveVelocity(0,0);
	// pros::delay(200);
	// RopoDevice::Chassis.AutoMovePosAbsBack(0.6,1.2,0);
	// while (!RopoDevice::Chassis.IfArrived()){
	//  	pros::delay(20);
	// }
	//RopoDevice::Chassis.MoveVelocity(0,2);

	// ------- End -------
	RopoDevice::Chassis.MoveVelocity(0,0);
}



void opcontrol() {
	pros::Task *RumbleTask = new pros::Task(ControllerModule::RumbleMe);
	pros::Task *PrintTask = new pros::Task(ControllerModule::ControllerPrint);
	pros::Controller MasterController(pros::E_CONTROLLER_MASTER);
	RopoController::ButtonTaskLine ButtonDetectLine(MasterController);
	RopoApi::FloatType VelocityMax = 2.3;
	RopoApi::FloatType RopoWcLimit = 3.5;
	bool ChassisMove = false;
	
	RopoController::AxisValueCast XVelocityInput(MasterController,pros::E_CONTROLLER_ANALOG_LEFT_Y,RopoController::Linear);
	RopoController::AxisValueCast WVelocityInput(MasterController,pros::E_CONTROLLER_ANALOG_RIGHT_X,RopoController::Linear);
	RopoMath::Vector<RopoApi::FloatType> Velocity(RopoMath::ColumnVector,2),ResVelocity;
	MasterController.clear();
	ButtonDetectLine.AddButtonDetect(pros::E_CONTROLLER_DIGITAL_Y  , RopoController::Rising,  autonomous);
	ButtonDetectLine.AddButtonDetect(pros::E_CONTROLLER_DIGITAL_L1, RopoController::Rising, ControllerModule::ChangeLift);
	ButtonDetectLine.AddButtonDetect(pros::E_CONTROLLER_DIGITAL_L2, RopoController::Rising, ControllerModule::Lift);
	// ButtonDetectLine.AddButtonDetect(pros::E_CONTROLLER_DIGITAL_R1, RopoController::DoubleEdge, ControllerModule::LifterBoolSwitch, &ControllerModule::LifterR1Tag);
	// ButtonDetectLine.AddButtonDetect(pros::E_CONTROLLER_DIGITAL_R2, RopoController::DoubleEdge, ControllerModule::LifterBoolSwitch, &ControllerModule::LifterR2Tag);
	ButtonDetectLine.AddButtonDetect(pros::E_CONTROLLER_DIGITAL_B, RopoController::Rising, ControllerModule::Push);
	ButtonDetectLine.AddButtonDetect(pros::E_CONTROLLER_DIGITAL_B, RopoController::Falling, ControllerModule::Pull);
	ButtonDetectLine.Enable();

	while (true) {
		RopoApi::FloatType XInput =  XVelocityInput.GetAxisValue();
		RopoApi::FloatType WInput = -WVelocityInput.GetAxisValue();
		RopoApi::FloatType RopoWc = RopoWcLimit-XInput*0.6;			

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

		pros::lcd::print(1,"Ready!!! V:%.1f %.1f",XInput,WInput);
		pros::delay(4);
	}
}
