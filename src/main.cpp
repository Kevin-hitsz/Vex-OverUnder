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
		RopoDevice::LiftMotors.Pull();
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

void autonomous_1() {
	// ------- Stage 1 - Catch the triball under the lift bar -------
	RopoDevice::LiftMotors.Hold();
	pros::delay(300);
	RopoDevice::Chassis.MoveVelocity(1.7,0);
	pros::delay(540);
	
	RopoDevice::Chassis.MoveVelocity(1.9,1.0);
	pros::delay(1100);
	RopoDevice::Chassis.MoveVelocity(0,0);
	RopoDevice::Position_Motor::MyPosition.Set_XY(1.83,0.822);
	pros::delay(700);
	RopoDevice::Chassis.MoveVelocity(-1,0);
	pros::delay(450);

	// ------- Stage 2 - Catch the union triball -------
	RopoDevice::LiftMotors.Hide();
	RopoDevice::Chassis.MoveVelocity(0,0);
	pros::delay(300);

	//RopoDevice::Chassis.AutoMovePosAbs(1.57,0.303,-50.0);	// 1.63
	RopoDevice::Chassis.AutoMovePosAbs(1.66,0.232,-46.0);	// 1.63
	while (!RopoDevice::Chassis.IfArrived()){
		pros::delay(20);
	}
	RopoDevice::Chassis.MoveVelocity(0.3,0);
	pros::delay(300);
	RopoDevice::Chassis.MoveVelocity(0,0);
	pros::delay(100);
	RopoDevice::LiftMotors.Hold();
	pros::delay(1200);
	RopoDevice::Chassis.MoveVelocity(-0.4,0);
	pros::delay(100);
	RopoDevice::LiftMotors.Pull();
	pros::delay(175);
	RopoDevice::LiftMotors.Hold();
	pros::delay(50);
	pros::delay(480);

	RopoDevice::Chassis.AutoMovePosAbsBack(0.97,1.43,-1);//0.92
	// RopoDevice::Chassis.AutoMovePosAbsBack(0.72,1.23,0);
	while (!RopoDevice::Chassis.IfArrived()){
		pros::delay(20);
	}
	RopoDevice::Chassis.MoveVelocity(1.3,0);
	pros::delay(600);
	RopoDevice::Chassis.MoveVelocity(0,0);
	RopoDevice::Position_Motor::MyPosition.Set_XY(1.65,1.32);
	pros::delay(300);
	RopoDevice::Chassis.MoveVelocity(-0.5,0);
	pros::delay(700);
	RopoDevice::Chassis.AutoMovePosAbsBack(1.15,1.35,0);	// 1.63
	while (!RopoDevice::Chassis.IfArrived()){
		pros::delay(20);
	}
	RopoDevice::Chassis.MoveVelocity(0,0);

	// ------- Stage 3 - Catch the triball - 1 -------
	RopoDevice::LiftMotors.Hide();
	pros::delay(300);
	RopoDevice::Chassis.AutoRotateAbs(90);
	while (!RopoDevice::Chassis.IfDegArrived()){
		pros::delay(20);
	}
	RopoDevice::LiftMotors.Hold();
	pros::delay(500);
	RopoDevice::Chassis.AutoRotateAbs(0);
	while (!RopoDevice::Chassis.IfDegArrived()){
		pros::delay(20);
	}
	pros::delay(300);
	RopoDevice::Chassis.MoveVelocity(1.3,0);
	pros::delay(420);
	RopoDevice::Chassis.MoveVelocity(0,0);
	RopoDevice::Position_Motor::MyPosition.Set_XY(1.72,1.382);
	pros::delay(600);
	RopoDevice::Chassis.MoveVelocity(-0.7,0);
	pros::delay(1000);

	// // ------- Stage 4 - Catch the triball - 2 ------
	// RopoDevice::LiftMotors.Hide();
	// pros::delay(500);
	// RopoDevice::Chassis.AutoMovePosAbs(0.95,1.55,135);
	// //RopoDevice::Chassis.AutoMovePosAbs(0.65,1.28,148);
	// while (!RopoDevice::Chassis.IfArrived()){
	// 	pros::delay(20);
	// }
	// pros::delay(300);
	// RopoDevice::LiftMotors.Pull();
	// pros::delay(1000);
	// RopoDevice::Chassis.MoveVelocity(0,-0.5);
	// pros::delay(600);
	// RopoDevice::LiftMotors.Hold();
	// RopoDevice::Chassis.AutoRotateAbs(0);
	// while (!RopoDevice::Chassis.IfDegArrived()){
	// 	pros::delay(20);
	// }

	// pros::delay(300);
	// RopoDevice::Chassis.MoveVelocity(1.2,0);
	// pros::delay(800);
	// RopoDevice::Chassis.MoveVelocity(0,0);
	// RopoDevice::Position_Motor::MyPosition.Set_XY(1.90,1.49);
	// pros::delay(700);
	// RopoDevice::Chassis.MoveVelocity(-0.5,0);
	// pros::delay(800);

	// // // ------- Stage 4 - Catch the triball - 3 ------
	// // RopoDevice::LiftMotors.Hide();
	// // pros::delay(500);
	// // RopoDevice::Chassis.AutoMovePosAbs(0.90,1.55,160);
	// // while (!RopoDevice::Chassis.IfArrived()){
	// // 	pros::delay(20);
	// // }
	// // pros::delay(300);
	// // RopoDevice::LiftMotors.Hold();
	// // pros::delay(1000);
	// // RopoDevice::Chassis.AutoRotateAbs(0);
	// // while (!RopoDevice::Chassis.IfDegArrived()){
	// // 	pros::delay(20);
	// // }

	// // pros::delay(300);
	// // RopoDevice::Chassis.MoveVelocity(1.3,0);
	// // pros::delay(1000);
	// // RopoDevice::Chassis.MoveVelocity(0,0);
	// // pros::delay(700);
	// // RopoDevice::Chassis.MoveVelocity(-0.8,0);
	// // pros::delay(800);

	//------- Stage 5 - Catch the triball - 4 ------
	
	RopoDevice::Chassis.AutoMovePosAbsBack(0.99,1.29,0);
	//RopoDevice::Chassis.AutoMovePosAbs(0.60,1.112,-125);
	while (!RopoDevice::Chassis.IfArrived()){
		pros::delay(20);
	}
	RopoDevice::LiftMotors.Hide();
	pros::delay(700);
	RopoDevice::Chassis.AutoRotateAbs(-130);
	while (!RopoDevice::Chassis.IfDegArrived()){
		pros::delay(20);
	}
	RopoDevice::LiftMotors.Pull();
	pros::delay(1000);
	RopoDevice::Chassis.MoveVelocity(0,0.6);
	pros::delay(600);
	RopoDevice::LiftMotors.Hold();
	RopoDevice::Chassis.AutoRotateAbs(0);
	while (!RopoDevice::Chassis.IfDegArrived()){
		pros::delay(20);
	}

	pros::delay(30);
	RopoDevice::Chassis.MoveVelocity(1.2,0);
	pros::delay(800);
	RopoDevice::Chassis.MoveVelocity(0,0);
	RopoDevice::Position_Motor::MyPosition.Set_XY(1.90,1.204);
	pros::delay(700);
	RopoDevice::Chassis.MoveVelocity(-0.5,0);
	pros::delay(800);

	// ------- Stage 6 - Touch the lift bar ------
	RopoDevice::LiftMotors.Hide();
	pros::delay(300);
	RopoDevice::Chassis.AutoMovePosAbs(1.08,0.564,-138);
	while (!RopoDevice::Chassis.IfArrived()){
		pros::delay(20);
	}
	pros::delay(300);
	RopoDevice::LiftMotors.Pull();

	// RopoDevice::LiftMotors.Hide();
	// pros::delay(700);
	// RopoDevice::Chassis.AutoMovePosAbs(0.63,0.841,-90);
	// while (!RopoDevice::Chassis.IfArrived()){
	// 	pros::delay(20);
	// }
	// RopoDevice::LiftMotors.Pull();
	// pros::delay(50);
	// RopoDevice::Chassis.MoveVelocity(0,-0.5);
	// pros::delay(600);
	
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

void autonomous_2() {
	// ------- Stage 1 - Catch the triball under the lift bar -------
	RopoDevice::LiftMotors.Hold();
	pros::delay(300);
	RopoDevice::Chassis.MoveVelocity(1.7,0);
	pros::delay(540);
	
	RopoDevice::Chassis.MoveVelocity(1.9,1.1);
	pros::delay(1100);
	RopoDevice::Chassis.MoveVelocity(0,0);
	RopoDevice::Position_Motor::MyPosition.Set_XY(1.83,0.822);
	pros::delay(700);
	RopoDevice::Chassis.MoveVelocity(-1,0);
	pros::delay(450);

	// ------- Stage 2 - Catch the union triball -------
	RopoDevice::LiftMotors.Hide();
	RopoDevice::Chassis.MoveVelocity(0,0);
	pros::delay(300);

	//RopoDevice::Chassis.AutoMovePosAbs(1.57,0.303,-50.0);	// 1.63
	RopoDevice::Chassis.AutoMovePosAbs(1.66,0.232,-46.0);	// 1.63
	while (!RopoDevice::Chassis.IfArrived()){
		pros::delay(20);
	}
	RopoDevice::Chassis.MoveVelocity(0.3,0);
	pros::delay(300);
	RopoDevice::Chassis.MoveVelocity(0,0);
	pros::delay(100);
	RopoDevice::LiftMotors.Hold();
	pros::delay(1200);
	RopoDevice::Chassis.MoveVelocity(-0.4,0);
	pros::delay(100);
	RopoDevice::LiftMotors.Pull();
	pros::delay(500);
	RopoDevice::LiftMotors.Hold();
	pros::delay(50);

	//
	RopoDevice::Chassis.AutoMovePosAbs(1.86,0.95,90.0);	// y 0.98
	while (!RopoDevice::Chassis.IfArrived()){
		pros::delay(20);
	}

	RopoDevice::Position_Motor::MyPosition.Set_XY(1.86, 1.002);

	RopoDevice::Chassis.MoveVelocity(-0.7,0);
	pros::delay(450);

	//
	RopoDevice::LiftMotors.Hide();
	pros::delay(500);
	RopoDevice::Chassis.AutoMovePosAbs(0.85,1.202,190.0);	// 1.63
	while (!RopoDevice::Chassis.IfArrived()){
		pros::delay(20);
	}
	RopoDevice::LiftMotors.Pull();
	pros::delay(500);
	RopoDevice::Chassis.MoveVelocity(-0.5,0);
	pros::delay(300);
	RopoDevice::LiftMotors.Hold();
	
	RopoDevice::Chassis.MoveVelocity(0,-5);
	pros::delay(100);
	RopoDevice::Chassis.MoveVelocity(0,-4);
	pros::delay(700);

	RopoDevice::LiftMotors.Hide();
	pros::delay(500);
	RopoDevice::Chassis.AutoMovePosAbs(0.77,1.480,120.0);	// 1.63
	while (!RopoDevice::Chassis.IfArrived()){
		pros::delay(20);
	}
	RopoDevice::LiftMotors.Pull();
	pros::delay(500);
	RopoDevice::Chassis.MoveVelocity(-0.5,-0.5);
	pros::delay(300);
	RopoDevice::LiftMotors.Hold();
	RopoDevice::Chassis.MoveVelocity(0,-5);
	pros::delay(100);
	RopoDevice::Chassis.MoveVelocity(0,-4);
	pros::delay(700);
	//

	pros::delay(300);
	RopoDevice::LiftMotors.Hide();
	pros::delay(500);
	// RopoDevice::Chassis.AutoMovePosAbs(1.40, 1.542, 0);
	// while (!RopoDevice::Chassis.IfArrived()){
	// 	pros::delay(20);
	// }
	// RopoDevice::Chassis.MoveVelocity(0.4,0);
	// pros::delay(500);
	// RopoDevice::Chassis.MoveVelocity(0,0);

	RopoDevice::Chassis.AutoMovePosAbs(0.68,0.540,-128.1);	// 1.63
	while (!RopoDevice::Chassis.IfArrived()){
		pros::delay(20);
	}
	RopoDevice::LiftMotors.Pull();
	pros::delay(500);

	RopoDevice::Chassis.MoveVelocity(0,0);
}

void skill() {
	// ------- Stage 1 - Catch the triball under the lift bar -------
	RopoDevice::LiftMotors.Hold();
	pros::delay(300);
	RopoDevice::Chassis.MoveVelocity(1.7,0);
	pros::delay(540);
	
	RopoDevice::Chassis.MoveVelocity(1.9,1.1);//
	pros::delay(1100);
	RopoDevice::Chassis.MoveVelocity(0,0);
	RopoDevice::Position_Motor::MyPosition.Set_XY(1.83,0.822);
	pros::delay(700);
	RopoDevice::Chassis.MoveVelocity(-1,0);
	pros::delay(450);

	// ------- Stage 2 - Catch the union triball -------
	RopoDevice::LiftMotors.Hide();
	RopoDevice::Chassis.MoveVelocity(0,0);
	pros::delay(300);

	//RopoDevice::Chassis.AutoMovePosAbs(1.57,0.303,-50.0);	// 1.63
	RopoDevice::Chassis.AutoMovePosAbs(1.66,0.232,-46.0);	// 1.63
	while (!RopoDevice::Chassis.IfArrived()){
		pros::delay(20);
	}
	RopoDevice::Chassis.MoveVelocity(0.3,0);
	pros::delay(300);
	RopoDevice::Chassis.MoveVelocity(0,0);
	pros::delay(100);
	RopoDevice::LiftMotors.Hold();
	pros::delay(1200);
	RopoDevice::Chassis.MoveVelocity(-0.4,0);
	pros::delay(100);
	RopoDevice::LiftMotors.Pull();
	pros::delay(500);
	RopoDevice::LiftMotors.Hold();
	pros::delay(50);

	//
	RopoDevice::Chassis.AutoMovePosAbs(1.86,0.98,90.0);	// 1.63
	while (!RopoDevice::Chassis.IfArrived()){
		pros::delay(20);
	}

	RopoDevice::Position_Motor::MyPosition.Set_XY(1.86, 1.002);

	RopoDevice::Chassis.MoveVelocity(-0.7,0);
	pros::delay(450);

	//
	RopoDevice::LiftMotors.Hide();
	pros::delay(500);
	RopoDevice::Chassis.AutoMovePosAbs(0.85,1.202,190.0);	//
	while (!RopoDevice::Chassis.IfArrived()){
		pros::delay(20);
	}
	RopoDevice::LiftMotors.Pull();
	pros::delay(500);
	RopoDevice::Chassis.MoveVelocity(-0.5,0);
	pros::delay(300);
	RopoDevice::LiftMotors.Hold();
	
	RopoDevice::Chassis.MoveVelocity(0,-5);
	pros::delay(100);
	RopoDevice::Chassis.MoveVelocity(0,-4);
	pros::delay(700);

	RopoDevice::LiftMotors.Hide();
	pros::delay(500);
	RopoDevice::Chassis.AutoMovePosAbs(0.77,1.480,120.0);	// 1.63
	while (!RopoDevice::Chassis.IfArrived()){
		pros::delay(20);
	}
	RopoDevice::LiftMotors.Pull();
	pros::delay(500);
	RopoDevice::Chassis.MoveVelocity(-0.5,-0.5);
	pros::delay(300);
	RopoDevice::LiftMotors.Hold();
	RopoDevice::ThreeWire::ExternPneumatic.set_value(true);
	pros::delay(500);
	RopoDevice::Chassis.MoveVelocity(0,-5);
	pros::delay(100);
	RopoDevice::Chassis.MoveVelocity(0,-4);
	pros::delay(15000);
	//

	pros::delay(300);
	RopoDevice::LiftMotors.Hide();
	pros::delay(500);
	
	// RopoDevice::Chassis.MoveVelocity(0,0);

	// RopoDevice::Chassis.AutoMovePosAbs(0.68,0.540,-128.1);	// 1.63
	// while (!RopoDevice::Chassis.IfArrived()){
	// 	pros::delay(20);
	// }
	// RopoDevice::LiftMotors.Pull();
	// pros::delay(500);

	RopoDevice::Chassis.MoveVelocity(0,0);
}

void autonomous(){
	autonomous_2();
}

void opcontrol() {
	pros::Task *RumbleTask = new pros::Task(ControllerModule::RumbleMe);
	pros::Task *PrintTask = new pros::Task(ControllerModule::ControllerPrint);
	pros::Controller MasterController(pros::E_CONTROLLER_MASTER);
	RopoController::ButtonTaskLine ButtonDetectLine(MasterController);
	FloatType VelocityMax = 2.3;
	FloatType RopoWcLimit = 3.5;
	bool ChassisMove = false;
	
	RopoController::AxisValueCast XVelocityInput(MasterController,pros::E_CONTROLLER_ANALOG_LEFT_Y,RopoController::Linear);
	RopoController::AxisValueCast WVelocityInput(MasterController,pros::E_CONTROLLER_ANALOG_RIGHT_X,RopoController::Linear);
	Vector Velocity(RopoMath::ColumnVector,2),ResVelocity;
	MasterController.clear();
	//ButtonDetectLine.AddButtonDetect(pros::E_CONTROLLER_DIGITAL_Y  , RopoController::Rising,  autonomous);
	ButtonDetectLine.AddButtonDetect(pros::E_CONTROLLER_DIGITAL_L1, RopoController::Rising, ControllerModule::ChangeLift);
	ButtonDetectLine.AddButtonDetect(pros::E_CONTROLLER_DIGITAL_L2, RopoController::Rising, ControllerModule::Lift);
	ButtonDetectLine.AddButtonDetect(pros::E_CONTROLLER_DIGITAL_R1,RopoController::Rising,ControllerModule::ChangeCatch);
	// ButtonDetectLine.AddButtonDetect(pros::E_CONTROLLER_DIGITAL_B, RopoController::Rising, ControllerModule::Push);
	// ButtonDetectLine.AddButtonDetect(pros::E_CONTROLLER_DIGITAL_B, RopoController::Falling, ControllerModule::Pull);
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

		pros::lcd::print(1,"Ready!!! V:%.1f %.1f",XInput,WInput);
		pros::delay(4);
	}
}
