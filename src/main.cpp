#include "main.h"
#include "RopoChassis.hpp"
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

	bool externFlag = false;
	void Switch(){
		externFlag ^= 1; 
		RopoDevice::ThreeWire::ExternPneumatic.set_value(externFlag);
	}

	bool locktag = false;
	bool timelock = true;
	void ChangeCatch(){
		if (!timelock)
			locktag ^= 1;
		RopoDevice::ThreeWire::CatchPneumatic.set_value(locktag);
	}

	void TimeLock(){
		FloatType aa = pros::millis();
		while(pros::millis()-aa<45000){
			pros::delay(100);
		}
		timelock = false;
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

	void Pull(){
		RopoDevice::LiftMotors.Pull();
		catch_1 = 2;
	}

	void Hide(){
		RopoDevice::LiftMotors.Hide();
		catch_1 = 0;
	}

	void Wait(){
		RopoDevice::LiftMotors.Wait();
		catch_1 = 3;
	}


	void ChangeLift(){
		if (catch_1 == 1) {
			Wait();
			Wait();
		} else {
			Hold();
		}
	}

	bool SlowModeFlag = false;
	void SlowModeSwitch(){
		SlowModeFlag ^= 1;
	}

	void ControllerPrint(){
		while(true) {
			pros::Controller MasterController(pros::E_CONTROLLER_MASTER);
			MasterController.print(0,1,"degree: %.1lf",-RopoDevice::Sensors::Inertial.get_yaw());
			pros::delay(10); 
			MasterController.print(1,1,"X: %.2lf Y:%.2lf",RopoDevice::Position_Motor::MyPosition.Get_X(),RopoDevice::Position_Motor::MyPosition.Get_Y());
			pros::delay(10);
			if(SlowModeFlag == true) 
				MasterController.print(2,1,"SlowModeOn!"),
				pros::delay(10);
			//MasterController.print(3,1,"%.2lf  %d",RopoDevice::LiftMotors.GetLifterPosition(), RopoDevice::LiftMotors.GetLifterStatus());
		}
	}
}

void initialize() {
	pros::lcd::initialize();
	pros::delay(10);
	//RopoDevice::MotorsInit();
	//RopoDevice::MotorsInit();
	RopoDevice::Position_Motor::MyPosition.initial();
	RopoDevice::DeviceInit();
}

void disabled() {}

void competition_initialize() {}

void autonomous_1() {
	// ------- Stage 1 - Catch the triball under the lift bar -------
	// RopoDevice::LiftMotors.Hold();
	// pros::delay(300);
	// RopoDevice::Chassis.MoveVelocity(1.5,0);
	// pros::delay(700);
		// ---
		//向前推球进网
		RopoDevice::LiftMotors.Hold();

		pros::delay(300);
		RopoDevice::Chassis.MoveVelocity(1.2,0.3);
		pros::delay(1200);

		RopoDevice::Chassis.AutoMovePosAbs(1.71,0.471,72.3);


		// RopoDevice::Chassis.AutoMovePosAbs(1.92,0.941,72.3);
		RopoDevice::Chassis.MoveVelocity(0.8,0);
		pros::delay(800);
		RopoDevice::Chassis.MoveVelocity(0,0);
		RopoDevice::Position_Motor::MyPosition.Set_XY(1.92,0.941);
		// RopoDevice::Chassis.AutoMovePosAbs(1.71,0.471,-45);
		// ---
		pros::delay(800);
		RopoDevice::Chassis.MoveVelocity(-0.4,-0.2);
		pros::delay(1200);

	// RopoDevice::Chassis.MoveVelocity(1.9,1.5);
	// pros::delay(1100);
	// RopoDevice::Chassis.MoveVelocity(0,0);
	// RopoDevice::Position_Motor::MyPosition.Set_XY(1.83,0.822);
	// pros::delay(700);
	// RopoDevice::Chassis.MoveVelocity(-1,0);
	// pros::delay(450);

	// ------- Stage 2 - Catch the union triball -------
	
	RopoDevice::LiftMotors.Hide();
	RopoDevice::Chassis.MoveVelocity(0,0);
	pros::delay(500);
	// ------- Stage 2 - Catch the union triball -------
	
	RopoDevice::LiftMotors.Hide();
	RopoDevice::Chassis.MoveVelocity(0,0);
	pros::delay(500);

// 	//RopoDevice::Chassis.AutoMovePosAbs(1.57,0.303,-50.0);	// 1.63
	RopoDevice::Chassis.AutoMovePosAbs(1.69,0.373,-53.0);	// 1.63
// 	while (!RopoDevice::Chassis.IfArrived()){
// 		pros::delay(20);
// 	}
	RopoDevice::Chassis.MoveVelocity(0.3,0);
	pros::delay(300);
	RopoDevice::Chassis.MoveVelocity(0,0);
	pros::delay(100);
	RopoDevice::LiftMotors.Hold();
	pros::delay(1200);
	RopoDevice::Chassis.MoveVelocity(-0.4,0);
	pros::delay(100);
	RopoDevice::LiftMotors.Pull();
	pros::delay(275);
	RopoDevice::LiftMotors.Hold();
	pros::delay(650);

	RopoDevice::Chassis.AutoMovePosAbs(1.62,0.570,48.4);
	RopoDevice::Chassis.MoveVelocity(0.65,0.5);
	pros::delay(1000);
	RopoDevice::Position_Motor::MyPosition.Set_XY(1.88,1.03);


	RopoDevice::Chassis.MoveVelocity(-0.5,-0.5);
	pros::delay(1000);
	RopoDevice::LiftMotors.Hide();
	RopoDevice::Chassis.MoveVelocity(0,0);
	pros::delay(500);



	// ------- Stage 3 - Catch the triball - 1 -------
	RopoDevice::Chassis.MoveVelocity(-0.8,0);
	pros::delay(600);
	RopoDevice::Chassis.AutoRotateAbs(120);
	RopoDevice::Chassis.MoveVelocity(1,0.5);
	pros::delay(750);
	RopoDevice::Chassis.AutoMovePosAbs(0.92,1.10,178.33);
	RopoDevice::LiftMotors.Pull();
	pros::delay(1000);
	RopoDevice::Chassis.MoveVelocity(-0.5,0);
	pros::delay(500);
	RopoDevice::Chassis.AutoRotateAbs(25);
	// RopoDevice::Chassis.MoveVelocity(-0.3,0);
	// pros::delay(300);
	RopoDevice::LiftMotors.Hide();
	pros::delay(500);
	RopoDevice::Chassis.MoveVelocity(0.8,0);
	pros::delay(800);
	RopoDevice::Position_Motor::MyPosition.Set_XY(1.62,1.241);

	// ------- Stage 4 - Push -------
	RopoDevice::Chassis.MoveVelocity(-1,0.5);
	pros::delay(800);
	RopoDevice::Chassis.MoveVelocity(0,0);
	RopoDevice::ThreeWire::ExternPneumatic.set_value(true);
	pros::delay(200);
	RopoDevice::Chassis.MoveVelocity(1,-0.6);
	pros::delay(800);
	RopoDevice::Position_Motor::MyPosition.Set_XY(1.64,1.291);


	// ------- Stage 5 - Touch -------
	RopoDevice::ThreeWire::ExternPneumatic.set_value(false);
	pros::delay(200);
	RopoDevice::Chassis.MoveVelocity(-0.8,0.5);
	pros::delay(500);
	RopoDevice::Chassis.BreakableAutoMovePosAbs(0.84,0.591);
	pros::delay(3000);
	RopoDevice::Chassis.AutoRotateAbs(-135);
	RopoDevice::Chassis.MoveVelocity(0.8,0);
	pros::delay(800);
	RopoDevice::LiftMotors.Pull();
	pros::delay(500);


	// ------- End -------
	RopoDevice::Chassis.MoveVelocity(0,0);
}

// void autonomous_2() {
// 	// ------- Stage 1 - Catch the triball under the lift bar -------
// 	RopoDevice::LiftMotors.Hold();
// 	pros::delay(300);
// 	RopoDevice::Chassis.MoveVelocity(1.7,0);
// 	pros::delay(540);
	
// 	RopoDevice::Chassis.MoveVelocity(1.9,1.1);
// 	pros::delay(1100);
// 	RopoDevice::Chassis.MoveVelocity(0,0);
// 	RopoDevice::Position_Motor::MyPosition.Set_XY(1.83,0.822);
// 	pros::delay(700);
// 	RopoDevice::Chassis.MoveVelocity(-1,0);
// 	pros::delay(450);

// 	// ------- Stage 2 - Catch the union triball -------
// 	RopoDevice::LiftMotors.Hide();
// 	RopoDevice::Chassis.MoveVelocity(0,0);
// 	pros::delay(300);

// 	//RopoDevice::Chassis.AutoMovePosAbs(1.57,0.303,-50.0);	// 1.63
// 	RopoDevice::Chassis.AutoMovePosAbs(1.66,0.232,-46.0);	// 1.63
// 	while (!RopoDevice::Chassis.IfArrived()){
// 		pros::delay(20);
// 	}
// 	RopoDevice::Chassis.MoveVelocity(0.3,0);
// 	pros::delay(300);
// 	RopoDevice::Chassis.MoveVelocity(0,0);
// 	pros::delay(100);
// 	RopoDevice::LiftMotors.Hold();
// 	pros::delay(1200);
// 	RopoDevice::Chassis.MoveVelocity(-0.4,0);
// 	pros::delay(100);
// 	RopoDevice::LiftMotors.Pull();
// 	pros::delay(500);
// 	RopoDevice::LiftMotors.Hold();
// 	pros::delay(50);

// 	//
// 	RopoDevice::Chassis.AutoMovePosAbs(1.86,0.95,90.0);	// y 0.98
// 	while (!RopoDevice::Chassis.IfArrived()){
// 		pros::delay(20);
// 	}

// 	RopoDevice::Position_Motor::MyPosition.Set_XY(1.86, 1.002);

// 	RopoDevice::Chassis.MoveVelocity(-0.7,0);
// 	pros::delay(450);

// 	//
// 	RopoDevice::LiftMotors.Hide();
// 	pros::delay(500);
// 	RopoDevice::Chassis.AutoMovePosAbs(0.85,1.202,190.0);	// 1.63
// 	while (!RopoDevice::Chassis.IfArrived()){
// 		pros::delay(20);
// 	}
// 	RopoDevice::LiftMotors.Pull();
// 	pros::delay(500);
// 	RopoDevice::Chassis.MoveVelocity(-0.5,0);
// 	pros::delay(300);
// 	RopoDevice::LiftMotors.Hold();
	
// 	RopoDevice::Chassis.MoveVelocity(0,-5);
// 	pros::delay(100);
// 	RopoDevice::Chassis.MoveVelocity(0,-4);
// 	pros::delay(700);

// 	RopoDevice::LiftMotors.Hide();
// 	pros::delay(500);
// 	RopoDevice::Chassis.AutoMovePosAbs(0.77,1.480,120.0);	// 1.63
// 	while (!RopoDevice::Chassis.IfArrived()){
// 		pros::delay(20);
// 	}
// 	RopoDevice::LiftMotors.Pull();
// 	pros::delay(500);
// 	RopoDevice::Chassis.MoveVelocity(-0.5,-0.5);
// 	pros::delay(300);
// 	RopoDevice::LiftMotors.Hold();
// 	RopoDevice::Chassis.MoveVelocity(0,-5);
// 	pros::delay(100);
// 	RopoDevice::Chassis.MoveVelocity(0,-4);
// 	pros::delay(700);
// 	//

// 	pros::delay(300);
// 	RopoDevice::LiftMotors.Hide();
// 	pros::delay(500);
// 	// RopoDevice::Chassis.AutoMovePosAbs(1.40, 1.542, 0);
// 	// while (!RopoDevice::Chassis.IfArrived()){
// 	// 	pros::delay(20);
// 	// }
// 	// RopoDevice::Chassis.MoveVelocity(0.4,0);
// 	// pros::delay(500);
// 	// RopoDevice::Chassis.MoveVelocity(0,0);

// 	RopoDevice::Chassis.AutoMovePosAbs(0.68,0.540,-128.1);	// 1.63
// 	while (!RopoDevice::Chassis.IfArrived()){
// 		pros::delay(20);
// 	}
// 	RopoDevice::LiftMotors.Pull();
// 	pros::delay(500);

// 	RopoDevice::Chassis.MoveVelocity(0,0);
// }

// void skill() {
// 	// ------- Stage 1 - Catch the triball under the lift bar -------
// 	RopoDevice::LiftMotors.Hold();
// 	pros::delay(300);
// 	RopoDevice::Chassis.MoveVelocity(1.7,0);
// 	pros::delay(540);
	
// 	RopoDevice::Chassis.MoveVelocity(1.9,1.1);//
// 	pros::delay(1100);
// 	RopoDevice::Chassis.MoveVelocity(0,0);
// 	RopoDevice::Position_Motor::MyPosition.Set_XY(1.83,0.822);
// 	pros::delay(700);
// 	RopoDevice::Chassis.MoveVelocity(-1,0);
// 	pros::delay(450);

// 	// ------- Stage 2 - Catch the union triball -------
// 	RopoDevice::LiftMotors.Hide();
// 	RopoDevice::Chassis.MoveVelocity(0,0);
// 	pros::delay(300);

// 	//RopoDevice::Chassis.AutoMovePosAbs(1.57,0.303,-50.0);	// 1.63
// 	RopoDevice::Chassis.AutoMovePosAbs(1.66,0.232,-46.0);	// 1.63
// 	while (!RopoDevice::Chassis.IfArrived()){
// 		pros::delay(20);
// 	}
// 	RopoDevice::Chassis.MoveVelocity(0.3,0);
// 	pros::delay(300);
// 	RopoDevice::Chassis.MoveVelocity(0,0);
// 	pros::delay(100);
// 	RopoDevice::LiftMotors.Hold();
// 	pros::delay(1200);
// 	RopoDevice::Chassis.MoveVelocity(-0.4,0);
// 	pros::delay(100);
// 	RopoDevice::LiftMotors.Pull();
// 	pros::delay(500);
// 	RopoDevice::LiftMotors.Hold();
// 	pros::delay(50);

// 	//
// 	RopoDevice::Chassis.AutoMovePosAbs(1.86,0.98,90.0);	// 1.63
// 	while (!RopoDevice::Chassis.IfArrived()){
// 		pros::delay(20);
// 	}

// 	RopoDevice::Position_Motor::MyPosition.Set_XY(1.86, 1.002);

// 	RopoDevice::Chassis.MoveVelocity(-0.7,0);
// 	pros::delay(450);

// 	//
// 	RopoDevice::LiftMotors.Hide();
// 	pros::delay(500);
// 	RopoDevice::Chassis.AutoMovePosAbs(0.85,1.202,190.0);	//
// 	while (!RopoDevice::Chassis.IfArrived()){
// 		pros::delay(20);
// 	}
// 	RopoDevice::LiftMotors.Pull();
// 	pros::delay(500);
// 	RopoDevice::Chassis.MoveVelocity(-0.5,0);
// 	pros::delay(300);
// 	RopoDevice::LiftMotors.Hold();
	
// 	RopoDevice::Chassis.MoveVelocity(0,-5);
// 	pros::delay(100);
// 	RopoDevice::Chassis.MoveVelocity(0,-4);
// 	pros::delay(700);

// 	RopoDevice::LiftMotors.Hide();
// 	pros::delay(500);
// 	RopoDevice::Chassis.AutoMovePosAbs(0.77,1.480,120.0);	// 1.63
// 	while (!RopoDevice::Chassis.IfArrived()){
// 		pros::delay(20);
// 	}
// 	RopoDevice::LiftMotors.Pull();
// 	pros::delay(500);
// 	RopoDevice::Chassis.MoveVelocity(-0.5,-0.5);
// 	pros::delay(300);
// 	RopoDevice::LiftMotors.Hold();
// 	RopoDevice::ThreeWire::ExternPneumatic.set_value(true);
// 	pros::delay(500);
// 	RopoDevice::Chassis.MoveVelocity(0,-5);
// 	pros::delay(100);
// 	RopoDevice::Chassis.MoveVelocity(0,-4);
// 	pros::delay(15000);
// 	//

// 	pros::delay(300);
// 	RopoDevice::LiftMotors.Hide();
// 	pros::delay(500);
	
// 	// RopoDevice::Chassis.MoveVelocity(0,0);

// 	// RopoDevice::Chassis.AutoMovePosAbs(0.68,0.540,-128.1);	// 1.63
// 	// while (!RopoDevice::Chassis.IfArrived()){
// 	// 	pros::delay(20);
// 	// }
// 	// RopoDevice::LiftMotors.Pull();
// 	// pros::delay(500);

// 	RopoDevice::Chassis.MoveVelocity(0,0);
// }

void autonomous(){
	autonomous_1();
}

void opcontrol() {
	pros::Task *RumbleTask = new pros::Task(ControllerModule::RumbleMe);
	pros::Task *TimeLockTask = new pros::Task(ControllerModule::TimeLock);
	pros::Task *PrintTask = new pros::Task(ControllerModule::ControllerPrint);
	pros::Controller MasterController(pros::E_CONTROLLER_MASTER);
	RopoController::ButtonTaskLine ButtonDetectLine(MasterController);
	FloatType VelocityMax = 3;
	FloatType RopoWcLimit = 4.5;
	bool ChassisMove = false;
	
	RopoController::AxisValueCast XVelocityInput(MasterController,pros::E_CONTROLLER_ANALOG_LEFT_Y,RopoController::Linear);
	RopoController::AxisValueCast WVelocityInput(MasterController,pros::E_CONTROLLER_ANALOG_RIGHT_X,RopoController::Linear);
	Vector Velocity(RopoMath::ColumnVector,2),ResVelocity;
	MasterController.clear();
	// ButtonDetectLine.AddButtonDetect(pros::E_CONTROLLER_DIGITAL_X  , RopoController::Rising,  autonomous);
	ButtonDetectLine.AddButtonDetect(pros::E_CONTROLLER_DIGITAL_X  , RopoController::Rising,  ControllerModule::SlowModeSwitch);
	ButtonDetectLine.AddButtonDetect(pros::E_CONTROLLER_DIGITAL_L2, RopoController::Rising, ControllerModule::ChangeLift);
	ButtonDetectLine.AddButtonDetect(pros::E_CONTROLLER_DIGITAL_L1, RopoController::Rising, ControllerModule::Hide);
	ButtonDetectLine.AddButtonDetect(pros::E_CONTROLLER_DIGITAL_Y, RopoController::Rising, ControllerModule::Pull);
	ButtonDetectLine.AddButtonDetect(pros::E_CONTROLLER_DIGITAL_R1,RopoController::Rising,ControllerModule::ChangeCatch);
	// ButtonDetectLine.AddButtonDetect(pros::E_CONTROLLER_DIGITAL_B, RopoController::Rising, ControllerModule::Push);
	// ButtonDetectLine.AddButtonDetect(pros::E_CONTROLLER_DIGITAL_B, RopoController::Falling, ControllerModule::Pull);
	ButtonDetectLine.AddButtonDetect(pros::E_CONTROLLER_DIGITAL_R2, RopoController::Rising, ControllerModule::Switch);
	ButtonDetectLine.Enable();

	while (true) {
		if(ControllerModule::SlowModeFlag == true) 
			RopoController::AxisValueCast XVelocityInput(MasterController,pros::E_CONTROLLER_ANALOG_LEFT_Y,RopoController::Ln);
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

		// Debug

		pros::delay(5);
	}
}
