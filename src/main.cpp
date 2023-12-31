#include "main.h"
#include "RopoController.hpp"
#include "RopoDevice.hpp"
#include "RopoPosition.hpp"
#include "pros/llemu.hpp"
#include "pros/misc.h"
#include "pros/misc.hpp"
#include "pros/rtos.hpp"
namespace ControllerModule{
	bool autoend = false;
	bool ifHide = false;
	void PositionAdjustLeft(){
        
        RopoDevice::Chassis.MoveVelocity(0,0);
        RopoDevice::Chassis.MoveVelocity(0.6,1.2);
        pros::delay(500);
        RopoDevice::Chassis.MoveVelocity(0,0);
		pros::delay(100);
        RopoDevice::Chassis.MoveVelocity(0.6,-1.2);
        pros::delay(400);
		RopoDevice::Chassis.MoveVelocity(0,0);
		pros::delay(100);
        RopoDevice::Chassis.MoveVelocity(-1.3,0);//后退
        pros::delay(320);
		RopoDevice::Chassis.MoveVelocity(-0.8,0);
		pros::delay(50);
		RopoDevice::Chassis.MoveVelocity(0,0);
		pros::delay(20);
    }

	void PositionAdjustRight(){
        
        RopoDevice::Chassis.MoveVelocity(0,0);
        RopoDevice::Chassis.MoveVelocity(0.6,-1.2);
        pros::delay(500);
        RopoDevice::Chassis.MoveVelocity(0,0);
		pros::delay(100);
        RopoDevice::Chassis.MoveVelocity(0.6,1.2);
        pros::delay(450);
		RopoDevice::Chassis.MoveVelocity(0,0);
		pros::delay(100);
        RopoDevice::Chassis.MoveVelocity(-1.3,0);//后退
        pros::delay(360);
		RopoDevice::Chassis.MoveVelocity(-0.8,0);
		pros::delay(50);
		RopoDevice::Chassis.MoveVelocity(0,0);
		pros::delay(20);
    }

	void Throw(void){
		RopoDevice::Thrower_Motor::MyThrower.Throw();
		RopoDevice::Intaker_Motor::MyIntaker.SetStayHidingMode(true);
		ifHide = false;
	}
	void Hide(void){

		RopoDevice::Thrower_Motor::MyThrower.Wait();
		pros::delay(200);
		while(!RopoDevice::Thrower_Motor::MyThrower.IfReady()){
			pros::delay(20);
		}
		RopoDevice::Thrower_Motor::MyThrower.Hide();
		RopoDevice::Intaker_Motor::MyIntaker.SetStayHidingMode(false);
		ifHide = true;

	}
	void Wait(void){
			RopoDevice::Thrower_Motor::MyThrower.Wait();
			RopoDevice::Intaker_Motor::MyIntaker.SetStayHidingMode(true);
			ifHide = false;
	}

	void ChangeHideAndWait(void){
		if (ifHide == true) {
			Wait();
		}
		else {
			Hide();
		}
	}
	
	static bool L1Pressing = false;
	pros::Task* L1PressingTask;
	static void L1RisingFunction2(void){
		while (L1Pressing == true) {
			if (L1Pressing == true) {
				RopoDevice::Chassis.MoveVelocity(-0.3,0);
				ControllerModule::Wait();
				pros::delay(200);
				while(RopoDevice::Thrower_Motor::MyThrower.IfReady() == false){
					pros::delay(20);
				}
				pros::delay(200);
				ControllerModule::Throw();
				pros::delay(500);
				ControllerModule::Wait();
				ifHide = false;
			}
			pros::delay(50);
		}

	}


	void L1BoolSwitch(void *param){
		bool *p = static_cast<bool *>(param);
		(*p) ^= 1;
		delete L1PressingTask;
		L1PressingTask = new pros::Task(L1RisingFunction2);
	}


	void BoolSwitch(void * Parameter){
		bool *p = static_cast<bool *>(Parameter);
		(*p) ^= 1;
	}

	void ChangeIntakerHideMode(void){
		RopoDevice::Intaker_Motor::MyIntaker.ChangeIntakeStatus();
	}
	void IntakerRest(void){
		if(RopoDevice::Intaker_Motor::MyIntaker.GetIntakeMode() == true && RopoDevice::Intaker_Motor::MyIntaker.GetRestingMode() == false){
			RopoDevice::Intaker_Motor::MyIntaker.ChangeRestMode();
		}
	}
	void Intake(void){
		RopoDevice::Intaker_Motor::MyIntaker.SetRestMode(false);
		RopoDevice::Intaker_Motor::MyIntaker.SetIntakeStatus(true);
	}

	void RumbleMe(){
		pros::Controller MasterController1(pros::E_CONTROLLER_MASTER);
		MasterController1.rumble("-.-.-");
		RopoApi::FloatType aa = pros::millis();
		while(pros::millis()-aa<55000){
			pros::delay(100);
		}
		MasterController1.rumble("-.-.-");
		while(pros::millis()-aa<65000){
			pros::delay(100);
		}
		MasterController1.rumble("-.-.-");
	}
	void Push(){
		RopoDevice::ThreeWire::PusherPneumatic.set_value(true);
	}
	void Pull(){
		RopoDevice::ThreeWire::PusherPneumatic.set_value(false);
	}

	void Display(){
		pros::Controller MasterController2(pros::E_CONTROLLER_MASTER);
		while(true){
			MasterController2.print(0,1,"degree: %.1lf",-RopoDevice::Sensors::Inertial.get_yaw());
			pros::delay(10); 
			MasterController2.print(1,1,"X: %.2lf Y:%.2lf",RopoDevice::Position_Motor::MyPosition.Get_X(),RopoDevice::Position_Motor::MyPosition.Get_Y());
			pros::delay(10); 
			// // MasterController2.print(2,1,"X: %.3lf Y:%.2lf F:%1d",RopoDevice::Sensors::myOpenMv.GetDistance(),RopoDevice::Sensors::myOpenMv.GetTheta(),RopoDevice::Sensors::myOpenMv.GetExists());
			// // pros::delay(10); 
		}
	}
}
void initialize() {
	pros::lcd::initialize();
	pros::delay(10);
	RopoDevice::DeviceInit();
}

void disabled() {}

void competition_initialize() {}

void autonomous_2() {
	//------吃球
	RopoDevice::DeviceInitOp2();
	ControllerModule::Wait();
	pros::delay(200);
	while(RopoDevice::Thrower_Motor::MyThrower.IfReady() == false){
		pros::delay(20);
	}
	pros::delay(400);
	RopoDevice::Thrower_Motor::MyThrower.set_is_disable(true);
	pros::delay(100);
	RopoDevice::Motors::ThrowerMotor.move_voltage(-8000);
	pros::delay(280);
	RopoDevice::Motors::ThrowerMotor.move_voltage(-2400);

	//----------塞球 
	RopoDevice::Chassis.MoveVelocity(0.7,0);
	pros::delay(400);
	RopoDevice::Chassis.MoveVelocity(0.4,0);
	pros::delay(100);
	RopoDevice::Chassis.MoveVelocity(0,0);
	pros::delay(30);
	RopoDevice::Chassis.MoveVelocity(0.5,-0.5);
	pros::delay(350);
	RopoDevice::Chassis.MoveVelocity(0.9,0);
	pros::delay(450);
	RopoDevice::Chassis.MoveVelocity(0.4,0);
	pros::delay(100);
	RopoDevice::Chassis.MoveVelocity(0,0);
	pros::delay(150);
	RopoDevice::Chassis.MoveVelocity(0,-3);
	pros::delay(500);
	RopoDevice::Chassis.AutoRotateAbs(-123);
	while (!RopoDevice::Chassis.IfDegArrived()){
	 	pros::delay(20);
	}
	pros::delay(200);
	RopoDevice::Chassis.MoveVelocity(0,0);
	pros::delay(100);
	RopoDevice::Chassis.MoveVelocity(-1.0,0);
	pros::delay(280);//300
	RopoDevice::Chassis.MoveVelocity(-0.6,0);
	pros::delay(140);
	
	RopoDevice::Chassis.MoveVelocity(-0.45,1.35);
	pros::delay(100);
	RopoDevice::Motors::ThrowerMotor.move_voltage(1000);
	pros::delay(670);

	RopoDevice::Chassis.MoveVelocity(0,0);
	ControllerModule::Wait();
	RopoDevice::Chassis.AutoRotateAbs(-45);
	pros::delay(250);
	RopoDevice::Motors::ThrowerMotor.move_voltage(0);
	RopoDevice::Chassis.MoveVelocity(-1.1,0);
	pros::delay(140);
	RopoDevice::Chassis.MoveVelocity(-0.6,0);
	pros::delay(90);//
	
	
	//--------推球

	RopoDevice::Chassis.MoveVelocity(0,0);
	pros::delay(50);
	RopoDevice::Chassis.AutoRotateAbs(-49);
	pros::delay(1200);
	RopoDevice::Chassis.MoveVelocity(-1.2,0);
	pros::delay(250);
	RopoDevice::Chassis.MoveVelocity(0,0);
	pros::delay(50);
	RopoDevice::Chassis.MoveVelocity(1.5,0);
	pros::delay(400);
	ControllerModule::ChangeIntakerHideMode();
	pros::delay(250);
	ControllerModule::Push();
	pros::delay(150);
	RopoDevice::Chassis.MoveVelocity(0.8,0);
	pros::delay(150);
	RopoDevice::Chassis.MoveVelocity(0.4,0);
	pros::delay(400);
	ControllerModule::Hide();
	ControllerModule::Pull();
	RopoDevice::Chassis.MoveVelocity(-0.5,0);
	pros::delay(300);
	
	pros::delay(100);
	RopoDevice::Chassis.MoveVelocity(0.30,0);
	ControllerModule::Push();
	pros::delay(400);
	
	pros::delay(1200);
	ControllerModule::Pull();
	RopoDevice::Chassis.MoveVelocity(0,0);
	pros::delay(100);
	RopoDevice::Position_Motor::MyPosition.SetXAndY(1.67, 0.01);
	
	RopoDevice::Chassis.MoveVelocity(-0.6,1.1);
	pros::delay(600);

	RopoDevice::Chassis.MoveVelocity(-0.6,0);
	pros::delay(600);

	

	// RopoDevice::Chassis.MoveVelocity(-0.3,-0.1);
	// pros::delay(400);
	// RopoDevice::Chassis.MoveVelocity(-0.3,0);
	// pros::delay(200);
	RopoDevice::Chassis.MoveVelocity(0,0);
	
	//------回去抛球
	RopoDevice::Chassis.AutoMovePosAbsBack(0.3,0.01,0);
	while (!RopoDevice::Chassis.IfArrived()){
	 	pros::delay(20);
	}
	RopoDevice::Chassis.MoveVelocity(-0.8,0);
	pros::delay(340);
	RopoDevice::Chassis.MoveVelocity(-0.25,0);
	pros::delay(400);
	
	for(int i = 0; i < 11; i++){
		
		RopoDevice::Chassis.MoveVelocity(-0.3,0);
		ControllerModule::Wait();
		pros::delay(200);
		while(RopoDevice::Thrower_Motor::MyThrower.IfReady() == false){
			pros::delay(20);
		}
		pros::delay(200);
		ControllerModule::Throw();
		pros::delay(500);
		
	}
	RopoDevice::Thrower_Motor::MyThrower.set_is_disable(true);
	pros::delay(100);
	RopoDevice::Motors::ThrowerMotor.move_voltage(-2400);
	pros::delay(200);
	RopoDevice::Position_Motor::MyPosition.SetXAndY(0, 0.02);
	ControllerModule::Intake();
	pros::delay(100);
	ControllerModule::Intake();
	//----碰杆
	RopoDevice::Chassis.MoveVelocity(0.1,0);
	pros::delay(100);
	RopoDevice::Chassis.MoveVelocity(0.6,-1.5);
	pros::delay(350);
	ControllerModule::Wait();
	pros::delay(400);
	ControllerModule::IntakerRest();
	RopoDevice::Chassis.MoveVelocity(0.5,0.0);
	pros::delay(200);
	RopoDevice::Chassis.MoveVelocity(0.5,0.45);
	pros::delay(100);
	RopoDevice::Chassis.MoveVelocity(0.8,1.0);
	pros::delay(520);
	RopoDevice::Chassis.MoveVelocity(1.8,0);
	pros::delay(350);
	RopoDevice::Chassis.MoveVelocity(0.4,0);
	pros::delay(40);
	
	RopoDevice::Chassis.MoveVelocity(0,0);
	//-------end-----------
	ControllerModule::autoend = true;
	pros::delay(100);
	RopoDevice::Chassis.MoveVelocity(0,0);
	pros::delay(30);
}

void test(){
	RopoDevice::DeviceInitOp2();
	ControllerModule::Wait();
	pros::delay(200);
	while(RopoDevice::Thrower_Motor::MyThrower.IfReady() == false){
		pros::delay(20);
	}
	pros::delay(400);
	RopoDevice::Thrower_Motor::MyThrower.set_is_disable(true);
	pros::delay(100);
	RopoDevice::Motors::ThrowerMotor.move_voltage(-8000);
	pros::delay(280);
	RopoDevice::Motors::ThrowerMotor.move_voltage(-2400);
	pros::delay(1000);
	RopoDevice::Motors::ThrowerMotor.move_voltage(1000);
	pros::delay(550);
	ControllerModule::Wait();
	pros::delay(200);
	while(RopoDevice::Thrower_Motor::MyThrower.IfReady() == false){
		pros::delay(20);
	}
	RopoDevice::Motors::ThrowerMotor.move_voltage(0);

}

void autonomous(){
	autonomous_2();
	//test();
}

void opcontrol() {
	RopoDevice::DeviceInitOp2();
	if(pros::competition::is_connected() == true){
		ControllerModule::Hide();
	}
	pros::Task *RumbleTask = new pros::Task(ControllerModule::RumbleMe);
	pros::Task *DisplayTask = new pros::Task(ControllerModule::Display);
	pros::Controller MasterController(pros::E_CONTROLLER_MASTER);
	RopoController::ButtonTaskLine ButtonDetectLine(MasterController);
	RopoApi::FloatType VelocityMax = 1.8;
	RopoApi::FloatType RopoWcLimit = 3.5;
	bool ChassisMove = false;
	RopoDevice::Chassis.MoveVelocity(0,0);
	RopoController::AxisValueCast XVelocityInput(MasterController,pros::E_CONTROLLER_ANALOG_LEFT_Y,RopoController::Exp);
	RopoController::AxisValueCast WVelocityInput(MasterController,pros::E_CONTROLLER_ANALOG_RIGHT_X,RopoController::Linear);
	RopoMath::Vector<RopoApi::FloatType> Velocity(RopoMath::ColumnVector,2),ResVelocity;
	MasterController.clear();

	ButtonDetectLine.AddButtonDetect(pros::E_CONTROLLER_DIGITAL_Y   , RopoController::Rising ,  autonomous        );
	ButtonDetectLine.AddButtonDetect(pros::E_CONTROLLER_DIGITAL_LEFT   , RopoController::Rising , ControllerModule::PositionAdjustLeft);
	ButtonDetectLine.AddButtonDetect(pros::E_CONTROLLER_DIGITAL_RIGHT   , RopoController::Rising , ControllerModule::PositionAdjustRight);
	ButtonDetectLine.AddButtonDetect(pros::E_CONTROLLER_DIGITAL_DOWN   , RopoController::Rising ,  [](void)->void{
		RopoDevice::Thrower_Motor::MyThrower.set_is_disable(true);
		pros::delay(100);
		RopoDevice::Motors::ThrowerMotor.move_voltage(5000);
	});
	ButtonDetectLine.AddButtonDetect(pros::E_CONTROLLER_DIGITAL_UP   , RopoController::Rising ,  [](void)->void{
		RopoDevice::Thrower_Motor::MyThrower.set_is_disable(true);
		pros::delay(100);
		RopoDevice::Motors::ThrowerMotor.move_voltage(-10000);
	});
	ButtonDetectLine.AddButtonDetect(pros::E_CONTROLLER_DIGITAL_DOWN   , RopoController::Falling ,  [](void)->void{
		RopoDevice::Motors::ThrowerMotor.move_voltage(-2000);
	});
	ButtonDetectLine.AddButtonDetect(pros::E_CONTROLLER_DIGITAL_UP   , RopoController::Falling ,  [](void)->void{
		RopoDevice::Motors::ThrowerMotor.move_voltage(-2000);
	});	
	ButtonDetectLine.AddButtonDetect(pros::E_CONTROLLER_DIGITAL_A   , RopoController::Rising ,  RopoDevice::DeviceInitOp2);
	ButtonDetectLine.AddButtonDetect(pros::E_CONTROLLER_DIGITAL_L2   , RopoController::Rising ,  ControllerModule::Push );
	ButtonDetectLine.AddButtonDetect(pros::E_CONTROLLER_DIGITAL_L2   , RopoController::Falling,  ControllerModule::Pull );
	ButtonDetectLine.AddButtonDetect(pros::E_CONTROLLER_DIGITAL_L1  , RopoController::Falling , [](void)->void{
		pros::delay(2000);
		ControllerModule::Hide();
	} );
	ButtonDetectLine.AddButtonDetect(pros::E_CONTROLLER_DIGITAL_L1  , RopoController::DoubleEdge , ControllerModule::L1BoolSwitch,&ControllerModule::L1Pressing );
	ButtonDetectLine.AddButtonDetect(pros::E_CONTROLLER_DIGITAL_B  , RopoController::Rising , ControllerModule::ChangeHideAndWait  );
	ButtonDetectLine.AddButtonDetect(pros::E_CONTROLLER_DIGITAL_X   , RopoController::Rising ,  RopoDevice::DeviceInitOp1 );
	ButtonDetectLine.AddButtonDetect(pros::E_CONTROLLER_DIGITAL_R1, RopoController::Rising, ControllerModule::Intake);
	ButtonDetectLine.AddButtonDetect(pros::E_CONTROLLER_DIGITAL_R1, RopoController::Falling, ControllerModule::IntakerRest);
	ButtonDetectLine.AddButtonDetect(pros::E_CONTROLLER_DIGITAL_R2, RopoController::Rising, ControllerModule::ChangeIntakerHideMode);
	ButtonDetectLine.Enable();
	
	while (true) {
		RopoApi::FloatType XInput =  XVelocityInput.GetAxisValue();
		RopoApi::FloatType WInput = -WVelocityInput.GetAxisValue();
		RopoApi::FloatType RopoWc = RopoWcLimit-fabs(XInput)*1.3;			

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
		//pros::lcd::set_background_color(0,100,0);
		pros::lcd::print(1,"Ready!!! V:%.1f %.1f",XInput,WInput);
		pros::delay(5);
	}
}
