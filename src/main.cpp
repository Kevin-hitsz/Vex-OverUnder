#include "main.h"
#include "RopoController.hpp"
#include "RopoDevice.hpp"
#include "RopoPosition.hpp"
#include "pros/misc.h"
#include "pros/rtos.hpp"
namespace ControllerModule{
	bool ifHide = false;
	void Throw(void){
		RopoDevice::Thrower_Motor::MyThrower.Throw();
		RopoDevice::Intaker_Motor::MyIntaker.SetStayHidingMode(true);
		ifHide = false;
	}
	void Hide(void){
		RopoDevice::Thrower_Motor::MyThrower.Wait();
		while(!RopoDevice::Thrower_Motor::MyThrower.IfReady()){
			pros::delay(20);
		}
		RopoDevice::Thrower_Motor::MyThrower.Hide();
		RopoDevice::Intaker_Motor::MyIntaker.SetStayHidingMode(false);
		ifHide = true;
	}
	int cnt = 0;
	void Wait(void){
		if (cnt >= 1){
			RopoDevice::Thrower_Motor::MyThrower.Wait();
			RopoDevice::Intaker_Motor::MyIntaker.SetStayHidingMode(true);
			ifHide = false;
		}
		cnt ++;
		
	}

	void ChangeHideAndWait(void){
		if (ifHide == true) {
			Wait();
		}
		else {
			Hide();
		}
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
		if(RopoDevice::Intaker_Motor::MyIntaker.GetIntakeMode() == false){
			ChangeIntakerHideMode();
		}
		if(RopoDevice::Intaker_Motor::MyIntaker.GetRestingMode()){
			RopoDevice::Intaker_Motor::MyIntaker.ChangeRestMode();
		}
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
			MasterController2.print(2,1,"X: %.3lf Y:%.2lf F:%1d",RopoDevice::Sensors::myOpenMv.GetDistance(),RopoDevice::Sensors::myOpenMv.GetTheta(),RopoDevice::Sensors::myOpenMv.GetExists());
			pros::delay(10); 
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

void autonomous() {
	//------塞球
	RopoDevice::DeviceInitOp2();
	ControllerModule::Wait();
	pros::delay(2450);
	RopoDevice::Thrower_Motor::MyThrower.set_is_disable(true);
	RopoDevice::Motors::ThrowerMotor.move_voltage(-5000);
	pros::delay(230);
	pros::delay(300);
	RopoDevice::Motors::ThrowerMotor.move_voltage(-1600);
	RopoDevice::Chassis.MoveVelocity(1,0);
	pros::delay(400);
	
	pros::delay(100);
	RopoDevice::Chassis.MoveVelocity(0,0);
	pros::delay(150);
	RopoDevice::Chassis.MoveVelocity(1,-1.1);
	pros::delay(350);
	RopoDevice::Chassis.MoveVelocity(0.9,0);
	pros::delay(100);
	RopoDevice::Chassis.MoveVelocity(0.46,-0.93);
	pros::delay(400);
	RopoDevice::Chassis.MoveVelocity(0.1,0);
	pros::delay(100);
	RopoDevice::Chassis.MoveVelocity(0,0);
	pros::delay(100);
	RopoDevice::Chassis.AutoRotateAbs(-125);
	while (!RopoDevice::Chassis.IfDegArrived()){
	 	pros::delay(20);
	}
	RopoDevice::Chassis.MoveVelocity(-1.4,0);
	pros::delay(300);
	RopoDevice::Chassis.MoveVelocity(-0.6,0);
	pros::delay(200);
	
	RopoDevice::Chassis.MoveVelocity(-0.45,1.5);
	pros::delay(100);
	RopoDevice::Motors::ThrowerMotor.move_voltage(0);
	pros::delay(620);
	RopoDevice::Chassis.MoveVelocity(-0.6,0.8);
	pros::delay(50);
	RopoDevice::Chassis.MoveVelocity(-1.1,0);
	pros::delay(190);
	//--------推球
	RopoDevice::Chassis.MoveVelocity(0,0);
	ControllerModule::ChangeIntakerHideMode();
	RopoDevice::Chassis.AutoRotateAbs(-45);
	pros::delay(1000);
	
	RopoDevice::Chassis.MoveVelocity(-1,0);
	pros::delay(200);
	ControllerModule::Push();
	RopoDevice::Chassis.MoveVelocity(1.8,0);
	pros::delay(750);
	RopoDevice::Chassis.MoveVelocity(0.8,0);
	pros::delay(800);
	ControllerModule::Hide();
	ControllerModule::Pull();
	RopoDevice::Chassis.MoveVelocity(-0.5,0);
	pros::delay(400);
	RopoDevice::Chassis.MoveVelocity(0.5,0);
	pros::delay(1000);

	RopoDevice::Position_Motor::MyPosition.SetXAndY(1.47, -0.03);
	RopoDevice::Chassis.MoveVelocity(-0.6,1.1);
	pros::delay(900);
	RopoDevice::Chassis.MoveVelocity(-0.6,0);
	pros::delay(300);
	RopoDevice::Chassis.MoveVelocity(-0.3,-0.75);
	pros::delay(500);
	RopoDevice::Chassis.MoveVelocity(-0.3,0);
	pros::delay(200);
	RopoDevice::Chassis.MoveVelocity(0,0);
	//------回去抛球
	RopoDevice::Chassis.AutoMovePosAbsBack(0.3,0,0);
	while (!RopoDevice::Chassis.IfArrived()){
	 	pros::delay(20);
		RopoDevice::Position_Motor::MyPosition.SetXAndY(RopoDevice::Position_Motor::MyPosition.Get_X(), 0);
	}
	RopoDevice::Chassis.MoveVelocity(-0.6,0);
	pros::delay(550);
	RopoDevice::Chassis.MoveVelocity(-0.15,0);
	pros::delay(300);
	
	for(int i = 0; i < 12; i++){
		
		ControllerModule::Wait();
		pros::delay(2000);
		ControllerModule::Throw();
		pros::delay(100);
	}
	RopoDevice::Position_Motor::MyPosition.SetXAndY(0, 0);
	
	ControllerModule::Intake();
	pros::delay(200);
	ControllerModule::Intake();
	
	// RopoDevice::Chassis.AutoMovePosAbsBack(0.84,-0.97,-45);
	// while (!RopoDevice::Chassis.IfArrived()){
	//  	pros::delay(20);
	// }

	//----碰杆
	RopoDevice::Chassis.MoveVelocity(0.1,0);
	pros::delay(100);
	RopoDevice::Chassis.MoveVelocity(0.6,-1.5);
	pros::delay(350);
	ControllerModule::Wait();
	pros::delay(400);
	ControllerModule::IntakerRest();
	RopoDevice::Chassis.MoveVelocity(1,0);
	pros::delay(160);
	RopoDevice::Chassis.MoveVelocity(0.8,1);
	pros::delay(520);
	RopoDevice::Chassis.MoveVelocity(1.35,0);
	pros::delay(700);
	RopoDevice::Chassis.MoveVelocity(0,0);

	ControllerModule::Hide();

}

namespace QuickHandControl{
	static bool RollerR1Tag = false;
	static bool RollerR2Tag = false;

	void RollerStatusUpdate(){
		if (RollerR1Tag && !RollerR2Tag)
			RopoDevice::Motors::RollerMotor.move_velocity(100);
		else if (RollerR2Tag && !RollerR1Tag)
			RopoDevice::Motors::RollerMotor.move_velocity(-100);
		else
			RopoDevice::Motors::RollerMotor.move_velocity(0);			// 逻辑有待优化！！！！！
		pros::delay(20);
	}

	void RollerBoolSwitch(void *param){
		bool *p = static_cast<bool *>(param);
		(*p) ^= 1;
		RollerStatusUpdate();
	}

}


void opcontrol() {
	RopoDevice::DeviceInitOp1();
	pros::Task *RumbleTask = new pros::Task(ControllerModule::RumbleMe);
	pros::Task *DisplayTask = new pros::Task(ControllerModule::Display);
	pros::Controller MasterController(pros::E_CONTROLLER_MASTER);
	RopoController::ButtonTaskLine ButtonDetectLine(MasterController);
	RopoApi::FloatType VelocityMax = 2;
	RopoApi::FloatType RopoWcLimit = 5;
	bool ChassisMove = false;
	
	RopoController::AxisValueCast XVelocityInput(MasterController,pros::E_CONTROLLER_ANALOG_LEFT_Y,RopoController::Exp);
	RopoController::AxisValueCast WVelocityInput(MasterController,pros::E_CONTROLLER_ANALOG_RIGHT_X,RopoController::Linear);
	RopoMath::Vector<RopoApi::FloatType> Velocity(RopoMath::ColumnVector,2),ResVelocity;
	MasterController.clear();

	ButtonDetectLine.AddButtonDetect(pros::E_CONTROLLER_DIGITAL_Y   , RopoController::Rising ,  autonomous        );
	ButtonDetectLine.AddButtonDetect(pros::E_CONTROLLER_DIGITAL_LEFT   , RopoController::Rising ,  [](void)->void{
		RopoDevice::Thrower_Motor::MyThrower.set_is_disable(true);
		RopoDevice::Motors::ThrowerMotor.move_voltage(5000);
	},3000);
	ButtonDetectLine.AddButtonDetect(pros::E_CONTROLLER_DIGITAL_RIGHT   , RopoController::Rising ,  [](void)->void{
		RopoDevice::Thrower_Motor::MyThrower.set_is_disable(true);
		RopoDevice::Motors::ThrowerMotor.move_voltage(-6000);
	});
	ButtonDetectLine.AddButtonDetect(pros::E_CONTROLLER_DIGITAL_LEFT   , RopoController::Falling ,  [](void)->void{
		RopoDevice::Motors::ThrowerMotor.move_voltage(-2000);
	});
	ButtonDetectLine.AddButtonDetect(pros::E_CONTROLLER_DIGITAL_RIGHT   , RopoController::Falling ,  [](void)->void{
		RopoDevice::Motors::ThrowerMotor.move_voltage(-2000);
	});
	ButtonDetectLine.AddButtonDetect(pros::E_CONTROLLER_DIGITAL_UP   , RopoController::Rising ,  [](void)->void{
		RopoDevice::DeviceInitOp2();
		RopoDevice::Chassis.MoveVelocity(-0.6,1.1);
		pros::delay(900);
		RopoDevice::Chassis.MoveVelocity(-0.6,0);
		pros::delay(100);
		RopoDevice::Chassis.MoveVelocity(-0.3,-0.75);
		pros::delay(500);
		RopoDevice::Chassis.MoveVelocity(-0.3,0);
		pros::delay(200);
		RopoDevice::Chassis.MoveVelocity(0,0);
	});
	ButtonDetectLine.AddButtonDetect(pros::E_CONTROLLER_DIGITAL_B   , RopoController::Rising ,  ControllerModule::Push );
	ButtonDetectLine.AddButtonDetect(pros::E_CONTROLLER_DIGITAL_B   , RopoController::Falling,  ControllerModule::Pull );
	ButtonDetectLine.AddButtonDetect(pros::E_CONTROLLER_DIGITAL_L1  , RopoController::Rising , ControllerModule::Throw );
	ButtonDetectLine.AddButtonDetect(pros::E_CONTROLLER_DIGITAL_L1  , RopoController::Falling, ControllerModule::Wait  );
	ButtonDetectLine.AddButtonDetect(pros::E_CONTROLLER_DIGITAL_L2  , RopoController::Rising , ControllerModule::ChangeHideAndWait  );
	ButtonDetectLine.AddButtonDetect(pros::E_CONTROLLER_DIGITAL_A   , RopoController::DoubleClick ,  RopoDevice::DeviceInitOp1 );
	ButtonDetectLine.AddButtonDetect(pros::E_CONTROLLER_DIGITAL_A   , RopoController::Pressing ,  RopoDevice::DeviceInitOp2 ,1000);
	ButtonDetectLine.AddButtonDetect(pros::E_CONTROLLER_DIGITAL_R1, RopoController::Rising, ControllerModule::Intake);
	ButtonDetectLine.AddButtonDetect(pros::E_CONTROLLER_DIGITAL_R1, RopoController::Falling, ControllerModule::IntakerRest);
	ButtonDetectLine.AddButtonDetect(pros::E_CONTROLLER_DIGITAL_R2, RopoController::Rising, ControllerModule::ChangeIntakerHideMode);
	ButtonDetectLine.Enable();

	while (true) {
		RopoApi::FloatType XInput =  XVelocityInput.GetAxisValue();
		RopoApi::FloatType WInput = -WVelocityInput.GetAxisValue();
		RopoApi::FloatType RopoWc = RopoWcLimit-XInput*2;			

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
		pros::delay(1);
	}
}
