#include "main.h"
#include "RopoController.hpp"
#include "RopoDevice.hpp"
#include "RopoPosition.hpp"
#include "pros/misc.h"
#include "pros/rtos.hpp"
void autonomous_2();
namespace ControllerModule {

	bool timeFlag = 0;

	void BoolSwitch(bool * Parameter){
		bool *p = Parameter;
		(*p) ^= 1;
	}

	// 挂杆
	bool trigger_flag = false;
		// 钩爪升起，触发flag置1
		void UpperExternSwitch(){
			RopoDevice::ThreeWire::UpperExternPneumatic.set_value(true);
			trigger_flag = true;
		}
		// 钩爪发力挂杆
		void UnderExternSwitch(){
			RopoDevice::ThreeWire::UnderExternPneumatic.set_value(true);
		}
	void Hangingup(){
		if(trigger_flag == 0){
			UpperExternSwitch();
		}else{
			UnderExternSwitch();
		}
	}

	bool backExternFlag = false;
	void BackExternChange(){
		backExternFlag ^= 1;
		RopoDevice::ThreeWire::BackExternPneumatic.set_value(backExternFlag);
	}

	void BackExternPush(){
		backExternFlag = true;
		RopoDevice::ThreeWire::BackExternPneumatic.set_value(backExternFlag);
	}

	void BackExternPull(){
		backExternFlag = false;
		RopoDevice::ThreeWire::BackExternPneumatic.set_value(backExternFlag);
	}

	bool frontExternFlag = false;
	void FrontExternChange(){
		frontExternFlag ^= 1;
		RopoDevice::ThreeWire::FrontExternPneumatic.set_value(frontExternFlag);
	}

	void FrontExternPush(){
		frontExternFlag = true;
		RopoDevice::ThreeWire::FrontExternPneumatic.set_value(frontExternFlag);
	}

	void FrontExternPull(){
		frontExternFlag = false;
		RopoDevice::ThreeWire::FrontExternPneumatic.set_value(frontExternFlag);
	}

	bool wideExternFlag = false;
	void WideExternSwitch(){
		wideExternFlag ^= 1; 
		// RopoDevice::ThreeWire::WideExternPneumatic.set_value(wideExternFlag);
	}

	void RumbleMe(){
		pros::Controller MasterController1(pros::E_CONTROLLER_MASTER);
		MasterController1.rumble("-.-.-");
		FloatType aa = pros::millis();
		while(pros::millis()-aa<45000){
			pros::delay(100);
		}
		MasterController1.rumble("-.-");
		while(pros::millis()-aa<55000){
			pros::delay(100);
		}
		RopoDevice::ChassisBrake();
		MasterController1.rumble("-.-.-");
		while(pros::millis()-aa<65000){
			pros::delay(100);
		}
		MasterController1.rumble("-.-.-.-");
	}


	bool intakerpusher_flag = false; // 0: back 1: for
	int intaker_flag = 0;			 //  0: stop 1: intake 2:outtake

	void SwitchIntakerpusherback(){
		intakerpusher_flag = false;
		RopoDevice::ThreeWire::IntakePusherPneumatic.set_value(intakerpusher_flag);
	}

	void SwitchIntakerpusherfor(){
		intakerpusher_flag = true;
		RopoDevice::ThreeWire::IntakePusherPneumatic.set_value(intakerpusher_flag);
	}

	void SwitchIntakerpusherchange(){
		if (intakerpusher_flag){
			SwitchIntakerpusherback();
		}else{
			SwitchIntakerpusherfor();
		}

	}

	void SwitchIntakerintake(){
		intaker_flag = 1;
		RopoDevice::Motors::IntakerMoveVoltage(12000);
	}

	void SwitchIntakerouttake(){
		intaker_flag = 0;
		RopoDevice::Motors::IntakerMoveVoltage(-12000);
	}

	void SwitchIntakerStop(){
		intaker_flag = 0;
		RopoDevice::Motors::IntakerMoveVoltage(0);
	}

	void SwitchIntakerChange(){
		if(intaker_flag == 2) {
			SwitchIntakerintake();
		}else if(intaker_flag == 1) {
			SwitchIntakerouttake();
		}
		else{
			SwitchIntakerStop();	// 停止状态change仍停止
		}
	}

	void TurnAround(){
		RopoDevice::Chassis.AutoRotateRelative(180);
	}

	void GpsUpdate(){
		RopoDevice::gpsAddPosition.GpsUpdate();
	}

	void AutoLift(){
		

	}

	void ControllerPrint(){
		while(true) {
			pros::Controller MasterController(pros::E_CONTROLLER_MASTER);
			MasterController.print(0,1,"degree: %.1lf",RopoDevice::GetPosition()[3]);
			pros::delay(50); 
			MasterController.print(2,1,"X: %.2lf Y:%.2lf",(RopoDevice::GetTransformedPosition())[1],(RopoDevice::GetTransformedPosition())[2]);
			pros::delay(50); 
			MasterController.print(1,1,"degree: %.1lf",-RopoDevice::Sensors::Inertial.get_rotation()*1.017);
			pros::delay(50); 
			// pros::delay(50);
		}
	}
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
	autonomous_2();
}

void opcontrol()
{
	//pros::Task *RumbleTask = new pros::Task(ControllerModule::RumbleMe);
	pros::Task *PrintTask = new pros::Task(ControllerModule::ControllerPrint);
	pros::Controller MasterController(pros::E_CONTROLLER_MASTER);
	RopoController::ButtonTaskLine ButtonDetectLine(MasterController);
	FloatType VelocityMax = 1.60;					// 1.7 m/s	// 2.0
	FloatType WcMax = 20;							// 7	// 10 
	FloatType VelocityRestrainRatio = 0.4; 			// 0 ~ 1
	FloatType WcRestrainRatio = 0.4; 				// 0 ~ 1
	bool ChassisMove = false;
	const FloatType nowTime = pros::millis();
	RopoDevice::ChassisCoast();
	
	RopoController::AxisValueCast XVelocityInput(MasterController,pros::E_CONTROLLER_ANALOG_LEFT_Y,RopoController::Linear);
	RopoController::AxisValueCast WVelocityInput(MasterController,pros::E_CONTROLLER_ANALOG_RIGHT_X,RopoController::Exp);

	Vector Velocity(RopoMath::ColumnVector,2),ResVelocity;

	MasterController.clear();
	ButtonDetectLine.AddButtonDetect(pros::E_CONTROLLER_DIGITAL_R1   , RopoController::Rising, ControllerModule::SwitchIntakerintake);
	ButtonDetectLine.AddButtonDetect(pros::E_CONTROLLER_DIGITAL_R1   , RopoController::Falling,ControllerModule::SwitchIntakerStop);
	ButtonDetectLine.AddButtonDetect(pros::E_CONTROLLER_DIGITAL_R2   , RopoController::Rising, ControllerModule::SwitchIntakerouttake);
	ButtonDetectLine.AddButtonDetect(pros::E_CONTROLLER_DIGITAL_R2   , RopoController::Falling,ControllerModule::SwitchIntakerStop);
	ButtonDetectLine.AddButtonDetect(pros::E_CONTROLLER_DIGITAL_L1   , RopoController::Rising, ControllerModule::FrontExternChange);
	ButtonDetectLine.AddButtonDetect(pros::E_CONTROLLER_DIGITAL_L2   , RopoController::Rising, ControllerModule::BackExternChange);
	ButtonDetectLine.AddButtonDetect(pros::E_CONTROLLER_DIGITAL_UP   , RopoController::Rising, ControllerModule::SwitchIntakerpusherchange);
	ButtonDetectLine.AddButtonDetect(pros::E_CONTROLLER_DIGITAL_DOWN , RopoController::Rising, ControllerModule::Hangingup);
	ButtonDetectLine.AddButtonDetect(pros::E_CONTROLLER_DIGITAL_X    , RopoController::Rising, ControllerModule::GpsUpdate);
	// ButtonDetectLine.AddButtonDetect(pros::E_CONTROLLER_DIGITAL_B    , RopoController::Rising, ControllerModule::ChangeCatch);
	// ButtonDetectLine.AddButtonDetect(pros::E_CONTROLLER_DIGITAL_A    , RopoController::Rising, RopoDevice::ChassisBrake);
	// ButtonDetectLine.AddButtonDetect(pros::E_CONTROLLER_DIGITAL_Y    , RopoController::Rising,  autonomous_2);

	ButtonDetectLine.Enable();
	RopoDevice::ChassisCoast();

	while (true) {

		FloatType XInput =  XVelocityInput.GetAxisValue();
		FloatType WInput = -WVelocityInput.GetAxisValue();
		FloatType RopoWc = WcMax * (1.0 - fabs(XInput) * WcRestrainRatio);
		FloatType RopoVx = VelocityMax * (1.0 - fabs(WInput) * VelocityRestrainRatio);

		if (fabs(XInput) <= 0.06 && fabs(WInput) <= 0.03) {
			if(ChassisMove == true){
				// RopoDevice::ChassisCoast();
				RopoDevice::Motors::MoveOpControll(0.0, 0.0);
				ChassisMove = false;
			}
		} else {
			RopoDevice::Chassis.StartChassisOpControll(); // 底盘MoveType设置为OpMove
			RopoDevice::Motors::MoveOpControll(XInput * RopoVx, WInput * RopoWc);
			ChassisMove = true;
		}
		pros::delay(4);
	}

	if (pros::millis() - nowTime > 55000) {
		ControllerModule::timeFlag = 1;
		ControllerModule::UnderExternSwitch();
	}
}

void delay(){ //代替pros::delay()进行时间控制
	while(!RopoDevice::Chassis.IfArrived()){
		pros::delay(1);
	}
	return;
}

void autonomous_2(){
	RopoDevice::Chassis.StartChassisAutoControll();//底盘MoveType设置为AutoMove
	// --------- begin ------------

	
}
