#include "main.h"
#include "RopoController.hpp"
#include "RopoMath/Misc.hpp"
#include "RopoDevice.hpp"
#include "RopoPosition.hpp"
#include "pros/misc.h"
#include "pros/rtos.hpp"
void autonomous_1();
void autonomous_2();
void skill();
void test();
namespace ControllerModule {

	bool timeFlag = 0;

	void BoolSwitch(bool * Parameter){
		bool *p = Parameter;
		(*p) ^= 1;
	}

	// 挂杆
	int trigger_flag = 0; // 0：触发 0    1：触发 1 & 主气缸 0    2：触发 1 & 主气缸 1
		// 钩爪升起，触发flag置1
		void UpperExternSwitch(){
			RopoDevice::ThreeWire::UpperExternPneumatic.set_value(true);
			trigger_flag = 1;
		}
		// 钩爪发力挂杆，触发flag置2
		void UnderExternSwitch(){
			RopoDevice::ThreeWire::UnderExternPneumatic.set_value(true);
			trigger_flag = 2;
		}
		// 钩爪卸力松杆，触发flag置1
		void DisUnderExternSwitch(){
			RopoDevice::ThreeWire::UnderExternPneumatic.set_value(false);
			trigger_flag = 1;
		}
	void Hangingup(){
		if(trigger_flag == 0){
			UpperExternSwitch();
		}else if(trigger_flag == 1){
			UnderExternSwitch();
		}else if(trigger_flag == 2){
			DisUnderExternSwitch();
		}
	}

	bool LeftExternFlag = false;
	void LeftExternChange(){
		LeftExternFlag ^= 1;
		RopoDevice::ThreeWire::LeftExternPneumatic.set_value(LeftExternFlag);
	}

	void LeftExternPush(){
		LeftExternFlag = true;
		RopoDevice::ThreeWire::LeftExternPneumatic.set_value(LeftExternFlag);
	}

	void LeftExternPull(){
		LeftExternFlag = false;
		RopoDevice::ThreeWire::LeftExternPneumatic.set_value(LeftExternFlag);
	}

	bool RightExternFlag = false;
	void RightExternChange(){
		RightExternFlag ^= 1;
		RopoDevice::ThreeWire::RightExternPneumatic.set_value(RightExternFlag);
	}

	void RightExternPush(){
		RightExternFlag = true;
		RopoDevice::ThreeWire::RightExternPneumatic.set_value(RightExternFlag);
	}

	void RightExternPull(){
		RightExternFlag = false;
		RopoDevice::ThreeWire::RightExternPneumatic.set_value(RightExternFlag);
	}

	void BothExternChange(){
		RightExternChange();
		LeftExternChange();
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


	bool intakerpusher_flag = false; 	// 0: for 1: back
	int intaker_flag = 0;			 	// 0: stop 1: intake 2:outtake

	void SwitchIntakerpusherback(){
		intakerpusher_flag = true;
		RopoDevice::ThreeWire::IntakePusherPneumatic.set_value(intakerpusher_flag);
	}

	void SwitchIntakerpusherfor(){
		intakerpusher_flag = false;
		RopoDevice::ThreeWire::IntakePusherPneumatic.set_value(intakerpusher_flag);
	}

	void SwitchIntakerpusherchange(){
		if (intakerpusher_flag){
			SwitchIntakerpusherfor();
		}else{
			SwitchIntakerpusherback();
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

	void HitOut(){
		RopoDevice::Motors::HitMoveVoltage(12000);
		if(RopoDevice::Motors::HitMotor.get_torque() == 1.9){
			RopoDevice::Motors::HitMoveVoltage(0);
		}
	}

	void HitIn(){
		RopoDevice::Motors::HitMoveVoltage(-12000);
		if(RopoDevice::Motors::HitMotor.get_torque() == 1.9){
			RopoDevice::Motors::HitMoveVoltage(0);
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
	ControllerModule::SwitchIntakerpusherfor();
}

void disabled() {}

void competition_initialize() {}

void autonomous(){
	autonomous_1();
}

void opcontrol()
{
	//pros::Task *RumbleTask = new pros::Task(ControllerModule::RumbleMe);
	pros::Task *PrintTask = new pros::Task(ControllerModule::ControllerPrint);
	pros::Controller MasterController(pros::E_CONTROLLER_MASTER);
	RopoController::ButtonTaskLine ButtonDetectLine(MasterController);
	FloatType VelocityMax = 2.0;					// 1.7 m/s
	FloatType WcMax = 15;							// 20 
	FloatType VelocityRestrainRatio = 0.4; 			// 0 ~ 1
	FloatType WcRestrainRatio = 0.6; 				// 0 ~ 1
	bool ChassisMove = false;
	const FloatType nowTime = pros::millis();
	RopoDevice::ChassisCoast();
	
	RopoController::AxisValueCast XVelocityInput(MasterController,pros::E_CONTROLLER_ANALOG_LEFT_Y,RopoController::Linear);
	RopoController::AxisValueCast WVelocityInput(MasterController,pros::E_CONTROLLER_ANALOG_RIGHT_X,RopoController::Linear);

	Vector Velocity(RopoMath::ColumnVector,2),ResVelocity;

	MasterController.clear();
	ButtonDetectLine.AddButtonDetect(pros::E_CONTROLLER_DIGITAL_R1   , RopoController::Rising, ControllerModule::SwitchIntakerintake);
	// ButtonDetectLine.AddButtonDetect(pros::E_CONTROLLER_DIGITAL_R1   , RopoController::Falling,ControllerModule::SwitchIntakerStop);
	ButtonDetectLine.AddButtonDetect(pros::E_CONTROLLER_DIGITAL_R2   , RopoController::Rising, ControllerModule::SwitchIntakerouttake);
	ButtonDetectLine.AddButtonDetect(pros::E_CONTROLLER_DIGITAL_R2   , RopoController::Falling,ControllerModule::SwitchIntakerStop);
	ButtonDetectLine.AddButtonDetect(pros::E_CONTROLLER_DIGITAL_L1   , RopoController::Rising, ControllerModule::BothExternChange);
	ButtonDetectLine.AddButtonDetect(pros::E_CONTROLLER_DIGITAL_L2   , RopoController::Rising, ControllerModule::SwitchIntakerpusherchange);
	// ButtonDetectLine.AddButtonDetect(pros::E_CONTROLLER_DIGITAL_UP   , RopoController::Rising, ControllerModule::LeftExternChange);
	ButtonDetectLine.AddButtonDetect(pros::E_CONTROLLER_DIGITAL_DOWN , RopoController::Rising, ControllerModule::Hangingup);
	ButtonDetectLine.AddButtonDetect(pros::E_CONTROLLER_DIGITAL_RIGHT, RopoController::Rising, ControllerModule::HitOut);
	ButtonDetectLine.AddButtonDetect(pros::E_CONTROLLER_DIGITAL_LEFT, RopoController::Rising, ControllerModule::HitIn);
	ButtonDetectLine.AddButtonDetect(pros::E_CONTROLLER_DIGITAL_X    , RopoController::Rising, ControllerModule::GpsUpdate);
	ButtonDetectLine.AddButtonDetect(pros::E_CONTROLLER_DIGITAL_Y    , RopoController::Rising,  autonomous);
	ButtonDetectLine.AddButtonDetect(pros::E_CONTROLLER_DIGITAL_B    , RopoController::Rising, skill);
	// ButtonDetectLine.AddButtonDetect(pros::E_CONTROLLER_DIGITAL_A    , RopoController::Rising, RopoDevice::ChassisBrake);

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
			FloatType XVelocity, YVelocity;
			XVelocity = RopoMath::LowPassFilter<FloatType>(XInput * RopoVx,XVelocity, 8,1000.0 / 4);
			YVelocity = RopoMath::LowPassFilter<FloatType>(WInput * RopoWc,YVelocity, 8,1000.0 / 4);
			RopoDevice::Motors::MoveOpControll(XVelocity, YVelocity);
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

void test(){
	
}

// 资格赛
void autonomous_1(){
	RopoDevice::Chassis.StartChassisAutoControll();//底盘MoveType设置为AutoMove
	RopoDevice::ChassisBrake();
	// --------- begin ------------

	// 等队友走
	// pros::delay(200);
	// 己方导入四个球：开环旋转前进，闭环回原位
	ControllerModule::RightExternChange();
	pros::delay(500);
	// for(int i=0; i<3; i++){
	// 	RopoDevice::Chassis.MoveVelocity(1,6);
	// 	pros::delay(250);
	// 	RopoDevice::Chassis.MoveVelocity(0,0);
	// 	pros::delay(100);
	// 	RopoDevice::Chassis.AutoPositionMoveBack(0,0,0);
	// 	delay();
	// }
	// RopoDevice::Chassis.MoveVelocity(1,6);	// 第四个球直接往前走通道
	// pros::delay(250); 
	// ControllerModule::RightExternChange();
	// RopoDevice::Chassis.MoveVelocity(0,0);
	// pros::delay(200);
	// 前两个开环来回
	for(int i=0; i<2; i++){
		RopoDevice::Chassis.MoveVelocity(1,6);
		pros::delay(250);
		RopoDevice::Chassis.MoveVelocity(0,0);
		pros::delay(100);
		RopoDevice::Chassis.MoveVelocity(-1,-6);
		pros::delay(250);
		RopoDevice::Chassis.MoveVelocity(0,0);
		pros::delay(900);
	}
	// 第三个闭环回原位
	RopoDevice::Chassis.MoveVelocity(1,6);
	pros::delay(250);
	RopoDevice::Chassis.MoveVelocity(0,0);
	pros::delay(100);
	RopoDevice::Chassis.AutoPositionMoveBack(0,0,0);
	delay();
	// 第四个直接走通道
	RopoDevice::Chassis.MoveVelocity(1,6);	
	pros::delay(250); 
	ControllerModule::RightExternChange();
	RopoDevice::Chassis.MoveVelocity(0,0);
	pros::delay(200);
	RopoDevice::Chassis.AutoPositionMove(0.35,0);
	delay();
	RopoDevice::Chassis.AutoRotateAbs(50);
	delay();
		
	// 沿通道推入网
	// 直行：开环，开右边翅膀
	ControllerModule::RightExternChange();
	RopoDevice::Chassis.MoveVelocity(0.8,0);
	pros::delay(2650); 	// 2350
	RopoDevice::Chassis.AutoRotateAbs(62);
	delay();
	// 转弯：开环
	RopoDevice::Chassis.MoveVelocity(0.67,1.5);		// 0.67, 0.95
	pros::delay(1700);								// 1700
	pros::delay(500);
	RopoDevice::Chassis.MoveVelocity(0,0);
	if (fabs(RopoDevice::GetPosition()[3] - 140) >= 5){
		RopoDevice::Chassis.AutoRotateAbs(140);
		delay();
	}
	ControllerModule::RightExternChange();

	// 处理对面进攻区三个球
	// 第一个球
	RopoDevice::Chassis.MoveVelocity(-1.8,0);
	pros::delay(300);
	RopoDevice::Chassis.MoveVelocity(0,0);
	RopoDevice::Chassis.AutoRotateAbs(-158);	// -163
	delay();
	ControllerModule::SwitchIntakerpusherback();
	ControllerModule::GpsUpdate();	
	pros::delay(200);
	RopoDevice::Chassis.AutoPositionMove(-0.80,-0.42);	// -0.77,-0.27
	delay();
	ControllerModule::SwitchIntakerintake();
	pros::delay(800);
	RopoDevice::Chassis.MoveVelocity(0,-7);
	pros::delay(500);
	RopoDevice::Chassis.MoveVelocity(0,0);
	ControllerModule::SwitchIntakerouttake();
	pros::delay(400);
	ControllerModule::SwitchIntakerpusherfor();
	pros::delay(600);
	// 第二、三个球
	ControllerModule::SwitchIntakerpusherback();
	ControllerModule::SwitchIntakerintake();
	RopoDevice::Chassis.AutoPositionMove(-1.21,-0.23, 150);	// -1.12,-0.11, 150
	delay();
	pros::delay(300);
	RopoDevice::Chassis.MoveVelocity(0.2,-5);
	pros::delay(500);
	RopoDevice::Chassis.MoveVelocity(0,0);
	ControllerModule::RightExternChange();
	ControllerModule::LeftExternChange();
	RopoDevice::Chassis.AutoRotateAbs(49);
	delay();
	RopoDevice::Chassis.MoveVelocity(1,0);
	pros::delay(1000);
	ControllerModule::SwitchIntakerouttake();
	RopoDevice::Chassis.MoveVelocity(0,0);
	RopoDevice::Chassis.MoveVelocity(-1,0.5);
	pros::delay(500);
	RopoDevice::Chassis.MoveVelocity(0,0);
	ControllerModule::SwitchIntakerStop();
	ControllerModule::RightExternChange();
	ControllerModule::LeftExternChange();
	// 吃联队粽球
	ControllerModule::SwitchIntakerpusherfor();
	ControllerModule::SwitchIntakerintake();
	RopoDevice::Chassis.AutoPositionMove(0.29,-0.04,6);	// 1.85, 2.08,7
	delay();
	pros::delay(500);
	RopoDevice::Chassis.MoveVelocity(-0.8,0);
	pros::delay(800);
	ControllerModule::SwitchIntakerpusherback();
	pros::delay(1000);
	RopoDevice::Chassis.MoveVelocity(0,0);
	RopoDevice::Chassis.AutoRotateAbs(49);
	delay();
	RopoDevice::Chassis.MoveVelocity(1,0);
	pros::delay(800);
	ControllerModule::SwitchIntakerouttake();
	RopoDevice::Chassis.MoveVelocity(0,0);
	RopoDevice::Chassis.MoveVelocity(-1,0.5);
	pros::delay(500);
	// 椪杆
	ControllerModule::SwitchIntakerpusherfor();
	RopoDevice::Chassis.AutoPositionMove(-1.00, -0.71);	// 0.82, 1.58, -87
	delay();
}

// 淘汰赛
void autonomous_2(){
	RopoDevice::Chassis.StartChassisAutoControll();//底盘MoveType设置为AutoMove
	// --------- begin ------------

	// 等队友走
	pros::delay(1000);
	// 己方导入三个球：开环旋转前进，闭环回原位
	ControllerModule::RightExternChange();
	pros::delay(500);
	for(int i=0; i<2; i++){
		RopoDevice::Chassis.MoveVelocity(1,6);
		pros::delay(250);
		RopoDevice::Chassis.MoveVelocity(0,0);
		pros::delay(100);
		RopoDevice::Chassis.AutoPositionMoveBack(0,0,0);
		delay();
	}
	RopoDevice::Chassis.MoveVelocity(1,6);	// 第三个球直接往前走通道
	pros::delay(300); 
	ControllerModule::RightExternChange();
	RopoDevice::Chassis.MoveVelocity(0,0);
	pros::delay(200);
	RopoDevice::Chassis.AutoPositionMove(0.35,0);
	delay();
	RopoDevice::Chassis.AutoRotateAbs(50);
	delay();
		
	// 沿通道推入网
	// 直行：开环
	RopoDevice::Chassis.MoveVelocity(0.8,0);
	pros::delay(2650); 	// 2350
	RopoDevice::Chassis.AutoRotateAbs(62);
	delay();
	ControllerModule::RightExternChange();
	// 转弯：开环
	RopoDevice::Chassis.MoveVelocity(0.67,1.3);		// 0.67, 0.95
	pros::delay(1700);								// 1700
	ControllerModule::RightExternChange();
	pros::delay(500);
	RopoDevice::Chassis.MoveVelocity(0,0);
	if (fabs(RopoDevice::GetPosition()[3] - 140) >= 5){
		RopoDevice::Chassis.AutoRotateAbs(140);
		delay();
	}

	// 处理对面进攻区三个球
	// 第一个球
	RopoDevice::Chassis.MoveVelocity(-1.8,0);
	pros::delay(300);
	RopoDevice::Chassis.MoveVelocity(0,0);
	RopoDevice::Chassis.AutoRotateAbs(-158);
	delay();
	ControllerModule::SwitchIntakerpusherback();
	ControllerModule::GpsUpdate();	
	pros::delay(200);
	RopoDevice::Chassis.AutoPositionMove(0.69,1.81);
	delay();
	ControllerModule::SwitchIntakerintake();
	pros::delay(800);
	RopoDevice::Chassis.MoveVelocity(0,-7);
	pros::delay(500);
	RopoDevice::Chassis.MoveVelocity(0,0);
	ControllerModule::SwitchIntakerouttake();
	pros::delay(400);
	ControllerModule::SwitchIntakerpusherfor();
	pros::delay(600);
	// 第二、三个球
	ControllerModule::SwitchIntakerpusherback();
	ControllerModule::SwitchIntakerintake();
	RopoDevice::Chassis.AutoPositionMove(0.28, 1.98, 150);
	delay();
	pros::delay(300);
	RopoDevice::Chassis.MoveVelocity(0.2,-5);
	pros::delay(500);
	RopoDevice::Chassis.MoveVelocity(0,0);
	ControllerModule::RightExternChange();
	ControllerModule::LeftExternChange();
	RopoDevice::Chassis.AutoRotateAbs(49);
	delay();
	RopoDevice::Chassis.MoveVelocity(1,0);
	pros::delay(800);
	ControllerModule::SwitchIntakerouttake();
	RopoDevice::Chassis.MoveVelocity(0,0);
	RopoDevice::Chassis.MoveVelocity(-1,0.5);
	pros::delay(500);
	RopoDevice::Chassis.MoveVelocity(0,0);
	ControllerModule::SwitchIntakerStop();
	ControllerModule::RightExternChange();
	ControllerModule::LeftExternChange();
	// 过杆
	RopoDevice::Chassis.AutoRotateAbs(250);
	delay();
	RopoDevice::Chassis.MoveVelocity(10,0);
	pros::delay(1150);
	RopoDevice::Chassis.MoveVelocity(0,0);
}

// 技能赛
void skill(){
	RopoDevice::Chassis.StartChassisAutoControll();//底盘MoveType设置为AutoMove
	// --------- begin ------------

	// 第一趟 导入3+预装1+通道1+对角1
	// 导入两个球：开环旋转前进，闭环回原位
	ControllerModule::RightExternChange();
	pros::delay(500);
	for(int i=0; i<2; i++){
		RopoDevice::Chassis.MoveVelocity(1,6);
		pros::delay(250);
		RopoDevice::Chassis.MoveVelocity(0,0);
		pros::delay(100);
		RopoDevice::Chassis.AutoPositionMoveBack(0,0,0);
		delay();
	}
	RopoDevice::Chassis.MoveVelocity(1,6);	// 第三个球直接往前走通道
	pros::delay(300); 
	ControllerModule::RightExternChange();
	RopoDevice::Chassis.MoveVelocity(0,0);
	pros::delay(200);
	RopoDevice::Chassis.AutoPositionMove(0.35,0);
	delay();
	RopoDevice::Chassis.AutoRotateAbs(50);
	delay();
		
	// 沿通道推入网
	// 直行：开环
	RopoDevice::Chassis.MoveVelocity(0.8,0);
	pros::delay(2650); 	
	RopoDevice::Chassis.AutoRotateAbs(62);
	delay();
	ControllerModule::RightExternChange();
	// 转弯：开环
	RopoDevice::Chassis.MoveVelocity(0.67,1.3);		
	pros::delay(1700);								
	ControllerModule::RightExternChange();
	pros::delay(500);
	RopoDevice::Chassis.MoveVelocity(0,0);
	if (fabs(RopoDevice::GetPosition()[3] - 140) >= 5){
		RopoDevice::Chassis.AutoRotateAbs(140);
		delay();
	}
	// 后退倒车撞网（不开后翅膀）
	RopoDevice::Chassis.MoveVelocity(-0.7,0);
	pros::delay(500);
	RopoDevice::Chassis.MoveVelocity(0,0);
	RopoDevice::Chassis.AutoRotateAbs(320);
	delay();
	RopoDevice::Chassis.MoveVelocity(-5,0);
	pros::delay(500);
	RopoDevice::Chassis.MoveVelocity(0,0);
	pros::delay(300);

	// 前行返回 开intaker防通道内有球
	ControllerModule::SwitchIntakerpusherback();
	ControllerModule::SwitchIntakerintake();
	pros::delay(500);
	RopoDevice::Chassis.MoveVelocity(0.67, -1.3);
	pros::delay(1350);
	RopoDevice::Chassis.MoveVelocity(0,0);
	RopoDevice::Chassis.AutoRotateAbs(227);
	delay();
	RopoDevice::Chassis.MoveVelocity(0.8,0);
	pros::delay(2650);
	RopoDevice::Chassis.MoveVelocity(0,0);

	// 第二趟 导入10
	// 导入十个球，开环旋转，闭环回原角度
	RopoDevice::Chassis.AutoRotateAbs(50);
	delay();
	RopoDevice::Chassis.MoveVelocity(-0.5,0);
	pros::delay(400);
	RopoDevice::Chassis.MoveVelocity(0,0);
	pros::delay(500);
	ControllerModule::HitOut();
	for(int i=0; i<6; i++){
		RopoDevice::Chassis.MoveVelocity(0,7);
		pros::delay(150);
		RopoDevice::Chassis.AutoRotateAbs(50);
		delay();
	}
	RopoDevice::Chassis.MoveVelocity(0.6,-3.5);
	pros::delay(650);
	RopoDevice::Chassis.AutoRotateAbs(50);
	delay();

	// 沿通道推入网
	// 直行：开环（不收击球件）
	RopoDevice::Chassis.MoveVelocity(0.8,0);
	pros::delay(2450);
	RopoDevice::Chassis.AutoRotateAbs(62);
	delay();
	ControllerModule::RightExternChange();
	// 转弯：开环
	RopoDevice::Chassis.MoveVelocity(0.67,1.3);		
	pros::delay(1700);								
	ControllerModule::RightExternChange();
	ControllerModule::SwitchIntakerouttake();
	pros::delay(500);
	RopoDevice::Chassis.MoveVelocity(0,0);
	if (fabs(RopoDevice::GetPosition()[3] - 140) >= 5){
		RopoDevice::Chassis.AutoRotateAbs(140);
		delay();
	}
	// 后退倒车撞网（收击球件）
	ControllerModule::HitIn();
	RopoDevice::Chassis.MoveVelocity(-0.7,0);
	pros::delay(500);
	RopoDevice::Chassis.MoveVelocity(0,0);
	RopoDevice::Chassis.AutoRotateAbs(320);
	delay();
	RopoDevice::Chassis.MoveVelocity(-5,0);
	pros::delay(500);
	RopoDevice::Chassis.MoveVelocity(0,0);
	pros::delay(300);

	// 前行返回 开intaker防通道内有球
	ControllerModule::SwitchIntakerintake();
	pros::delay(500);
	RopoDevice::Chassis.MoveVelocity(0.67, -1.3);
	pros::delay(1350);
	RopoDevice::Chassis.MoveVelocity(0,0);
	RopoDevice::Chassis.AutoRotateAbs(227);
	delay();
	RopoDevice::Chassis.MoveVelocity(0.8,0);
	pros::delay(2650);
	RopoDevice::Chassis.MoveVelocity(0,0);

	// 第三趟 导入10
	// 导入十个球，开环旋转，闭环回原角度
	RopoDevice::Chassis.AutoRotateAbs(50);
	delay();
	RopoDevice::Chassis.MoveVelocity(-0.5,0);
	pros::delay(400);
	RopoDevice::Chassis.MoveVelocity(0,0);
	pros::delay(500);
	ControllerModule::HitOut();
	for(int i=0; i<6; i++){
		RopoDevice::Chassis.MoveVelocity(0,7);
		pros::delay(150);
		RopoDevice::Chassis.AutoRotateAbs(50);
		delay();
	}
	RopoDevice::Chassis.MoveVelocity(0.6,-3.5);
	pros::delay(650);
	RopoDevice::Chassis.AutoRotateAbs(50);
	delay();

	// 沿通道推入网
	// 直行：开环（不收击球件）
	RopoDevice::Chassis.MoveVelocity(0.8,0);
	pros::delay(2450);
	RopoDevice::Chassis.AutoRotateAbs(62);
	delay();
	ControllerModule::RightExternChange();
	// 转弯：开环
	RopoDevice::Chassis.MoveVelocity(0.67,1.3);		
	pros::delay(1700);								
	ControllerModule::RightExternChange();
	ControllerModule::SwitchIntakerouttake();
	pros::delay(500);
	RopoDevice::Chassis.MoveVelocity(0,0);
	if (fabs(RopoDevice::GetPosition()[3] - 140) >= 5){
		RopoDevice::Chassis.AutoRotateAbs(140);
		delay();
	}
	// 后退倒车撞网（收击球件）
	ControllerModule::HitIn();
	RopoDevice::Chassis.MoveVelocity(-0.7,0);
	pros::delay(500);
	RopoDevice::Chassis.MoveVelocity(0,0);
	RopoDevice::Chassis.AutoRotateAbs(320);
	delay();
	RopoDevice::Chassis.MoveVelocity(-5,0);
	pros::delay(500);
	RopoDevice::Chassis.MoveVelocity(0,0);
	pros::delay(300);
}