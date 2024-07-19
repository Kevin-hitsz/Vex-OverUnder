#include "main.h"
#include "RopoController.hpp"
#include "RopoDevice.hpp"
#include "RopoPosition.hpp"
#include "pros/misc.h"
#include "pros/rtos.hpp"
void autonomous_1();
void autonomous_2();
namespace ControllerModule {

	bool timeFlag = 0;
	bool bulletTime = 0;

	void BoolSwitch(void * Parameter){
		bool *p = static_cast<bool *>(Parameter);
		(*p) ^= 1;
	}

	bool wideExternFlag = false;
	void WideExternSwitch(){
		wideExternFlag ^= 1; 
		RopoDevice::ThreeWire::WideExternPneumatic.set_value(wideExternFlag);
	}

	bool leftExternFlag = false;
	void LeftExternSwitch(){
		leftExternFlag ^= 1; 
		RopoDevice::ThreeWire::LeftExternPneumatic.set_value(leftExternFlag);
	}

	bool rightExternFlag = false;
	void RightExternSwitch(){
		rightExternFlag ^= 1; 
		RopoDevice::ThreeWire::RightExternPneumatic.set_value(rightExternFlag);
	}

	void BothExternSwitch(){
		leftExternFlag ^= 1;
		rightExternFlag ^= 1;
		RopoDevice::ThreeWire::LeftExternPneumatic.set_value(leftExternFlag);
		RopoDevice::ThreeWire::RightExternPneumatic.set_value(rightExternFlag);
	}

	bool underExternFlag = false;
	void UnderExternSwitch(){
		underExternFlag ^= 1; 
		if (timeFlag == 1){
			underExternFlag = false;
		}
		RopoDevice::ThreeWire::UnderExternPneumatic.set_value(underExternFlag);
	}

	void PushExternSwitch(){
		underExternFlag ^= 1;
		wideExternFlag ^= 1;
		RopoDevice::ThreeWire::UnderExternPneumatic.set_value(underExternFlag);
		RopoDevice::ThreeWire::WideExternPneumatic.set_value(wideExternFlag);
	}

	bool locktag = false;
	void ChangeCatch(){
		locktag ^= 1;
		RopoDevice::ThreeWire::CatchPneumatic.set_value(locktag);
		RopoDevice::ChassisHold();
	}

	bool hit_tag = false;
	void HitBall(){
		hit_tag ^= 1;
		RopoDevice::ThreeWire::HitPneumatic.set_value(hit_tag);
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
	int catch_1 = 0;
	void Hold(){
		RopoDevice::LiftMotors.Hold();
		catch_1 = 1;
	}

	void Touch(){
		RopoDevice::LiftMotors.Touch();
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

	bool intaker_forward = false;
	bool intaker_backward = false;
	void RollIntaker(){
		if (intaker_forward && !intaker_backward) {
			RopoDevice::Motors::IntakerMoveVoltage(150);
		}
		else if (!intaker_forward && intaker_backward) {
			RopoDevice::Motors::IntakerMoveVoltage(-150);
		}
		else {
			RopoDevice::Motors::IntakerMoveVoltage(0);
		}
	}

	void SwitchIntakerFor(){
		intaker_forward ^= 1;
		intaker_backward = false;
		RollIntaker();
	}

	void SwitchIntakerBack(){
		intaker_backward ^= 1;
		intaker_forward = false;
		RollIntaker();
	}

	void SwitchIntakerForToBack(){
		intaker_forward ^= 1;
		intaker_backward ^= 1;
		pros::delay(50);
		RollIntaker();
	}

	bool pusher_tag = false;
	void IntakerPusherSwitch(){
		pusher_tag ^= 1;
		RopoDevice::ThreeWire::PusherPneumatic.set_value(pusher_tag);
	}

	void TurnAround(){
		RopoDevice::Chassis.AutoRotateRelative(180);
	}

	void GpsUpdate(){
		RopoDevice::gpsAddPosition.GpsUpdate();
	}

	void AutoLift(){
		RopoDevice::Chassis.MoveVelocity(-0.6,0);
		pros::delay(400);
		RopoDevice::Chassis.MoveVelocity(-0.3,0);
		pros::delay(500);
		RopoDevice::Chassis.MoveVelocity(0,0);
		ChangeCatch();
		RopoDevice::Chassis.MoveVelocity(-0.6,0);
		pros::delay(1000);
		RopoDevice::Chassis.MoveVelocity(-1.2,0);
		pros::delay(1000);
		RopoDevice::Chassis.MoveVelocity(-2.0,0);
		pros::delay(1000);
		RopoDevice::Chassis.MoveVelocity(-1.3,0);
		pros::delay(1000);
		RopoDevice::Chassis.MoveVelocity(-0.6,0);
		pros::delay(1000);
		RopoDevice::Chassis.MoveVelocity(-0.3,0);
		pros::delay(500);
		RopoDevice::Chassis.MoveVelocity(0,0);

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
			// MasterController.print(2,1,"%.2lf  %d",RopoDevice::LiftMotors.GetLifterPosition(), RopoDevice::LiftMotors.GetLifterStatus());
			// pros::delay(50);
		}
	}

	void BulletTimeStart(){
		bulletTime ^= 1;
		if (bulletTime == 1) {
			RopoDevice::ChassisHold();
		}
		else {
			RopoDevice::ChassisCoast();
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
	autonomous_1();
}

void opcontrol()
{
	//pros::Task *RumbleTask = new pros::Task(ControllerModule::RumbleMe);
	pros::Task *PrintTask = new pros::Task(ControllerModule::ControllerPrint);
	pros::Controller MasterController(pros::E_CONTROLLER_MASTER);
	RopoController::ButtonTaskLine ButtonDetectLine(MasterController);
	FloatType VelocityMax = 1.7;	// 1.7 m/s
	FloatType WcMax = 8;	// 7 
	FloatType VelocityRestrainRatio = 0.4; // 0 ~ 1
	FloatType WcRestrainRatio = 0.45; // 0 ~ 1
	bool ChassisMove = false;
	const FloatType nowTime = pros::millis();
	RopoDevice::ChassisCoast();
	
	RopoController::AxisValueCast XVelocityInput(MasterController,pros::E_CONTROLLER_ANALOG_LEFT_Y,RopoController::Linear);
	RopoController::AxisValueCast WVelocityInput(MasterController,pros::E_CONTROLLER_ANALOG_RIGHT_X,RopoController::Exp);

	Vector Velocity(RopoMath::ColumnVector,2),ResVelocity;

	MasterController.clear();
	ButtonDetectLine.AddButtonDetect(pros::E_CONTROLLER_DIGITAL_R1   , RopoController::Rising, ControllerModule::BothExternSwitch);
	ButtonDetectLine.AddButtonDetect(pros::E_CONTROLLER_DIGITAL_R2   , RopoController::Rising,ControllerModule::UnderExternSwitch);
	ButtonDetectLine.AddButtonDetect(pros::E_CONTROLLER_DIGITAL_L1   , RopoController::DoubleEdge, ControllerModule::SwitchIntakerForToBack);
	ButtonDetectLine.AddButtonDetect(pros::E_CONTROLLER_DIGITAL_L2   , RopoController::DoubleEdge, ControllerModule::IntakerPusherSwitch);
	ButtonDetectLine.AddButtonDetect(pros::E_CONTROLLER_DIGITAL_LEFT , RopoController::Rising, ControllerModule::LeftExternSwitch);
	ButtonDetectLine.AddButtonDetect(pros::E_CONTROLLER_DIGITAL_RIGHT , RopoController::Rising, ControllerModule::RightExternSwitch);
	ButtonDetectLine.AddButtonDetect(pros::E_CONTROLLER_DIGITAL_X    , RopoController::Rising, ControllerModule::SwitchIntakerFor);
	ButtonDetectLine.AddButtonDetect(pros::E_CONTROLLER_DIGITAL_B    , RopoController::Rising, ControllerModule::WideExternSwitch);
	ButtonDetectLine.AddButtonDetect(pros::E_CONTROLLER_DIGITAL_A    , RopoController::Rising, RopoDevice::ChassisBrake);
	ButtonDetectLine.AddButtonDetect(pros::E_CONTROLLER_DIGITAL_DOWN , RopoController::Rising,  ControllerModule::HitBall);
	ButtonDetectLine.AddButtonDetect(pros::E_CONTROLLER_DIGITAL_UP   , RopoController::Rising,  ControllerModule::GpsUpdate);
	ButtonDetectLine.AddButtonDetect(pros::E_CONTROLLER_DIGITAL_Y    , RopoController::Rising,  autonomous_2);

	ButtonDetectLine.Enable();
	RopoDevice::ChassisCoast();
	ControllerModule::intaker_backward = false;
	ControllerModule::intaker_forward = true;
	ControllerModule::leftExternFlag = false;
	ControllerModule::rightExternFlag = false;
	ControllerModule::wideExternFlag = false;
	ControllerModule::underExternFlag = false;
	ControllerModule::pusher_tag = false;
	ControllerModule::hit_tag = false;

	ControllerModule::RollIntaker();

	while (true) {

		FloatType XInput =  XVelocityInput.GetAxisValue();
		FloatType WInput = -WVelocityInput.GetAxisValue();
		FloatType RopoWc = WcMax * (1.0 - fabs(XInput) * WcRestrainRatio);
		FloatType RopoVx = VelocityMax * (1.0 - fabs(WInput) * VelocityRestrainRatio);

		if (fabs(XInput) <= 0.06 && fabs(WInput) <= 0.03) {
			if(ChassisMove == true){
				RopoDevice::Motors::MoveOpControll(0.0, 0.0);
				ChassisMove = false;
			}
		} else if (ControllerModule::bulletTime == 1) {
			RopoDevice::Chassis.StartChassisOpControll(); // 底盘MoveType设置为OpMove
			RopoDevice::Motors::MoveOpControll(XInput * 0.6, WInput * RopoWc);
			ChassisMove = true;
		}
		else {
			RopoDevice::Chassis.StartChassisOpControll(); // 底盘MoveType设置为OpMove
			RopoDevice::Motors::MoveOpControll(XInput * RopoVx, WInput * RopoWc);
			ChassisMove = true;
		}
		pros::delay(4);
	}
}

void delay(){ //代替pros::delay()进行时间控制
	while(!RopoDevice::Chassis.IfArrived()){
		pros::delay(1);
	}
	return;
}

void autonomous_1(){	//淘汰赛 上限策略
	RopoDevice::Chassis.StartChassisAutoControll();//底盘MoveType设置为AutoMove
	RopoDevice::ChassisBrake();
	// --------- begin ------------

	/*----- Stage 1 抢场地中心四个球 ------*/
	ControllerModule::HitBall();		// 出杆
	RopoDevice::Chassis.AutoDirectMove(0.95,0,false);
	delay();
	ControllerModule::IntakerPusherSwitch();	// Intaker前伸，延长杆的范围
	
	pros::delay(500);
	// RopoDevice::Chassis.MoveVelocity(0, -1.5);
	// pros::delay(700);						// 先转一点，防止太快
	RopoDevice::gpsAddPosition.SetUpdateFlag(1);	// 开启GPS更新
	RopoDevice::Chassis.AutoRotateAbs(-120);	// 旋转110度，用杆将两个球扫入己方半场
	delay();
	ControllerModule::HitBall();		// 收杆
	ControllerModule::IntakerPusherSwitch();	// Intaker收回（有必要吗）
	pros::delay(300);

	RopoDevice::Chassis.AutoRotateAbs(-90);
	delay();

	RopoDevice::Chassis.AutoDirectMove(1.29,-0.24,false);
	delay();
	RopoDevice::Chassis.AutoRotateAbs(-40);
	delay();

	RopoDevice::Chassis.AutoDirectMove(1.32,-0.34,false);		// 1.32 -0.34
	pros::delay(1000);
	// // RopoDevice::Chassis.AutoPositionMoveWithTimeLimit(1.31, -0.37, 3000);	// 找点 1.33 0.25
	RopoDevice::Chassis.MoveVelocity(0.0, 0.0);
	RopoDevice::ChassisHold();
	ControllerModule::SwitchIntakerFor();	// 开启Intaker
	ControllerModule::IntakerPusherSwitch();	// Intaker前伸，延长杆的范围
	pros::delay(600);
	ControllerModule::HitBall();		// 出杆
	pros::delay(400);
	RopoDevice::Chassis.AutoRotateAbs(-80);	// 旋转至-70度，用杆将一个球扫入己方半场
	delay();
	pros::delay(200);
	RopoDevice::ChassisBrake();
	ControllerModule::HitBall();		// 收杆
	ControllerModule::IntakerPusherSwitch();	// Intaker收回
	pros::delay(300);
	RopoDevice::Chassis.MoveVelocity(-0.4, 0.0);	// 后退
	pros::delay(350);
	// //RopoDevice::Chassis.AutoPositionMoveWithTimeLimit(1.13, -0.11, 1500);	// 找点
	RopoDevice::Chassis.AutoRotateAbs(45);	// 旋转至45度
	delay();
	RopoDevice::gpsAddPosition.SetUpdateFlag(0);	//关闭GPS更新
	
	ControllerModule::PushExternSwitch();	// 两侧翅膀打开
	// RopoDevice::Chassis.AutoPositionMove(1.29, -0.24);	// 找点
	// delay();
	RopoDevice::Chassis.MoveVelocity(-0.6,-1.0);
	pros::delay(500);
	RopoDevice::Chassis.AutoRotateAbs(0);
	delay();
	RopoDevice::Chassis.MoveVelocity(-1.0,0.0);
	pros::delay(400);
	RopoDevice::Chassis.MoveVelocity(-0.4, 0.0);
	pros::delay(800);
	

	// /*----- Stage 2 回到导入区扫球 + 导三个球 ------*/
	ControllerModule::PushExternSwitch();	 // 翅膀关闭
	RopoDevice::Chassis.MoveVelocity(0.8, 0.0);
	pros::delay(250);
	RopoDevice::Chassis.MoveVelocity(-0.8, 0.0);
	pros::delay(350);
	// RopoDevice::Chassis.MoveVelocity(-0.6,0.0);
	// pros::delay(100);
	RopoDevice::Chassis.AutoRotateAbs(-87.0);	// 旋转至-90度
	delay();
	RopoDevice::Chassis.MoveVelocity(-1.0,0.0);
	RopoDevice::gpsAddPosition.SetUpdateFlag(1);	// 开启GPS更新
	pros::delay(1400);
	RopoDevice::Chassis.MoveVelocity(0.0,0.0);
	pros::delay(300);
	
	RopoDevice::Chassis.AutoRotateAbs(-135);	// 再向右旋转45度
	delay();
	ControllerModule::BothExternSwitch();	// 两侧翅膀打开
	RopoDevice::Chassis.MoveVelocity(0.8,0.0);
	pros::delay(200);
	ControllerModule::SwitchIntakerForToBack();
	pros::delay(400);
	RopoDevice::Chassis.MoveVelocity(0.8,2.4);
	pros::delay(400);
	RopoDevice::Chassis.MoveVelocity(1.0,0.0);
	pros::delay(700);
	RopoDevice::Chassis.MoveVelocity(0,0);		// 第一次扫球结束

	ControllerModule::SwitchIntakerBack();
	ControllerModule::BothExternSwitch();	// 两侧翅膀关闭
	pros::delay(100);
	// RopoDevice::Chassis.MoveVelocity(-1.0,-1.0);
	// pros::delay(400);
	RopoDevice::Chassis.AutoPositionMoveBack(0.30, 0.70, 115);	// 找点导入 0.65
	delay();
	RopoDevice::Chassis.MoveVelocity(1.0,0.0);
	pros::delay(300);
	RopoDevice::Chassis.MoveVelocity(0.0,0.0);
	ControllerModule::IntakerPusherSwitch();
	// //ControllerModule::HitBall();		// 出杆
	RopoDevice::ChassisHold();
	RopoDevice::gpsAddPosition.SetUpdateFlag(0);	// 关闭GPS更新
	pros::delay(800);
	for (int i = 0; i < 6; i++)
	{
		RopoDevice::Chassis.AutoRotateAbs(170);	// 旋转至170度
		pros::delay(700);
		// RopoDevice::Chassis.MoveVelocity(0.0,6.0);
		// pros::delay(350);
		RopoDevice::Chassis.AutoRotateAbs(115);
		pros::delay(700);
	}
	RopoDevice::Chassis.AutoRotateAbs(170);	// 旋转至170度
	delay();
	ControllerModule::BothExternSwitch();	// 前两侧翅膀打开
	RopoDevice::Chassis.AutoRotateAbs(-130);	// 旋转至-155度
	delay();
	// //ControllerModule::HitBall();		// 收杆
	ControllerModule::IntakerPusherSwitch();
	RopoDevice::ChassisBrake();
	RopoDevice::gpsAddPosition.SetUpdateFlag(1);

	RopoDevice::Chassis.MoveVelocity(0.8,1.0);	// 过窄道
	pros::delay(800);
	RopoDevice::Chassis.AutoRotateAbs(-90);
	delay();
	RopoDevice::Chassis.MoveVelocity(1.2,0.0);
	pros::delay(400);
	ControllerModule::LeftExternSwitch();	// 左侧翅膀关闭
	pros::delay(400);				// 窄道直行
	ControllerModule::LeftExternSwitch();	// 左侧翅膀开启
	RopoDevice::Chassis.AutoDirectMove(-0.29,-1.57,false);
	delay();
	ControllerModule::RightExternSwitch();	// 右侧翅膀关闭

	/*----- Stage 3 推球入网 + 回到导球区 ------*/
	
	RopoDevice::Chassis.MoveVelocity(0.6,0.0);
	pros::delay(300);
	RopoDevice::Chassis.AutoRotateAbs(-60);	// 旋转至-60度
	delay();
	RopoDevice::Chassis.MoveVelocity(1.0,0.0);
	pros::delay(400);
	RopoDevice::Chassis.MoveVelocity(1.0,4.0);
	pros::delay(400);
	RopoDevice::Chassis.AutoRotateAbs(-15);	// 旋转至-15度
	delay();
	RopoDevice::Chassis.MoveVelocity(1.0,0.0);
	pros::delay(800);
	ControllerModule::LeftExternSwitch();	// 左侧翅膀关闭
	RopoDevice::Chassis.MoveVelocity(-0.6,0.0);
	pros::delay(800);
	RopoDevice::Chassis.AutoRotateAbs(-10);	// 旋转至-10度
	delay();
	ControllerModule::LeftExternSwitch();	// 左侧翅膀开启
	RopoDevice::Chassis.MoveVelocity(1.0,0.0);
	pros::delay(800);
	ControllerModule::LeftExternSwitch();	// 左侧翅膀关闭
	RopoDevice::Chassis.MoveVelocity(-0.6,0.0);
	pros::delay(800);
	RopoDevice::Chassis.AutoRotateAbs(90);	// 旋转至90度
	delay();
	RopoDevice::Chassis.MoveVelocity(1.0,-3.0);
	pros::delay(1000);
	RopoDevice::Chassis.MoveVelocity(0,0);
}

void autonomous_2(){	// 资格赛 保守策略
	RopoDevice::Chassis.StartChassisAutoControll();//底盘MoveType设置为AutoMove
	RopoDevice::ChassisBrake();
	// --------- begin ------------

	/*----- Stage 1 抢场地中心四个球 ------*/
	ControllerModule::HitBall();		// 出杆
	RopoDevice::Chassis.AutoDirectMove(1.05,0,false);
	delay();
	ControllerModule::IntakerPusherSwitch();	// Intaker前伸，延长杆的范围
	
	pros::delay(500);
	// RopoDevice::Chassis.MoveVelocity(0, -1.5);
	// pros::delay(700);						// 先转一点，防止太快
	//RopoDevice::gpsAddPosition.SetUpdateFlag(1);	// 开启GPS更新
	RopoDevice::Chassis.AutoRotateAbs(-90);	// 旋转110度，用杆将两个球扫入己方半场
	delay();
	ControllerModule::HitBall();		// 收杆
	//ControllerModule::IntakerPusherSwitch();	// Intaker收回（有必要吗）
	pros::delay(300);
	ControllerModule::SwitchIntakerFor();	// 开启Intaker
	// RopoDevice::Chassis.AutoRotateAbs(-90);
	// delay();

	RopoDevice::Chassis.AutoDirectMove(1.08,-0.44,false);
	delay();
	// RopoDevice::Chassis.AutoRotateAbs(-40);
	// delay();

	// RopoDevice::Chassis.AutoDirectMove(1.32,-0.34,false);		// 1.32 -0.34
	// pros::delay(1000);
	// // // RopoDevice::Chassis.AutoPositionMoveWithTimeLimit(1.31, -0.37, 3000);	// 找点 1.33 0.25
	// RopoDevice::Chassis.MoveVelocity(0.0, 0.0);
	// RopoDevice::ChassisHold();

	// ControllerModule::IntakerPusherSwitch();	// Intaker前伸，延长杆的范围
	// pros::delay(600);
	// ControllerModule::HitBall();		// 出杆
	// pros::delay(400);
	// RopoDevice::Chassis.AutoRotateAbs(-80);	// 旋转至-70度，用杆将一个球扫入己方半场
	// delay();
	pros::delay(300);
	// RopoDevice::ChassisBrake();
	// ControllerModule::HitBall();		// 收杆
	ControllerModule::IntakerPusherSwitch();	// Intaker收回
	pros::delay(300);
	// RopoDevice::Chassis.MoveVelocity(-0.8, 0.0);	// 后退
	// pros::delay(150);
	// //RopoDevice::Chassis.AutoPositionMoveWithTimeLimit(1.13, -0.11, 1500);	// 找点
	// RopoDevice::Chassis.AutoRotateAbs(45);	// 旋转至45度
	// delay();
	//RopoDevice::gpsAddPosition.SetUpdateFlag(0);	//关闭GPS更新
	

	// RopoDevice::Chassis.AutoPositionMove(1.29, -0.24);	// 找点
	// delay();
	// RopoDevice::Chassis.MoveVelocity(-0.6,-1.0);
	// pros::delay(500);
	RopoDevice::Chassis.AutoRotateAbs(5);
	delay();
	ControllerModule::PushExternSwitch();	// 两侧翅膀打开
	RopoDevice::Chassis.MoveVelocity(-1.2,0.0);
	pros::delay(500);
	RopoDevice::Chassis.MoveVelocity(-0.5, 0.0);
	pros::delay(600);
	

	// /*----- Stage 2 回到导入区扫球 + 导三个球 ------*/
	ControllerModule::PushExternSwitch();	 // 翅膀关闭
	RopoDevice::Chassis.MoveVelocity(0.8, 0.0);
	pros::delay(250);
	RopoDevice::Chassis.MoveVelocity(-0.8, 0.0);
	pros::delay(300);
	// RopoDevice::Chassis.MoveVelocity(-0.6,0.0);
	// pros::delay(100);
	RopoDevice::Chassis.AutoRotateAbs(-87.0);	// 旋转至-90度
	delay();
	RopoDevice::Chassis.MoveVelocity(-1.7,0.0);
	RopoDevice::gpsAddPosition.SetUpdateFlag(1);	// 开启GPS更新
	pros::delay(900);
	RopoDevice::Chassis.MoveVelocity(0.0,0.0);
	pros::delay(200);
	
	RopoDevice::Chassis.AutoRotateAbs(-135);	// 再向右旋转45度
	delay();
	ControllerModule::BothExternSwitch();	// 两侧翅膀打开
	RopoDevice::Chassis.MoveVelocity(0.8,0.0);
	pros::delay(200);
	ControllerModule::SwitchIntakerForToBack();
	pros::delay(400);
	RopoDevice::Chassis.MoveVelocity(0.8,2.4);
	pros::delay(400);
	RopoDevice::Chassis.MoveVelocity(1.0,0.0);
	pros::delay(700);
	RopoDevice::Chassis.MoveVelocity(0,0);		// 第一次扫球结束

	ControllerModule::SwitchIntakerBack();
	pros::delay(100);
	// RopoDevice::Chassis.MoveVelocity(-1.0,-1.0);
	// pros::delay(400);
	RopoDevice::Chassis.MoveVelocity(-1.0,0.0);
	pros::delay(800);
	ControllerModule::BothExternSwitch();	// 两侧翅膀关闭
	RopoDevice::Chassis.AutoPositionMoveBack(0.30, 0.70, 120);	// 找点导入 0.65
	delay();
	RopoDevice::Chassis.MoveVelocity(1.0,0.0);
	pros::delay(300);
	RopoDevice::Chassis.MoveVelocity(0.0,0.0);
	ControllerModule::IntakerPusherSwitch();
	// //ControllerModule::HitBall();		// 出杆
	RopoDevice::ChassisHold();
	RopoDevice::gpsAddPosition.SetUpdateFlag(0);	// 关闭GPS更新
	pros::delay(800);
	for (int i = 0; i < 6; i++)
	{
		RopoDevice::Chassis.AutoRotateAbs(170);	// 旋转至170度
		// delay();
		// RopoDevice::Chassis.MoveVelocity(0.0,6.0);
		pros::delay(700);
		RopoDevice::Chassis.AutoRotateAbs(115);
		pros::delay(700);
	}
	RopoDevice::Chassis.AutoRotateAbs(170);	// 旋转至170度
	delay();
	ControllerModule::BothExternSwitch();	// 前两侧翅膀打开
	RopoDevice::Chassis.AutoRotateAbs(-130);	// 旋转至-155度
	delay();
	// //ControllerModule::HitBall();		// 收杆
	ControllerModule::IntakerPusherSwitch();
	RopoDevice::ChassisBrake();
	RopoDevice::gpsAddPosition.SetUpdateFlag(1);

	RopoDevice::Chassis.MoveVelocity(0.8,1.0);	// 过窄道
	pros::delay(800);
	RopoDevice::Chassis.AutoRotateAbs(-92);
	delay();
	RopoDevice::Chassis.MoveVelocity(1.2,0.0);
	pros::delay(400);
	ControllerModule::LeftExternSwitch();	// 左侧翅膀关闭
	pros::delay(400);				// 窄道直行
	ControllerModule::LeftExternSwitch();	// 左侧翅膀开启
	RopoDevice::Chassis.AutoDirectMove(-0.29,-1.57,false);
	delay();


	/*----- Stage 3 推球入网 + 回到导球区 ------*/
	
	RopoDevice::Chassis.MoveVelocity(0.6,0.0);
	pros::delay(300);
	ControllerModule::RightExternSwitch();	// 右侧翅膀关闭
	RopoDevice::Chassis.AutoRotateAbs(-60);	// 旋转至-60度
	delay();
	RopoDevice::Chassis.MoveVelocity(1.0,0.0);
	pros::delay(400);
	RopoDevice::Chassis.MoveVelocity(1.0,4.0);
	pros::delay(400);
	RopoDevice::Chassis.AutoRotateAbs(-15);	// 旋转至-15度
	delay();
	RopoDevice::Chassis.MoveVelocity(1.0,0.0);
	pros::delay(800);
	ControllerModule::LeftExternSwitch();	// 左侧翅膀关闭
	RopoDevice::Chassis.MoveVelocity(-0.6,0.0);
	pros::delay(800);
	RopoDevice::Chassis.AutoRotateAbs(-10);	// 旋转至-10度
	delay();
	ControllerModule::LeftExternSwitch();	// 左侧翅膀开启
	RopoDevice::Chassis.MoveVelocity(1.0,0.0);
	pros::delay(800);
	ControllerModule::LeftExternSwitch();	// 左侧翅膀关闭
	RopoDevice::Chassis.MoveVelocity(-0.6,0.0);
	pros::delay(800);
	RopoDevice::Chassis.AutoRotateAbs(-45);	// 旋转至-45度
	delay();
	// RopoDevice::Chassis.AutoDirectMove(0.32,-2.00,true);
	// delay();
	RopoDevice::Chassis.MoveVelocity(-1.2,0.0);
	pros::delay(900);
	RopoDevice::Chassis.AutoRotateAbs(-95);	// 旋转至-95度
	delay();
	RopoDevice::Chassis.MoveVelocity(-1.2,0.0);
	pros::delay(400);
	ControllerModule::BothExternSwitch();	// 双侧翅膀开启
	RopoDevice::Chassis.MoveVelocity(-0.8,0.0);
	pros::delay(800);
	RopoDevice::Chassis.MoveVelocity(0,0);
}