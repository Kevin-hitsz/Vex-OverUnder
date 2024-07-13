#include "main.h"
#include "RopoController.hpp"
#include "RopoDevice.hpp"
#include "RopoPosition.hpp"
#include "pros/misc.h"
#include "pros/rtos.hpp"
void autonomous_1();
namespace ControllerModule {

	bool timeFlag = 0;

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

	bool hittag = false;
	void HitBall(){
		hittag ^= 1;
		RopoDevice::ThreeWire::HitPneumatic.set_value(hittag);
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
	ButtonDetectLine.AddButtonDetect(pros::E_CONTROLLER_DIGITAL_R2   , RopoController::Rising,ControllerModule::PushExternSwitch);
	ButtonDetectLine.AddButtonDetect(pros::E_CONTROLLER_DIGITAL_L1   , RopoController::DoubleEdge, ControllerModule::SwitchIntakerForToBack);
	ButtonDetectLine.AddButtonDetect(pros::E_CONTROLLER_DIGITAL_L2   , RopoController::Rising, ControllerModule::SwitchIntakerFor);
	ButtonDetectLine.AddButtonDetect(pros::E_CONTROLLER_DIGITAL_LEFT , RopoController::Rising, ControllerModule::LeftExternSwitch);
	ButtonDetectLine.AddButtonDetect(pros::E_CONTROLLER_DIGITAL_RIGHT , RopoController::Rising, ControllerModule::RightExternSwitch);
	ButtonDetectLine.AddButtonDetect(pros::E_CONTROLLER_DIGITAL_X    , RopoController::Rising, ControllerModule::IntakerPusherSwitch);
	ButtonDetectLine.AddButtonDetect(pros::E_CONTROLLER_DIGITAL_B    , RopoController::Rising, ControllerModule::ChangeCatch);
	ButtonDetectLine.AddButtonDetect(pros::E_CONTROLLER_DIGITAL_A    , RopoController::Rising, RopoDevice::ChassisBrake);
	ButtonDetectLine.AddButtonDetect(pros::E_CONTROLLER_DIGITAL_DOWN , RopoController::Rising,  ControllerModule::HitBall);
	ButtonDetectLine.AddButtonDetect(pros::E_CONTROLLER_DIGITAL_UP   , RopoController::Rising,  ControllerModule::GpsUpdate);
	ButtonDetectLine.AddButtonDetect(pros::E_CONTROLLER_DIGITAL_Y    , RopoController::Rising,  autonomous_1);

	ButtonDetectLine.Enable();
	RopoDevice::ChassisCoast();
	ControllerModule::intaker_backward = false;
	ControllerModule::intaker_forward = true;
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

void autonomous_1(){
	RopoDevice::Chassis.StartChassisAutoControll();//底盘MoveType设置为AutoMove
	// --------- begin ------------

	/*----- Stage 1 抢场地中心四个球 ------*/
	ControllerModule::HitBall();		// 出杆
	RopoDevice::Chassis.AutoDirectMove(0.92,0,false);
	delay();
	ControllerModule::IntakerPusherSwitch();	// Intaker前伸，延长杆的范围
	pros::delay(300);
	RopoDevice::Chassis.AutoRotateAbs(-90);	// 旋转90度，用杆将两个球扫入己方半场
	delay();
	RopoDevice::gpsAddPosition.SetUpdateFlag(1);	// 开启GPS更新

	ControllerModule::HitBall();		// 收杆
	ControllerModule::IntakerPusherSwitch();	// Intaker收回（有必要吗）
	
	pros::delay(300);
	RopoDevice::Chassis.AutoDirectMove(0.92,0.5,false);	// 直行
	delay();
	RopoDevice::gpsAddPosition.SetUpdateFlag(0);	// 关闭GPS更新
	RopoDevice::Chassis.AutoRotateAbs(-60);
	ControllerModule::SwitchIntakerFor();
	delay();
	ControllerModule::IntakerPusherSwitch();	// Intaker前伸，延长杆的范围
	ControllerModule::HitBall();		// 出杆
	pros::delay(300);
	RopoDevice::Chassis.AutoRotateAbs(-90);	// 旋转90度，用杆将一个球扫入己方半场
	delay();
	ControllerModule::HitBall();		// 收杆
	ControllerModule::IntakerPusherSwitch();	// Intaker收回
	ControllerModule::WideExternSwitch();	 // 后两侧翅膀打开
	pros::delay(300);
	RopoDevice::Chassis.AutoRotateAbs(40);
	delay();
	RopoDevice::Chassis.MoveVelocity(-0.6,-0.6);
	pros::delay(500);
	if (fabs(RopoDevice::Position_Motor::MyPosition.Get_Angle()) > 3) {
		RopoDevice::Chassis.AutoRotateAbs(0);
		delay();
	}
	RopoDevice::Chassis.MoveVelocity(-1.0,0.0);
	pros::delay(400);
	RopoDevice::Chassis.MoveVelocity(0.0,0.0);
	pros::delay(100);

	/*----- Stage 2 回到导入区扫球 + 导三个球 ------*/
	RopoDevice::Chassis.MoveVelocity(0.6,0.0);
	pros::delay(400);
	RopoDevice::Chassis.MoveVelocity(0.0,0.0);
	ControllerModule::WideExternSwitch();	// 后两侧翅膀关闭
	pros::delay(200);
	RopoDevice::Chassis.AutoRotateAbs(-90);	// 旋转至-90度
	delay();
	RopoDevice::gpsAddPosition.SetUpdateFlag(1);	// 开启GPS更新
	RopoDevice::Chassis.MoveVelocity(-1.0,0.0);
	pros::delay(700);
	RopoDevice::Chassis.AutoRotateAbs(-135);	// 再向右旋转45度
	delay();
	RopoDevice::Chassis.MoveVelocity(0.8,0.0);
	pros::delay(300);
	ControllerModule::BothExternSwitch();	// 前两侧翅膀打开
	ControllerModule::SwitchIntakerForToBack();
	pros::delay(200);
	RopoDevice::Chassis.MoveVelocity(0.8,0.8);
	pros::delay(300);
	RopoDevice::Chassis.MoveVelocity(0,0);		
	ControllerModule::SwitchIntakerBack();			// 第一次扫球结束
	RopoDevice::gpsAddPosition.SetUpdateFlag(0);	// 关闭GPS更新


	// ControllerModule::BothExternSwitch();	// 前两侧翅膀关闭
	// pros::delay(100);
	// RopoDevice::Chassis.AutoDirectMove(0.2,-0.5,true);	// 直行
	// delay();
	// RopoDevice::Chassis.AutoRotateAbs(-225);	// 旋转至-225度
	// delay();
	// ControllerModule::HitBall();		// 出杆
	// pros::delay(100);
	// for (int i = 0; i < 2; i++)
	// {
	// 	RopoDevice::Chassis.AutoRotateAbs(-145);	// 旋转至-145度
	// 	delay();
	// 	RopoDevice::Chassis.AutoRotateAbs(-225);
	// 	delay();
	// }
	// RopoDevice::Chassis.AutoRotateAbs(-145);	// 旋转至-145度
	// delay();
	// ControllerModule::HitBall();		// 收杆
	// RopoDevice::Chassis.AutoRotateAbs(-125);	// 旋转至-125度
	// delay();
	// ControllerModule::BothExternSwitch();	// 前两侧翅膀打开

	// RopoDevice::Chassis.MoveVelocity(0.75,0.4);	// 过窄道
	// pros::delay(500);
	// ControllerModule::LeftExternSwitch();	// 左侧翅膀关闭
	// RopoDevice::Chassis.MoveVelocity(1.2,0);
	// pros::delay(1000);				// 窄道直行
	// RopoDevice::Chassis.MoveVelocity(0,0);
	// pros::delay(200);
	// RopoDevice::Chassis.AutoRotateAbs(-55);	// 旋转至-55度
	// delay();
	// ControllerModule::LeftExternSwitch();	// 左侧翅膀打开
	// RopoDevice::Chassis.MoveVelocity(0.8,0);
	// pros::delay(400);
	// RopoDevice::Chassis.MoveVelocity(0.8,0.6);
	// pros::delay(500);

	// /*----- Stage 3 推球入网 + 回到导球区 ------*/
	// RopoDevice::Chassis.AutoRotateAbs(-3);
	// pros::delay(400);
	// RopoDevice::Chassis.MoveVelocity(-0.6,0);
	// ControllerModule::LeftExternSwitch();	// 左侧翅膀关闭
	// pros::delay(300);
	// RopoDevice::Chassis.MoveVelocity(1.4,0);
	// pros::delay(600);
	// RopoDevice::Chassis.MoveVelocity(0,0);
	// ControllerModule::RightExternSwitch();	// 右侧翅膀关闭
	// pros::delay(200);
	// RopoDevice::Chassis.MoveVelocity(-0.6,0);
	// pros::delay(300);
	// RopoDevice::Chassis.AutoRotateAbs(-45); // 旋转至-45度
	// delay();
	// RopoDevice::Chassis.MoveVelocity(-1.0,0);
	// pros::delay(600);

	/*--------------------------------------------------------------------*/

	// //---推场地中间4个球
	// RopoDevice::gpsAddPosition.SetUpdateFlag(0);
	// RopoDevice::Chassis.AutoDirectMove(0.92,0,0);
	// delay();
	// ControllerModule::UnderExternSwitch();
	// // RopoDevice::gpsAddPosition.SetUpdateFlag(10);
	// RopoDevice::Chassis.AutoRotateAbs(90);
	// delay();
	// // 前移防止后杆因撞球放不下
	// // RopoDevice::ThreeWire::SpadePneumatic.set_value(true);
	// // RopoDevice::Chassis.MoveVelocity(0.2,0);
	// // pros::delay(230);
	// // RopoDevice::ThreeWire::WideExternPneumatic.set_value(true);
	// ControllerModule::WideExternSwitch();
	// RopoDevice::Chassis.MoveVelocity(-1.25,0);
	// pros::delay(500);
	// RopoDevice::Chassis.MoveVelocity(-0.5,0);
	// pros::delay(600);
	// RopoDevice::Chassis.MoveVelocity(0,0);
	// RopoDevice::Chassis.MoveVelocity(0.6,0);
	// pros::delay(500);
	// RopoDevice::Chassis.MoveVelocity(-0.5,0);
	// pros::delay(800);
	// RopoDevice::Chassis.MoveVelocity(0,0);
	// ControllerModule::WideExternSwitch();
	// pros::delay(2000);
	// ControllerModule::UnderExternSwitch();
	// // RopoDevice::Position_Motor::MyPosition.Set_XY(1.11, -0.70);
	// // 前移防止后杆收不回来
	// // RopoDevice::Chassis.MoveVelocity(0.4,0);
	// // pros::delay(280);
	// // RopoDevice::ThreeWire::WideExternPneumatic.set_value(false);

	// //---回到导入位置，对准角度，通过转动导入球
	// // 前翅膀导球
	// RopoDevice::Chassis.AutoPositionMove(0.05,0.33,-130.0);		// -123 degree
	// delay();
	// RopoDevice::Chassis.MoveVelocity(-0.2,-0.3);
	// pros::delay(2000);
	// RopoDevice::Chassis.MoveVelocity(0,0);
	// RopoDevice::Chassis.MoveVelocity(0.3,0.2);
	// pros::delay(1000);
	// RopoDevice::Chassis.MoveVelocity(0,0);
	// for(int i = 1; i<= 4; i++){
	// 	pros::delay(200);
	// 	ControllerModule::RightExternSwitch();
	// 	RopoDevice::Chassis.MoveVelocity(0.4,0.4);
	// 	pros::delay(900);
	// 	RopoDevice::Chassis.MoveVelocity(0,0);
	// 	if (4 == i){
	// 		RopoDevice::Chassis.AutoRotateAbs(-110);
	// 		delay();
	// 		ControllerModule::RightExternSwitch();
	// 		RopoDevice::Chassis.MoveVelocity(-0.4,-0.3);
	// 		pros::delay(700);						// 800
	// 		RopoDevice::Chassis.AutoRotateAbs(-135);
	// 		pros::delay(500);
	// 		RopoDevice::Chassis.MoveVelocity(0.2,0.0);
	// 		pros::delay(250);
	// 	}else{
	// 		ControllerModule::RightExternSwitch();
	// 		RopoDevice::Chassis.MoveVelocity(-0.4,-0.4);
	// 		pros::delay(700);						// 800
	// 		}
	// }
	// for (int i = 1; i <= 3; i++){
	// 	pros::delay(200);
	// 	ControllerModule::RightExternSwitch();
	// 	if (i == 1)
	// 	{
	// 		RopoDevice::Chassis.MoveVelocity(0.4,0.4);
	// 		pros::delay(700);			// 900
	// 	}
	// 	else
	// 	{
	// 		RopoDevice::Chassis.MoveVelocity(0.4,0.4);
	// 		pros::delay(900);
	// 	}
	// 	RopoDevice::Chassis.MoveVelocity(0,0);
	// 	ControllerModule::RightExternSwitch();
	// 	// RopoDevice::Chassis.AutoRotateAbs(-110);
	// 	// delay();
	// 	RopoDevice::Chassis.MoveVelocity(-0.4,-0.4);
	// 	pros::delay(800);
	// }
	
	// RopoDevice::Chassis.AutoRotateAbs(-135);
	// pros::delay(800);
	// ControllerModule::BothExternSwitch();
	// RopoDevice::Chassis.MoveVelocity(0.5, 0);	
	// // while (fabs(RopoDevice::Position_Motor::MyPosition.Get_Angle() + 135) < 2 ){
	// // 	pros::delay(10);
	// // }
	// pros::delay(450);
	// RopoDevice::Chassis.MoveVelocity(0, 0);
	// RopoDevice::Chassis.AutoRotateAbs(-100);
	// delay();
	// RopoDevice::Chassis.MoveVelocity(0.75,0);
	// pros::delay(350);
	// ControllerModule::LeftExternSwitch();
	// RopoDevice::Chassis.MoveVelocity(0.75,0.4);
	// RopoDevice::gpsAddPosition.SetUpdateFlag(1);
	// pros::delay(500);
	// RopoDevice::Chassis.MoveVelocity(1.2,0);
	// pros::delay(1000);							// 窄道直行
	// RopoDevice::Chassis.MoveVelocity(0,0);
	

	

	// //---直线加曲线推球入网

	// // RopoDevice::Chassis.MoveVelocity(0.75,0);
	// // while (RopoDevice::Position_Motor::MyPosition.Get_Y() > -1.45) {
	// // pros::delay(15);
	// // }
	// //RopoDevice::Chassis.MoveVelocity(0,0);
	// ControllerModule::LeftExternSwitch();
	// if (fabs(RopoDevice::Position_Motor::MyPosition.Get_Angle() + 90) > 3) {
	// 	RopoDevice::Chassis.AutoRotateAbs(-90);
	// 	delay();
	// }
	// RopoDevice::Chassis.MoveVelocity(0.75,0.95);		// 0.75, 0.95
	// pros::delay(700);
	// // ControllerModule::RightExternSwitch();
	// pros::delay(950);
	// ControllerModule::LeftExternSwitch();
	// pros::delay(500);
	// for (int i = 0; i < 1; i++)
	// {	
	// 	RopoDevice::Chassis.AutoRotateAbs(0);
	// 	pros::delay(400);
	// 	RopoDevice::Chassis.MoveVelocity(-0.6,0);
	// 	pros::delay(300);
	// 	RopoDevice::Chassis.MoveVelocity(1.4,0);
	// 	pros::delay(600);
	// }
	// RopoDevice::Chassis.MoveVelocity(0,0);
	// RopoDevice::Chassis.MoveVelocity(-0.6,0);
	// pros::delay(300);
	// ControllerModule::RightExternSwitch();
	// RopoDevice::Chassis.AutoRotateAbs(160);
	// delay();
	// RopoDevice::Chassis.MoveVelocity(-1.4,0);
	// pros::delay(500);
	// RopoDevice::Chassis.MoveVelocity(0,0);
	// RopoDevice::Chassis.MoveVelocity(1.0,0);
	// pros::delay(500);
	
	// ControllerModule::SwitchIntakerFor();
	// RopoDevice::Chassis.AutoRotateAbs(145);
	// delay();
	// RopoDevice::Chassis.MoveVelocity(1.0,0);
	// pros::delay(1000);
	// RopoDevice::Chassis.AutoRotateAbs(92);
	// delay();
	// ControllerModule::RightExternSwitch();
	// RopoDevice::ChassisHold();
	// pros::delay(100);
	// RopoDevice::Chassis.MoveVelocity(1.0,0);
	// pros::delay(900);

	RopoDevice::Chassis.MoveVelocity(0,0);
}
