#include "main.h"
#include "RopoController.hpp"
#include "RopoDevice.hpp"
#include "RopoPosition.hpp"
#include "pros/misc.h"
#include "pros/rtos.hpp"
void autonomous_2();
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

	void SwitchIntakerFor(){
		RopoDevice::intaker.SwitchIntakerFor();
	}

	void SwitchIntakerBack(){
		RopoDevice::intaker.SwitchIntakerBack();
	}

	void SwitchIntakerStop(){
		RopoDevice::intaker.SwitchIntakerStop();
	}

	void SwitchIntakerSwitch(){
		RopoDevice::intaker.SwitchIntakerSwitch();
	}

	void SwitchIntakerChange(){
		RopoDevice::intaker.SwitchIntakerChange();
	}

	void SwitchIntakerGrabFor(){
		RopoDevice::intaker.SwitchIntakerGrabFor();
	}

	void SwitchIntakerGrabBack(){
		RopoDevice::intaker.SwitchIntakerGrabBack();
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
			MasterController.print(1,1,"GPS:X: %.2lfY:%.2lf",RopoDevice::gpsAddPosition.GetGpsTransformRelativePositionX(),RopoDevice::gpsAddPosition.GetGpsTransformRelativePositionY());
			pros::delay(50); 
			MasterController.print(2,1,"ECR:X: %.2lfY:%.2lf",RopoDevice::GetPosition()[1],RopoDevice::GetPosition()[2]);
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
	autonomous_2();
}

void opcontrol()
{
	//pros::Task *RumbleTask = new pros::Task(ControllerModule::RumbleMe);
	pros::Task *PrintTask = new pros::Task(ControllerModule::ControllerPrint);
	pros::Controller MasterController(pros::E_CONTROLLER_MASTER);
	RopoController::ButtonTaskLine ButtonDetectLine(MasterController);
	FloatType VelocityMax = 1.60;	// 1.7 m/s
	FloatType WcMax = 20;	// 7 
	FloatType VelocityRestrainRatio = 0.4; // 0 ~ 1
	FloatType WcRestrainRatio = 0.4; // 0 ~ 1
	bool ChassisMove = false;
	const FloatType nowTime = pros::millis();
	RopoDevice::ChassisCoast();
	
	RopoController::AxisValueCast XVelocityInput(MasterController,pros::E_CONTROLLER_ANALOG_LEFT_Y,RopoController::Linear);
	RopoController::AxisValueCast WVelocityInput(MasterController,pros::E_CONTROLLER_ANALOG_RIGHT_X,RopoController::Exp);

	Vector Velocity(RopoMath::ColumnVector,2),ResVelocity;

	MasterController.clear();
	ButtonDetectLine.AddButtonDetect(pros::E_CONTROLLER_DIGITAL_R1   , RopoController::Rising, ControllerModule::BothExternSwitch);
	ButtonDetectLine.AddButtonDetect(pros::E_CONTROLLER_DIGITAL_R2   , RopoController::Rising,ControllerModule::PushExternSwitch);
	// ButtonDetectLine.AddButtonDetect(pros::E_CONTROLLER_DIGITAL_L1   , RopoController::DoubleClick, ControllerModule::SwitchIntakerStop);
	ButtonDetectLine.AddButtonDetect(pros::E_CONTROLLER_DIGITAL_L1   , RopoController::Rising, ControllerModule::SwitchIntakerBack);
	ButtonDetectLine.AddButtonDetect(pros::E_CONTROLLER_DIGITAL_L1   , RopoController::Falling, ControllerModule::SwitchIntakerFor);
	ButtonDetectLine.AddButtonDetect(pros::E_CONTROLLER_DIGITAL_L2   , RopoController::Rising, ControllerModule::SwitchIntakerGrabFor);
	ButtonDetectLine.AddButtonDetect(pros::E_CONTROLLER_DIGITAL_L2   , RopoController::Falling, ControllerModule::SwitchIntakerGrabBack);

	ButtonDetectLine.AddButtonDetect(pros::E_CONTROLLER_DIGITAL_LEFT , RopoController::Rising, ControllerModule::LeftExternSwitch);
	ButtonDetectLine.AddButtonDetect(pros::E_CONTROLLER_DIGITAL_RIGHT , RopoController::Rising, ControllerModule::RightExternSwitch);
	//ButtonDetectLine.AddButtonDetect(pros::E_CONTROLLER_DIGITAL_X    , RopoController::Rising, ControllerModule::UnderExternSwitch);
	ButtonDetectLine.AddButtonDetect(pros::E_CONTROLLER_DIGITAL_B    , RopoController::Rising, ControllerModule::ChangeCatch);
	ButtonDetectLine.AddButtonDetect(pros::E_CONTROLLER_DIGITAL_A    , RopoController::Rising, RopoDevice::ChassisBrake);
	//ButtonDetectLine.AddButtonDetect(pros::E_CONTROLLER_DIGITAL_DOWN , RopoController::Rising,  ControllerModule::WideExternSwitch);
	ButtonDetectLine.AddButtonDetect(pros::E_CONTROLLER_DIGITAL_UP   , RopoController::Rising,  ControllerModule::GpsUpdate);
	//ButtonDetectLine.AddButtonDetect(pros::E_CONTROLLER_DIGITAL_Y    , RopoController::Rising,  autonomous_2);

	ButtonDetectLine.Enable();
	RopoDevice::ChassisCoast();
	RopoDevice::intaker.SwitchIntakerFor();

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

void autonomous_2(){
	RopoDevice::Chassis.StartChassisAutoControll();//底盘MoveType设置为AutoMove
	// --------- begin ------------

	//---推场地中间4个球
	//RopoDevice::gpsAddPosition.SetUpdateFlag(1);
	RopoDevice::Chassis.AutoDirectMove(-0.44,0,true);
	delay();
	RopoDevice::Chassis.AutoRotateAbs(90);
	delay();
	RopoDevice::Chassis.AutoDirectMove(-0.44,-0.55,true);
	delay();
	RopoDevice::Chassis.AutoRotateAbs(0);
	delay();
	ControllerModule::UnderExternSwitch();
	//RopoDevice::gpsAddPosition.SetUpdateFlag(1);
	// 前移防止后杆因撞球放不下
	// RopoDevice::ThreeWire::SpadePneumatic.set_value(true);
	// RopoDevice::Chassis.MoveVelocity(0.2,0);
	// pros::delay(230);
	// RopoDevice::ThreeWire::WideExternPneumatic.set_value(true);
	ControllerModule::WideExternSwitch();
	RopoDevice::Chassis.MoveVelocity(-1.25,0);
	pros::delay(500);
	RopoDevice::Chassis.MoveVelocity(-0.5,0);
	pros::delay(650);
	RopoDevice::Chassis.MoveVelocity(0,0);
	ControllerModule::WideExternSwitch();
	pros::delay(500);
	//RopoDevice::gpsAddPosition.SetUpdateFlag(0);

	// 导联队球
	ControllerModule::UnderExternSwitch();
	RopoDevice::Chassis.AutoPositionMove(-0.27,0.343,-30);
	//RopoDevice::Chassis.AutoPositionMove(-0.40,0.422,-30);
	delay();
	ControllerModule::LeftExternSwitch();
	RopoDevice::Chassis.MoveVelocity(0.35,-0.8);
	pros::delay(1000);
	RopoDevice::Chassis.MoveVelocity(0.45,0);		//0.6
	pros::delay(400);
	RopoDevice::Chassis.MoveVelocity(0,0);
	pros::delay(100);
	ControllerModule::LeftExternSwitch();
	//ControllerModule::RightExternSwitch();
	pros::delay(500);
	RopoDevice::Chassis.AutoRotateAbs(80);
	pros::delay(100);
	RopoDevice::Chassis.AutoRotateAbs(110);
	delay();
	RopoDevice::Chassis.MoveVelocity(0.5,0.0);
	pros::delay(300);
	RopoDevice::Chassis.MoveVelocity(0,0);
	RopoDevice::Chassis.MoveVelocity(-1.5,-0.5);
	pros::delay(1000);
	//ControllerModule::RightExternSwitch();

	//---回到导入位置，通过转动导入球
	// 前翅膀导球
	RopoDevice::Chassis.MoveVelocity(0.5,0.55);
	pros::delay(1000);
	RopoDevice::Chassis.MoveVelocity(0,0);
	for(int i = 1; i<= 4; i++){
		pros::delay(200);
		// ControllerModule::RightExternSwitch();
		RopoDevice::Chassis.MoveVelocity(0.4,0.4);
		pros::delay(200);
		ControllerModule::RightExternSwitch();
		pros::delay(700);
		RopoDevice::Chassis.MoveVelocity(0,0);
		if (4 == i){
			RopoDevice::Chassis.AutoRotateAbs(150);
			delay();
			ControllerModule::RightExternSwitch();
			RopoDevice::Chassis.MoveVelocity(-0.4,-0.3);
			pros::delay(700);						// 800
			RopoDevice::Chassis.AutoRotateAbs(135);
			pros::delay(500);
			RopoDevice::Chassis.MoveVelocity(0.2,0.0);
			pros::delay(250);
		}else{
			//ControllerModule::RightExternSwitch();
			RopoDevice::Chassis.MoveVelocity(-0.4,-0.4);
			pros::delay(350);
			ControllerModule::RightExternSwitch();
			pros::delay(350);						// 800
		}
	}
	for (int i = 1; i <= 2; i++){					//
		pros::delay(200);
		ControllerModule::RightExternSwitch();
		if (i == 1)
		{
			RopoDevice::Chassis.MoveVelocity(0.4,0.4);
			pros::delay(700);			// 900
		}
		else
		{
			RopoDevice::Chassis.MoveVelocity(0.4,0.4);
			pros::delay(900);
		}
		RopoDevice::Chassis.MoveVelocity(0,0);
		// ControllerModule::RightExternSwitch();
		// RopoDevice::Chassis.AutoRotateAbs(-110);
		// delay();
		RopoDevice::Chassis.MoveVelocity(-0.4,-0.4);
		pros::delay(350);
		ControllerModule::RightExternSwitch();
		pros::delay(450);
	}
	

	RopoDevice::Chassis.AutoRotateAbs(135);
	delay();
	ControllerModule::BothExternSwitch();
	RopoDevice::Chassis.MoveVelocity(0.7, 0);	
	// while (fabs(RopoDevice::Position_Motor::MyPosition.Get_Angle() + 135) < 2 ){
	// 	pros::delay(10);
	// }
	pros::delay(500);
	RopoDevice::Chassis.MoveVelocity(0, 0);
	RopoDevice::Chassis.AutoRotateAbs(170);
	delay();
	RopoDevice::Chassis.MoveVelocity(0.75,0);
	pros::delay(350);
	ControllerModule::LeftExternSwitch();
	RopoDevice::Chassis.MoveVelocity(0.75,0.4);
	RopoDevice::gpsAddPosition.SetUpdateFlag(1);
	pros::delay(500);
	RopoDevice::Chassis.MoveVelocity(1.2,0);
	pros::delay(1000);							// 窄道直行
	RopoDevice::Chassis.MoveVelocity(0,0);
	

	

	//---直线加曲线推球入网

	// RopoDevice::Chassis.MoveVelocity(0.75,0);
	// while (RopoDevice::Position_Motor::MyPosition.Get_Y() > -1.45) {
	// pros::delay(15);
	// }
	//RopoDevice::Chassis.MoveVelocity(0,0);
	ControllerModule::LeftExternSwitch();
	if (fabs(int(RopoDevice::Position_Motor::MyPosition.Get_Angle() + 180) % 360) > 3) {
		RopoDevice::Chassis.AutoRotateAbs(181);
		delay();
	}
	RopoDevice::Chassis.MoveVelocity(0.75,0.95);		// 0.75, 0.95
	pros::delay(700);
	// ControllerModule::RightExternSwitch();
	pros::delay(750);
	ControllerModule::LeftExternSwitch();
	pros::delay(700);
	for (int i = 0; i < 1; i++)
	{	
		RopoDevice::Chassis.AutoRotateAbs(-90);
		pros::delay(400);
		RopoDevice::Chassis.MoveVelocity(-0.6,0);
		pros::delay(400);
		RopoDevice::Chassis.MoveVelocity(1.4,0);
		pros::delay(600);
	}
	RopoDevice::Chassis.MoveVelocity(0,0);
	RopoDevice::Chassis.MoveVelocity(-0.6,0);
	pros::delay(300);

	ControllerModule::SwitchIntakerFor();
	RopoDevice::Chassis.AutoRotateAbs(-125);
	delay();
	RopoDevice::Chassis.MoveVelocity(-1.0,0);
	pros::delay(1000);
	RopoDevice::Chassis.AutoRotateAbs(179);
	delay();
	ControllerModule::LeftExternSwitch();
	pros::delay(100);
	RopoDevice::Chassis.MoveVelocity(-1.0,0);
	pros::delay(1000);

	RopoDevice::Chassis.MoveVelocity(0,0);
}
