#include "main.h"
#include "RopoController.hpp"
#include "RopoDevice.hpp"
#include "RopoPosition.hpp"
#include "pros/misc.h"
#include "pros/rtos.hpp"
void autonomous_1();
void autonomous_2();
void autonomous_3();
void test();
namespace ControllerModule {

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

	bool intaker_forward = false;
	bool intaker_backward = false;
	void RollIntaker(){
		if (intaker_forward && !intaker_backward) {
			RopoDevice::Motors::IntakeMotor.move_voltage(-7000);
		}
		else if (!intaker_forward && intaker_backward) {
			RopoDevice::Motors::IntakeMotor.move_voltage(7000);
		}
		else {
			RopoDevice::Motors::IntakeMotor.move_voltage(0);
		}
	}

	void SwitchIntakerFor(){
		intaker_forward ^= 1;
		intaker_backward = false;
		RollIntaker();
	}

	void SwitchIntakerBack(){
		intaker_backward ^= 1;
		RollIntaker();
	}

	void SwitchIntakerForToBack(){
		intaker_forward ^= 1;
		intaker_backward ^= 1;
		pros::delay(50);
		RollIntaker();
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
	autonomous_2();
}

void opcontrol()
{
	//pros::Task *RumbleTask = new pros::Task(ControllerModule::RumbleMe);
	pros::Task *PrintTask = new pros::Task(ControllerModule::ControllerPrint);
	pros::Controller MasterController(pros::E_CONTROLLER_MASTER);
	RopoController::ButtonTaskLine ButtonDetectLine(MasterController);
	FloatType VelocityMax = 1.7;	// 1.4 m/s
	FloatType WcMax = 7;	// 6  wyj反映太快了 
	FloatType VelocityRestrainRatio = 0.4; // 0 ~ 1
	FloatType WcRestrainRatio = 0.4; // 0 ~ 1
	bool ChassisMove = false;
	
	RopoController::AxisValueCast XVelocityInput(MasterController,pros::E_CONTROLLER_ANALOG_LEFT_Y,RopoController::Linear);
	RopoController::AxisValueCast WVelocityInput(MasterController,pros::E_CONTROLLER_ANALOG_RIGHT_X,RopoController::Exp);

	Vector Velocity(RopoMath::ColumnVector,2),ResVelocity;

	MasterController.clear();
	ButtonDetectLine.AddButtonDetect(pros::E_CONTROLLER_DIGITAL_R1   , RopoController::Rising, ControllerModule::BothExternSwitch);
	ButtonDetectLine.AddButtonDetect(pros::E_CONTROLLER_DIGITAL_R2   , RopoController::Rising,ControllerModule::ChangeLift);
	ButtonDetectLine.AddButtonDetect(pros::E_CONTROLLER_DIGITAL_L1   , RopoController::DoubleEdge, ControllerModule::SwitchIntakerForToBack);
	ButtonDetectLine.AddButtonDetect(pros::E_CONTROLLER_DIGITAL_L2   , RopoController::Rising, ControllerModule::SwitchIntakerFor);
	ButtonDetectLine.AddButtonDetect(pros::E_CONTROLLER_DIGITAL_LEFT , RopoController::Rising, ControllerModule::LeftExternSwitch);
	ButtonDetectLine.AddButtonDetect(pros::E_CONTROLLER_DIGITAL_RIGHT , RopoController::Rising, ControllerModule::RightExternSwitch);
	ButtonDetectLine.AddButtonDetect(pros::E_CONTROLLER_DIGITAL_X    , RopoController::Rising, ControllerModule::TurnAround);
	ButtonDetectLine.AddButtonDetect(pros::E_CONTROLLER_DIGITAL_B    , RopoController::Rising, ControllerModule::ChangeCatch);
	ButtonDetectLine.AddButtonDetect(pros::E_CONTROLLER_DIGITAL_A    , RopoController::Rising, RopoDevice::ChassisBrake);
	ButtonDetectLine.AddButtonDetect(pros::E_CONTROLLER_DIGITAL_DOWN , RopoController::Rising,  ControllerModule::WideExternSwitch);
	ButtonDetectLine.AddButtonDetect(pros::E_CONTROLLER_DIGITAL_UP   , RopoController::Rising,  ControllerModule::GpsUpdate);
	ButtonDetectLine.AddButtonDetect(pros::E_CONTROLLER_DIGITAL_Y    , RopoController::Pressing,  autonomous_1);
	ButtonDetectLine.AddButtonDetect(pros::E_CONTROLLER_DIGITAL_Y , RopoController::DoubleClick,  autonomous_2);

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
}

void delay(){ //代替pros::delay()进行时间控制
	while(!RopoDevice::Chassis.IfArrived()){
		pros::delay(15);
	}
		return;
}

void autonomous_1(){
	RopoDevice::Chassis.StartChassisAutoControll();//底盘MoveType设置为AutoMove
	// --------- begin ------------

	//---推场地中间4个球
	RopoDevice::gpsAddPosition.SetUpdateFlag(0);
	RopoDevice::Chassis.AutoDirectMove(0.92,0,0);
	delay();
	// RopoDevice::gpsAddPosition.SetUpdateFlag(10);
	RopoDevice::Chassis.AutoRotateAbs(90);
	delay();
	// 前移防止后杆因撞球放不下
	// RopoDevice::ThreeWire::SpadePneumatic.set_value(true);
	// RopoDevice::Chassis.MoveVelocity(0.2,0);
	// pros::delay(230);
	// RopoDevice::ThreeWire::WideExternPneumatic.set_value(true);
	RopoDevice::Chassis.MoveVelocity(-1.25,0);
	pros::delay(700);
	// RopoDevice::Position_Motor::MyPosition.Set_XY(1.11, -0.70);
	// 前移防止后杆收不回来
	// RopoDevice::Chassis.MoveVelocity(0.4,0);
	// pros::delay(280);
	// RopoDevice::ThreeWire::WideExternPneumatic.set_value(false);

	//---回到导入位置，对准角度，通过转动导入球
	// 前翅膀导球
	RopoDevice::Chassis.AutoPositionMove(0.05,0.33,-123);
	delay();
	RopoDevice::Chassis.MoveVelocity(-0.2,-0.3);
	pros::delay(2000);
	RopoDevice::Chassis.MoveVelocity(0,0);
	RopoDevice::Chassis.MoveVelocity(0.3,0.2);
	pros::delay(1000);
	RopoDevice::Chassis.MoveVelocity(0,0);
	for(int i = 1; i<= 4; i++){
		pros::delay(200);
		ControllerModule::RightExternSwitch();
		RopoDevice::Chassis.MoveVelocity(0.4,0.4);
		pros::delay(900);
		RopoDevice::Chassis.MoveVelocity(0,0);
		if (4 == i){
			RopoDevice::Chassis.AutoRotateAbs(-110);
			delay();
			ControllerModule::RightExternSwitch();
			RopoDevice::Chassis.MoveVelocity(-0.4,-0.3);
			pros::delay(800);
			RopoDevice::Chassis.AutoRotateAbs(-135);
			pros::delay(500);
			RopoDevice::Chassis.MoveVelocity(0.2,0.0);
			pros::delay(250);
		}else{
			ControllerModule::RightExternSwitch();
			RopoDevice::Chassis.MoveVelocity(-0.4,-0.4);
			pros::delay(800);
			}
	}
	for (int i = 1; i <= 3; i++){
		pros::delay(200);
		ControllerModule::RightExternSwitch();
		RopoDevice::Chassis.MoveVelocity(0.4,0.4);
		pros::delay(900);
		RopoDevice::Chassis.MoveVelocity(0,0);
		ControllerModule::RightExternSwitch();
		// RopoDevice::Chassis.AutoRotateAbs(-110);
		// delay();
		RopoDevice::Chassis.MoveVelocity(-0.4,-0.4);
		pros::delay(800);
	}
	
	// 后翅膀导球
	// RopoDevice::Chassis.MoveVelocity(0,-5);
	// pros::delay(800);
	// // RopoDevice::Chassis.MoveVelocity(0,10);
	// // pros::delay(200);							// 反转防止翅膀无法收回影响位置
	// // RopoDevice::gpsAddPosition.SetUpdateFlag(0);
	// RopoDevice::Chassis.AutoPositionMoveBack(0.4,0.02,-50);
	// delay();
	// // RopoDevice::Chassis.AutoPositionMoveBack(0.05,0.330); // -0.01,0.411;-0.06,0.52
	// RopoDevice::Chassis.MoveVelocity(-0.5,0);
	// pros::delay(900);
	// RopoDevice::Chassis.MoveVelocity(0,0);
	// // RopoDevice::gpsAddPosition.SetUpdateFlag(10);
	// // RopoDevice::Chassis.AutoRotateAbs(135);
	// // delay();
	// ControllerModule::ChangeLift();
	// // RopoDevice::Chassis.MoveVelocity(-0.35,0);  //靠杆
	// // pros::delay(800);
	// // RopoDevice::Chassis.MoveVelocity(0,0);
	// RopoDevice::gpsAddPosition.SetUpdateFlag(1);
	// for (int i = 1; i <= 6; i++) {  
	// RopoDevice::Chassis.MoveVelocity(-0.2,6);
	// pros::delay(200);
	// // 开环
	// // RopoDevice::Chassis.MoveVelocity(0,0);
	// // RopoDevice::Chassis.MoveVelocity(0.1,-5);
	// // pros::delay(400);
	// // 闭环
	// RopoDevice::Chassis.AutoPositionMoveBack(-0.03,0.45,-50);
	// delay();
	// RopoDevice::Chassis.MoveVelocity(-0.6,0);  //靠杆
	// pros::delay(500);
	// }
	// // RopoDevice::gpsAddPosition.SetUpdateFlag(1);
	// pros::delay(100);
	// RopoDevice::gpsAddPosition.SetUpdateFlag(0);	// 更新GPS

	// RopoDevice::Chassis.AutoPositionMove(-0.15, 0.35, -120);	// 0.16, 0.224, -120
	// delay();
	RopoDevice::Chassis.AutoRotateAbs(-135);
	delay();
	ControllerModule::BothExternSwitch();
	RopoDevice::Chassis.MoveVelocity(0.5, 0);	
	while (fabs(RopoDevice::Position_Motor::MyPosition.Get_Angle() + 135) < 2 ){
		pros::delay(20);
	}
	RopoDevice::Chassis.MoveVelocity(0, 0);
	RopoDevice::Chassis.AutoRotateAbs(-100);
	delay();
	RopoDevice::Chassis.MoveVelocity(0.75,0);
	pros::delay(150);
	ControllerModule::LeftExternSwitch();
	pros::delay(2100);							// 窄道直行
	RopoDevice::Chassis.MoveVelocity(0,0);
	

	

	//---直线加曲线推球入网

	// RopoDevice::Chassis.MoveVelocity(0.75,0);
	// while (RopoDevice::Position_Motor::MyPosition.Get_Y() > -1.45) {
	// pros::delay(15);
	// }
	RopoDevice::Chassis.MoveVelocity(0,0);
	ControllerModule::LeftExternSwitch();
	RopoDevice::Chassis.AutoRotateAbs(-90);
	pros::delay(200);
	RopoDevice::Chassis.MoveVelocity(0.67,0.95);
	pros::delay(700);
	// ControllerModule::RightExternSwitch();
	pros::delay(950);
	ControllerModule::LeftExternSwitch();
	pros::delay(500);
	for (int i = 0; i < 2; i++)
	{	
		RopoDevice::Chassis.AutoRotateAbs(0);
		pros::delay(500);
		RopoDevice::Chassis.MoveVelocity(-0.6,0);
		pros::delay(500);
		RopoDevice::Chassis.MoveVelocity(1.4,0);
		pros::delay(500);
	}

	RopoDevice::Chassis.MoveVelocity(0,0);
	// RopoDevice::Chassis.MoveVelocity(0.5,0);
	// pros::delay(600);
	// RopoDevice::Chassis.AutoRotateAbs(-30);
	// pros::delay(500);
	// RopoDevice::Chassis.MoveVelocity(-0.7,0.3);
	// pros::delay(600);

	//---开到三角区，勾出联队粽球并送入网（由于GPS实装，正在考虑用闭环代替）

	// RopoDevice::Chassis.AutoRotateAbs(175);
	// delay();
	// RopoDevice::Chassis.MoveVelocity(0.55,0.20);
	// pros::delay(2000);
	// RopoDevice::Chassis.MoveVelocity(0,0);
	// RopoDevice::gpsAddPosition.SetUpdateFlag(0);  //停用GPS并设置相对坐标
	// pros::delay(150);
	// ControllerModule::ChangeIntakerPneumatic();
	// RopoDevice::Position_Motor::MyPosition.Set_XY(0, 0);
	// pros::delay(150);
	// ControllerModule::Lift();
	// pros::delay(800);
	// ControllerModule::Intake();
	// RopoDevice::Chassis.MoveVelocity(-0.4,0);
	// pros::delay(800);
	// RopoDevice::Chassis.MoveVelocity(0,0);
	// ControllerModule::Hide();
	// pros::delay(300);
	// RopoDevice::Chassis.MoveVelocity(0.3,0);
	// pros::delay(300);
	// RopoDevice::Chassis.MoveVelocity(0,0);
	// RopoDevice::Chassis.AutoRotateAbs(-45);
	// delay();
	// RopoDevice::Chassis.MoveVelocity(0.55,0.5);
	// pros::delay(600);
	// RopoDevice::Chassis.AutoRotateAbs(0);
	// delay();
	// ControllerModule::IntakerStop();
	// ControllerModule::ChangeIntakerPneumatic();
	// RopoDevice::gpsAddPosition.SetUpdateFlag(10);
	// RopoDevice::Chassis.MoveVelocity(0.7,0);
	// pros::delay(750);
	// RopoDevice::Chassis.MoveVelocity(0,0);

	// //---回过道碰爬升杆

	// RopoDevice::Chassis.MoveVelocity(-0.62,-0.9);
	// pros::delay(2050);
	// RopoDevice::Chassis.MoveVelocity(0,0);
	// pros::delay(30);
	// RopoDevice::Chassis.AutoRotateAbs(-90);
	// RopoDevice::Chassis.MoveVelocity(-0.6,0);
	// pros::delay(1600);
	// RopoDevice::Chassis.MoveVelocity(0,0);
	// pros::delay(30);
	// // --------- end --------------
	// pros::delay(300);
	// RopoDevice::Chassis.MoveVelocity(0,0);

	pros::delay(300);
	RopoDevice::Chassis.MoveVelocity(0,0);
}

void autonomous_2(){
	RopoDevice::Chassis.StartChassisAutoControll();//底盘MoveType设置为AutoMove
	RopoDevice::ChassisBrake();
	// --------- begin ------------
	ControllerModule::RightExternSwitch();
	
	// --------- stage 1 --------
	RopoDevice::Chassis.AutoDirectMove(1.35, 0, false);
	pros::delay(1100);

	//RopoDevice::gpsAddPosition.SetUpdateFlag(1);
	RopoDevice::Chassis.MoveVelocity(0.9, 0.75);
	pros::delay(1000);
	//RopoDevice::gpsAddPosition.SetUpdateFlag(0);

	// ControllerModule::RightExternSwitch();
	// RopoDevice::Chassis.AutoPositionMove(2.60,0.22);
	// ControllerModule::RightExternSwitch();
	// pros::delay(500);
	// RopoDevice::Chassis.AutoRotateAbs(75);
	// pros::delay(500);

	RopoDevice::Chassis.AutoPositionMove(1.8, 0.6, 90);
	ControllerModule::SwitchIntakerFor();	// 关 roller 避免球脱落
	RopoDevice::Chassis.MoveVelocity(-0.6, 0);
	pros::delay(300);
	RopoDevice::Chassis.MoveVelocity(1.2, 0);
	pros::delay(600);
	RopoDevice::Chassis.MoveVelocity(0, 0);

	// --------- stage 2 ----------
	
	RopoDevice::gpsAddPosition.SetUpdateFlag(1);
	RopoDevice::Chassis.AutoRotateAbs(90);
	pros::delay(800);

	RopoDevice::Chassis.MoveVelocity(-0.5, 0);
	pros::delay(300);
	ControllerModule::RightExternSwitch();
	pros::delay(2500);
	RopoDevice::Chassis.MoveVelocity(0, 0);
	ControllerModule::Hold();
	pros::delay(1000);

	RopoDevice::Chassis.MoveVelocity(0.6, 0.1);
	pros::delay(500);
	RopoDevice::Chassis.MoveVelocity(0.8, 0);
	pros::delay(800);
	RopoDevice::gpsAddPosition.SetUpdateFlag(0);

	// --------- stage 3 ----------
	RopoDevice::Chassis.AutoPositionMove(1.42, 1.3);
	
	pros::delay(500);
	RopoDevice::Chassis.AutoRotateAbs(182);
	pros::delay(400);
	ControllerModule::Hide();
	pros::delay(600);
	ControllerModule::SwitchIntakerFor();	// 开 roller 吃球
	
	ControllerModule::WideExternSwitch();	// 开横翅膀
	pros::delay(500);
	RopoDevice::Chassis.MoveVelocity(-1.2, 0);
	pros::delay(800);

	RopoDevice::Chassis.MoveVelocity(0.4, 0);
	pros::delay(300);
	ControllerModule::WideExternSwitch();	// 关横翅膀
	RopoDevice::Chassis.AutoPositionMove(1.65, 1.25, -1);
	ControllerModule::SwitchIntakerFor();	// 关 roller 吃球
	ControllerModule::BothExternSwitch();	// 开两侧翅膀
	pros::delay(300);

	RopoDevice::gpsAddPosition.SetUpdateFlag(1);
	RopoDevice::Chassis.MoveVelocity(1, 0);
	pros::delay(800);
	RopoDevice::Chassis.MoveVelocity(0, 0);
	RopoDevice::ChassisHold();
	RopoDevice::gpsAddPosition.SetUpdateFlag(0);
	pros::delay(500);
	RopoDevice::ChassisBrake();

	// --------- stage 4 --------------
	RopoDevice::Chassis.MoveVelocity(-0.5, -0);
	pros::delay(500);
	ControllerModule::BothExternSwitch();	// 关两侧翅膀
	pros::delay(200);
	ControllerModule::SwitchIntakerFor();	// 开 roller 吃球
	RopoDevice::Chassis.AutoPositionMove(1.45, 0.9, -145);
	pros::delay(500);
	RopoDevice::Chassis.AutoPositionMove(1.6,1.25,2);
	ControllerModule::SwitchIntakerFor();	// 关 roller 吃球
	ControllerModule::BothExternSwitch();	// 开两侧翅膀
	RopoDevice::Chassis.MoveVelocity(1.2, 0);
	pros::delay(800);
	
	
	// --------- end --------------
	pros::delay(300);
	RopoDevice::Chassis.MoveVelocity(0,0);
}

void autonomous_3() {
	RopoDevice::Chassis.StartChassisAutoControll();//底盘MoveType设置为AutoMove
	RopoDevice::ChassisBrake();
	// --------- begin ------------
	//ControllerModule::RightExternSwitch();
	
	// --------- stage 1 --------
	RopoDevice::Chassis.AutoDirectMove(1.35, 0, false);
	pros::delay(1100);

	//RopoDevice::gpsAddPosition.SetUpdateFlag(1);
	RopoDevice::Chassis.MoveVelocity(0.9, 0.8);
	pros::delay(400);
	//RopoDevice::gpsAddPosition.SetUpdateFlag(0);

	// ControllerModule::RightExternSwitch();
	// RopoDevice::Chassis.AutoPositionMove(2.60,0.22);
	// ControllerModule::RightExternSwitch();
	// pros::delay(500);
	// RopoDevice::Chassis.AutoRotateAbs(75);
	// pros::delay(500);

	ControllerModule::RightExternSwitch();
	pros::delay(600);

	RopoDevice::Chassis.AutoPositionMove(1.8, 0.6, 90);
	ControllerModule::SwitchIntakerFor();	// 关 roller 避免球脱落
	RopoDevice::gpsAddPosition.SetUpdateFlag(1);
	for (int i = 0; i < 2; i++)
	{
		RopoDevice::Chassis.MoveVelocity(-0.5, 0);
		pros::delay(300);
		RopoDevice::Chassis.MoveVelocity(1.2, 0);
		pros::delay(600);
	}
	RopoDevice::Chassis.MoveVelocity(0, 0);
	pros::delay(200);
	RopoDevice::gpsAddPosition.SetUpdateFlag(0);

	// --------- stage 2 ----------
	RopoDevice::Chassis.MoveVelocity(-0.8, -0.8);
	pros::delay(500);
	ControllerModule::RightExternSwitch();
	pros::delay(1000);
	RopoDevice::Chassis.AutoPositionMove(1.42, 1.3);
	
	pros::delay(500);
	RopoDevice::Chassis.AutoRotateAbs(182);
	pros::delay(400);
	ControllerModule::Hide();
	pros::delay(600);
	ControllerModule::SwitchIntakerFor();	// 开 roller 吃球
	
	ControllerModule::WideExternSwitch();	// 开横翅膀
	pros::delay(500);
	RopoDevice::Chassis.MoveVelocity(-1.2, 0);
	pros::delay(800);

	RopoDevice::Chassis.MoveVelocity(0.4, 0);
	pros::delay(300);
	ControllerModule::WideExternSwitch();	// 关横翅膀
	RopoDevice::Chassis.AutoPositionMove(1.65, 1.25, -1);
	ControllerModule::SwitchIntakerFor();	// 关 roller 吃球
	ControllerModule::BothExternSwitch();	// 开两侧翅膀
	pros::delay(300);

	RopoDevice::gpsAddPosition.SetUpdateFlag(1);
	RopoDevice::Chassis.MoveVelocity(1, 0);
	pros::delay(800);
	RopoDevice::Chassis.MoveVelocity(0, 0);
	RopoDevice::ChassisHold();
	RopoDevice::gpsAddPosition.SetUpdateFlag(0);
	pros::delay(500);
	RopoDevice::ChassisBrake();
	
	// --------- stage 3 --------------

	
	
	// --------- end --------------
	pros::delay(300);
	RopoDevice::Chassis.MoveVelocity(0,0);
}

void test(){
	RopoDevice::Chassis.StartChassisAutoControll();//底盘MoveType设置为AutoMove
	RopoDevice::ChassisBrake();
	RopoDevice::Chassis.AutoPositionMove(1.12, -0.287);

	// --------- end --------------
	pros::delay(300);
	RopoDevice::Chassis.MoveVelocity(0,0);
}