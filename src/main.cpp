#include "main.h"
#include "RopoController.hpp"
#include "RopoDevice.hpp"
#include "RopoPosition.hpp"
#include "pros/misc.h"
#include "pros/rtos.hpp"
void skill();
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

	bool underExternFlag = false;
	void UnderExternSwitch(){
		underExternFlag ^= 1; 
		RopoDevice::ThreeWire::UnderExternPneumatic.set_value(underExternFlag);
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
		intaker_forward = false;
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
	skill();
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
	ButtonDetectLine.AddButtonDetect(pros::E_CONTROLLER_DIGITAL_X    , RopoController::Rising, ControllerModule::UnderExternSwitch);
	ButtonDetectLine.AddButtonDetect(pros::E_CONTROLLER_DIGITAL_B    , RopoController::Rising, ControllerModule::ChangeCatch);
	ButtonDetectLine.AddButtonDetect(pros::E_CONTROLLER_DIGITAL_A    , RopoController::Rising, RopoDevice::ChassisBrake);
	ButtonDetectLine.AddButtonDetect(pros::E_CONTROLLER_DIGITAL_DOWN , RopoController::Rising,  ControllerModule::WideExternSwitch);
	ButtonDetectLine.AddButtonDetect(pros::E_CONTROLLER_DIGITAL_UP   , RopoController::Rising,  ControllerModule::GpsUpdate);
	ButtonDetectLine.AddButtonDetect(pros::E_CONTROLLER_DIGITAL_Y    , RopoController::Rising,  skill);

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

void skill(){
RopoDevice::Chassis.StartChassisAutoControll();//底盘MoveType设置为AutoMove
	RopoDevice::ChassisBrake();
	// --------- begin ------------
	ControllerModule::UnderExternSwitch();
	pros::delay(300);
	RopoDevice::Chassis.MoveVelocity(-1.0,0);
	pros::delay(500);
	// --------- stage 1 --------
	ControllerModule::UnderExternSwitch();
	// RopoDevice::Chassis.MoveVelocity(0.3,0);
	// pros::delay(500);
	// RopoDevice::Chassis.MoveVelocity(0,0);
	RopoDevice::Chassis.MoveVelocity(0.3,0.2);
	pros::delay(2000);
	RopoDevice::Chassis.MoveVelocity(0,0);
	for(int i = 1; i<= 15; i++){
		pros::delay(200);
		ControllerModule::RightExternSwitch();
		RopoDevice::Chassis.MoveVelocity(0.4,0.4);
		pros::delay(900);
		RopoDevice::Chassis.MoveVelocity(0,0);
		if (i % 4 == 0){
			RopoDevice::Chassis.AutoRotateAbs(70);
			delay();
			ControllerModule::RightExternSwitch();
			RopoDevice::Chassis.MoveVelocity(-0.4,-0.3);
			pros::delay(800);
			RopoDevice::Chassis.AutoRotateAbs(45);
			pros::delay(500);
			RopoDevice::Chassis.MoveVelocity(0.2,0.0);
			pros::delay(250);
		}else{
			ControllerModule::RightExternSwitch();
			RopoDevice::Chassis.MoveVelocity(-0.4,-0.4);
			pros::delay(800);
			}
	}

	// --------- stage 2 ----------
	RopoDevice::Chassis.AutoRotateAbs(45);
	delay();
	ControllerModule::BothExternSwitch();
	RopoDevice::Chassis.MoveVelocity(0.5, 0);	
	// while (fabs(RopoDevice::Position_Motor::MyPosition.Get_Angle() + 135) < 2 ){
	// 	pros::delay(10);
	// }
	pros::delay(600);
	RopoDevice::Chassis.MoveVelocity(0, 0);
	RopoDevice::Chassis.AutoRotateAbs(80);
	delay();
	ControllerModule::SwitchIntakerBack();
	RopoDevice::Chassis.MoveVelocity(0.75,0);
	pros::delay(350);
	ControllerModule::LeftExternSwitch();
	RopoDevice::gpsAddPosition.SetUpdateFlag(1);
	pros::delay(2100);							// 窄道直行
	RopoDevice::Chassis.MoveVelocity(0,0);
	ControllerModule::LeftExternSwitch();	// 开左翅膀
	if (fabs(RopoDevice::Position_Motor::MyPosition.Get_Angle() - 90) > 10 )
	{
		RopoDevice::Chassis.AutoRotateAbs(90);
		delay();
	}
	
	// --------- stage 3 --------------
	RopoDevice::Chassis.MoveVelocity(0.8, 1.8);	// 0.9, 0.75
	pros::delay(1000);
	RopoDevice::Chassis.MoveVelocity(0, 0);

	RopoDevice::Chassis.AutoPositionMove(-0.11, 2.730, 179, 3000);
	ControllerModule::LeftExternSwitch();	// 关左翅膀
	ControllerModule::SwitchIntakerBack();
	for (int i = 0; i < 3; i++)
	{
		RopoDevice::Chassis.MoveVelocity(-0.6, 0);
		pros::delay(500);
		RopoDevice::Chassis.AutoRotateAbs(130 - i * 3);
		delay();
		RopoDevice::Chassis.MoveVelocity(1.5, -0.5);
		pros::delay(600);
	}
	ControllerModule::SwitchIntakerBack();	// 反转 roller 帮助车退出
	RopoDevice::Chassis.MoveVelocity(-0.5, 0);
	pros::delay(700);
	ControllerModule::RightExternSwitch();
	ControllerModule::SwitchIntakerBack();	// 关闭反转
	RopoDevice::Chassis.AutoRotateAbs(135);
	delay();

	RopoDevice::Chassis.MoveVelocity(-0.5, 0);
	pros::delay(1300);
	RopoDevice::gpsAddPosition.SetUpdateFlag(0);

	// --------- end --------------
	pros::delay(300);
	RopoDevice::Chassis.MoveVelocity(0,0);
}