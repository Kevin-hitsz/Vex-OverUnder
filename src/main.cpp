#include "main.h"
#include "RopoController.hpp"
#include "RopoDevice.hpp"
#include "RopoPosition.hpp"
#include "pros/misc.h"
#include "pros/rtos.hpp"
void test();
void skill();
void autonomous_1();
namespace ControllerModule {

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
	void ChangeCatch(){
		locktag ^= 1;
		RopoDevice::ThreeWire::CatchPneumatic.set_value(locktag);
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

	void Lift(){
		RopoDevice::LiftMotors.Wait();
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
			RopoDevice::Motors::IntakeMotor.move_velocity(-500);
		}
		else if (!intaker_forward && intaker_backward) {
			RopoDevice::Motors::IntakeMotor.move_velocity(500);
		}
		else {
			RopoDevice::Motors::IntakeMotor.move_velocity(0);
		}
	}

	void SwitchIntakerFor(){
		intaker_forward ^= 1;
		RollIntaker();
	}

	void SwitchIntakerBack(){
		intaker_backward ^= 1;
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
	autonomous_1();
	//skill();
}

void opcontrol()
{

	//pros::Task *RumbleTask = new pros::Task(ControllerModule::RumbleMe);
	pros::Task *PrintTask = new pros::Task(ControllerModule::ControllerPrint);
	pros::Controller MasterController(pros::E_CONTROLLER_MASTER);
	RopoController::ButtonTaskLine ButtonDetectLine(MasterController);
	FloatType VelocityMax = 1.8;//1.4
	FloatType RopoWcLimit = 5;
	bool ChassisMove = false;
	
	RopoController::AxisValueCast XVelocityInput(MasterController,pros::E_CONTROLLER_ANALOG_LEFT_Y,RopoController::Linear);
	RopoController::AxisValueCast WVelocityInput(MasterController,pros::E_CONTROLLER_ANALOG_RIGHT_X,RopoController::Linear);

	Vector Velocity(RopoMath::ColumnVector,2),ResVelocity;

	MasterController.clear();
	ButtonDetectLine.AddButtonDetect(pros::E_CONTROLLER_DIGITAL_R1, RopoController::Rising, ControllerModule::ChangeLift);
	ButtonDetectLine.AddButtonDetect(pros::E_CONTROLLER_DIGITAL_R2,RopoController::Rising,ControllerModule::ChangeCatch);
	ButtonDetectLine.AddButtonDetect(pros::E_CONTROLLER_DIGITAL_L1, RopoController::DoubleEdge, ControllerModule::SwitchIntakerFor);
	ButtonDetectLine.AddButtonDetect(pros::E_CONTROLLER_DIGITAL_L2, RopoController::DoubleEdge, ControllerModule::SwitchIntakerBack);
	ButtonDetectLine.AddButtonDetect(pros::E_CONTROLLER_DIGITAL_X, RopoController::Rising, ControllerModule::TurnAround);
	ButtonDetectLine.AddButtonDetect(pros::E_CONTROLLER_DIGITAL_B, RopoController::Rising, ControllerModule::Switch);
	ButtonDetectLine.AddButtonDetect(pros::E_CONTROLLER_DIGITAL_A  , RopoController::Rising,  RopoAuto::Auto_Find);
	ButtonDetectLine.AddButtonDetect(pros::E_CONTROLLER_DIGITAL_UP  , RopoController::Rising,  autonomous);
	ButtonDetectLine.AddButtonDetect(pros::E_CONTROLLER_DIGITAL_LEFT , RopoController::Rising,  test);
	ButtonDetectLine.AddButtonDetect(pros::E_CONTROLLER_DIGITAL_DOWN , RopoController::Rising,  ControllerModule::GpsUpdate);

	ButtonDetectLine.Enable();

	while (true) {
		FloatType XInput =  XVelocityInput.GetAxisValue();
		FloatType WInput = -WVelocityInput.GetAxisValue();
		FloatType RopoWc = RopoWcLimit-fabs(XInput)*1.1;			

		if (fabs(XInput) <= 0.06 && fabs(WInput) <= 0.03 ) {
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
		pros::delay(4);
	}
}

void test(){
	RopoDevice::Chassis.AutoRotateAbs(90);
	while(!RopoDevice::Chassis.IfArrived()) pros::delay(50);
	RopoDevice::Chassis.AutoPositionMove(0,0.5,-90);
	//RopoDevice::Chassis.AutoPositionMove(0.5,-0.5,-90);
}

void autonomous_1(){
	// --------- begin ------------
	RopoDevice::Chassis.MoveVelocity(0.8,0);
	pros::delay(500);
	RopoDevice::Chassis.AutoRotateAbs(-60);
	pros::delay(1000);
	ControllerModule::SwitchIntakerBack();//1+

	pros::delay(700);
	ControllerModule::SwitchIntakerBack();//1-			吐

	ControllerModule::SwitchIntakerFor();//1+
	RopoDevice::Chassis.AutoPositionMove(0.55,0.35);
	RopoDevice::Chassis.MoveVelocity(0.2,0);
	pros::delay(200);
	RopoDevice::Chassis.MoveVelocity(0.0,0);
	pros::delay(100);
	ControllerModule::SwitchIntakerFor();//1-			吃

	RopoDevice::Chassis.AutoRotateAbs(-90);
	pros::delay(1000);
	RopoDevice::Chassis.AutoPositionMove(0.55,0.1);

	ControllerModule::SwitchIntakerBack();//1+
	pros::delay(700);
	ControllerModule::SwitchIntakerBack();//1-			吐

	RopoDevice::Chassis.AutoDirectMove(0.55,0.46,true);
	pros::delay(800);
	ControllerModule::ChangeLift();		// 放
	pros::delay(1500);

	RopoDevice::Chassis.MoveVelocity(0.3,0);
	pros::delay(1000);
	RopoDevice::Chassis.MoveVelocity(0,1);
	pros::delay(500);
	RopoDevice::Chassis.AutoRotateAbs(90);
	pros::delay(1000);
	ControllerModule::ChangeLift();		// 收
	pros::delay(800);
	RopoDevice::Chassis.MoveVelocity(-0.4,0);
	pros::delay(600);

	RopoDevice::Chassis.AutoPositionMove(0.62,0.401);
	RopoDevice::Chassis.AutoRotateAbs(5);
	pros::delay(1000);

	ControllerModule::SwitchIntakerBack();//1+
	RopoDevice::Chassis.AutoPositionMove(0.85,0.401);
	pros::delay(500);
	ControllerModule::SwitchIntakerBack();//1-

	// --------- end --------------
	pros::delay(300);
	RopoDevice::Chassis.MoveVelocity(0,0);
}

void skill(){
	//*推预装球
	RopoDevice::Chassis.MoveVelocity(0.8,0);
	pros::delay(600);
	RopoDevice::Chassis.MoveVelocity(-0.35,0);
	pros::delay(1200);
	RopoDevice::Chassis.MoveVelocity(-0.3,0);
	pros::delay(500);
	RopoDevice::Chassis.MoveVelocity(-0.2,0.1);
	for(int i = 1; i <= 9; i++){
		//*1-----------抓三角区球
		ControllerModule::ChangeLift();
		pros::delay(500);
		
		RopoDevice::Chassis.MoveVelocity(0,0);
		pros::delay(380);
		RopoDevice::Chassis.MoveVelocity(0.5,0);
		pros::delay(520);
		RopoDevice::Chassis.MoveVelocity(0,0);
		//*--------------推走三角区球

		pros::delay(100);
		RopoDevice::Chassis.MoveVelocity(0,-5);
		pros::delay(1000);		
		
		RopoDevice::Chassis.AutoRotateAbs(44);
		pros::delay(300);
		ControllerModule::ChangeLift();
		RopoDevice::Chassis.MoveVelocity(-0.3,-0.35);
		pros::delay(600);
		RopoDevice::Chassis.MoveVelocity(-0.2,0.3);
	}
	ControllerModule::ChangeLift();
	pros::delay(500);
	
	RopoDevice::Chassis.MoveVelocity(0,0);
	pros::delay(450);
	RopoDevice::Chassis.MoveVelocity(0.3,0);
	pros::delay(800);
	RopoDevice::Chassis.MoveVelocity(0,0);
	//*--------------推走三角区球

	pros::delay(100);
	RopoDevice::Chassis.MoveVelocity(0,-5);
	pros::delay(1000);		
	RopoDevice::Chassis.AutoRotateAbs(145);
	pros::delay(800);
	//10
	RopoDevice::Chassis.MoveVelocity(0,0);


	/*-------------推二号球
	//ControllerModule::GpsUpdate();
	RopoDevice::Chassis.AutoPositionMove(0.34,0.83,90);
	ControllerModule::SwitchIntakerFor();//1+
	RopoDevice::Chassis.MoveVelocity(0.4,0);
	pros::delay(1000);
	ControllerModule::SwitchIntakerFor();//1-
	//吐
	ControllerModule::SwitchIntakerBack();//2+
	RopoDevice::Chassis.MoveVelocity(-0.3,0);
	pros::delay(700);
	ControllerModule::SwitchIntakerBack();//2-
	RopoDevice::Chassis.MoveVelocity(0,0);

	//*---------------推三号球
	ControllerModule::SwitchIntakerFor();//1+
	RopoDevice::Chassis.AutoPositionMove(0.87,0.83,90);
	RopoDevice::Chassis.MoveVelocity(0.4,0);
	pros::delay(1000);
	ControllerModule::SwitchIntakerFor();//1-
	//吐
	ControllerModule::SwitchIntakerBack();//2+
	RopoDevice::Chassis.MoveVelocity(-0.3,0);
	pros::delay(700);
	ControllerModule::SwitchIntakerBack();//2-
	RopoDevice::Chassis.MoveVelocity(0,0);
	//*
	
	*/
}
