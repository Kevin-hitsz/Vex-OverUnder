#include "main.h"
#include "RopoApi.hpp"
#include "RopoController.hpp"
#include "RopoDevice.hpp"
#include "RopoGpsAddPosition.hpp"
#include "RopoMath/Misc.hpp"
#include "RopoPosition.hpp"
#include "pros/misc.h"
#include "pros/rtos.hpp"
#include <cmath>
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
	void ChangeExtern(){
		locktag ^= 1;
		RopoDevice::ThreeWire::ExternPneumatic.set_value(locktag);
	}

	void RumbleMe(){
		pros::Controller MasterController1(pros::E_CONTROLLER_MASTER);
		MasterController1.rumble("-.-.-");
		FloatType aa = pros::millis();
		while(pros::millis()-aa<45000){
			pros::delay(100);
		}
		RopoDevice::ChassisBrake();
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

	bool wing_status = false;
	bool spade_status = false;
	void TogetherPush(){
		wing_status = true;
		spade_status = true;
		RopoDevice::ThreeWire::WingPneumatic.set_value(true);
		RopoDevice::ThreeWire::SpadePneumatic.set_value(true);
	}
	void TogetherUnpush(){
		wing_status = false;
		spade_status = false;
		RopoDevice::ThreeWire::WingPneumatic.set_value(false);
		RopoDevice::ThreeWire::SpadePneumatic.set_value(false);
	}
	void WingPush(){
		wing_status = true;
		RopoDevice::ThreeWire::WingPneumatic.set_value(true);
	}
	void WingUnpush(){
		wing_status = false;
		RopoDevice::ThreeWire::WingPneumatic.set_value(false);
	}

	void ChangeWingStatus(){
		if(spade_status == false) {
			wing_status ^= 1;
			RopoDevice::ThreeWire::WingPneumatic.set_value(wing_status);
		}
		else {
			wing_status = false;
			spade_status = false;
			RopoDevice::ThreeWire::WingPneumatic.set_value(false);
			RopoDevice::ThreeWire::SpadePneumatic.set_value(false);
		}
	}

	void ChangeTogetherWingStatus(){
		if(wing_status == true) {
			wing_status = false;
			spade_status = false;
			RopoDevice::ThreeWire::WingPneumatic.set_value(false);
			RopoDevice::ThreeWire::SpadePneumatic.set_value(false);
		}
		else {
			wing_status = true;
			spade_status = true;
			RopoDevice::ThreeWire::WingPneumatic.set_value(true);
			RopoDevice::ThreeWire::SpadePneumatic.set_value(true);
		}
	}

	  
	void Intake(){
		RopoDevice::Motors::IntakeMotor.move_velocity(-500);
	}
	void Outtake(){
		RopoDevice::Motors::IntakeMotor.move_velocity(400);
		pros::delay(200);
		RopoDevice::Motors::IntakeMotor.move_voltage(100);
	}
	void IntakerStop(){
		RopoDevice::Motors::IntakeMotor.move_voltage(0);
	}

	bool intaker_status = false;  
	void ChangeIntakerPneumatic(){
		
		intaker_status ^= 1;
		RopoDevice::ThreeWire::IntakerPneumatic.set_value(intaker_status);
		pros::delay(100);
		if(intaker_status)Outtake();
	}

	void TurnAround(){
		RopoDevice::Chassis.AutoRotateRelative(180);
	}

	void GpsUpdate(){
		RopoDevice::gpsAddPosition.GpsUpdate();
	}

	void AutoLift(){
		RopoDevice::Chassis.StartChassisAutoControll();//底盘MoveType设置为AutoMove
		RopoDevice::LiftMotors.Wait2();
		RopoDevice::Chassis.MoveVelocity(-0.2,-0.05);
		while(RopoDevice::Sensors::distance.get() - 0.45 * RopoDevice::Sensors::distance.get_object_velocity() < 117) pros::delay(20);
		ChangeExtern();
		pros::delay(200);
		ChangeExtern();
		pros::delay(1000);
		ChangeExtern();
		pros::delay(100);
		RopoDevice::Chassis.MoveVelocity(0,0);

	}

	void ControllerPrint(){
		while(true) {
			pros::Controller MasterController(pros::E_CONTROLLER_MASTER);
			// Position ******************************************************************************************************
			MasterController.print(0,1,"X: %.2lf Y:%.2lf",(RopoDevice::GetTransformedPosition())[1],(RopoDevice::GetTransformedPosition())[2]);
			pros::delay(50); 
			// MasterController.print(1,1,"degree: %.1lf",RopoDevice::GetPosition()[3]);
			// pros::delay(50); 
			

			//Openmv**********************************************************************************************************
			// MasterController.print(0,1,"Read:%s",RopoDevice::Sensors::My_openMV.IsReading()?"yes":"no");
			// pros::delay(50); 
			// MasterController.print(1,1,"Deg:%.2f",RopoDevice::Sensors::My_openMV.Get_Ball_Deg());
			// pros::delay(50);
			// MasterController.print(2,1,"See:%s",RopoDevice::Sensors::My_openMV.If_See()?"yes":"no");
			// pros::delay(50); 

			//Distance********************************************************************************************************
			MasterController.print(1,1,"Deg:%.2f",RopoDevice::Sensors::distance.get_object_velocity());
			pros::delay(50); 
			MasterController.print(2,1,"Deg:%.2f",RopoDevice::Sensors::distance.get());
			pros::delay(50); 

			//Lifter**********************************************************************************************************
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
	RopoDevice::ThreeWire::IntakerPneumatic.set_value(false);
	RopoDevice::ThreeWire::WingPneumatic.set_value(false);
}

void disabled() {}

void competition_initialize() {}

void autonomous(){
	RopoDevice::ChassisBrake();
	autonomous_1();
	//skill();
}

void opcontrol()
{
	//autonomous();
	//pros::Task *RumbleTask = new pros::Task(ControllerModule::RumbleMe);
	RopoDevice::ChassisBrake();
	pros::Task *PrintTask = new pros::Task(ControllerModule::ControllerPrint);
	pros::Controller MasterController(pros::E_CONTROLLER_MASTER);
	RopoController::ButtonTaskLine ButtonDetectLine(MasterController);
	FloatType opTime = pros::millis();
	FloatType VelocityMax = 1.6;//1.4
	FloatType RopoWcLimit = 6;
	bool ChassisMove = false;
	
	RopoController::AxisValueCast XVelocityInput(MasterController,pros::E_CONTROLLER_ANALOG_LEFT_Y,RopoController::Linear);
	RopoController::AxisValueCast WVelocityInput(MasterController,pros::E_CONTROLLER_ANALOG_RIGHT_X,RopoController::Linear);

	Vector Velocity(RopoMath::ColumnVector,2),ResVelocity;

	MasterController.clear();
	ButtonDetectLine.AddButtonDetect(pros::E_CONTROLLER_DIGITAL_R1, RopoController::Rising , ControllerModule::Intake);
	ButtonDetectLine.AddButtonDetect(pros::E_CONTROLLER_DIGITAL_R1, RopoController::Falling, ControllerModule::IntakerStop);
	ButtonDetectLine.AddButtonDetect(pros::E_CONTROLLER_DIGITAL_X, RopoController::Rising, ControllerModule::ChangeIntakerPneumatic);
	ButtonDetectLine.AddButtonDetect(pros::E_CONTROLLER_DIGITAL_R2, RopoController::Rising , ControllerModule::ChangeLift);
	ButtonDetectLine.AddButtonDetect(pros::E_CONTROLLER_DIGITAL_R2, RopoController::Falling, ControllerModule::ChangeLift);
	//*单点触发
	ButtonDetectLine.AddButtonDetect(pros::E_CONTROLLER_DIGITAL_L1, RopoController::Rising , ControllerModule::ChangeWingStatus);
	ButtonDetectLine.AddButtonDetect(pros::E_CONTROLLER_DIGITAL_L2, RopoController::Rising , ControllerModule::ChangeTogetherWingStatus);
	//*/
	/*按住触发
	ButtonDetectLine.AddButtonDetect(pros::E_CONTROLLER_DIGITAL_L1, RopoController::Rising , ControllerModule::WingPush);
	ButtonDetectLine.AddButtonDetect(pros::E_CONTROLLER_DIGITAL_L1, RopoController::Falling, ControllerModule::WingUnpush);
	ButtonDetectLine.AddButtonDetect(pros::E_CONTROLLER_DIGITAL_L2, RopoController::Rising , ControllerModule::TogetherPush);
	ButtonDetectLine.AddButtonDetect(pros::E_CONTROLLER_DIGITAL_L2, RopoController::Falling, ControllerModule::TogetherUnpush);
	//*/
	//ButtonDetectLine.AddButtonDetect(pros::E_CONTROLLER_DIGITAL_B , RopoController::Rising , ControllerModule::AutoLift);	
	ButtonDetectLine.AddButtonDetect(pros::E_CONTROLLER_DIGITAL_B , RopoController::Rising , ControllerModule::Switch);

	ButtonDetectLine.AddButtonDetect(pros::E_CONTROLLER_DIGITAL_UP   , RopoController::Rising,  autonomous);
	ButtonDetectLine.AddButtonDetect(pros::E_CONTROLLER_DIGITAL_LEFT , RopoController::Rising,  test);
	ButtonDetectLine.AddButtonDetect(pros::E_CONTROLLER_DIGITAL_DOWN , RopoController::Rising,  ControllerModule::GpsUpdate);
	ButtonDetectLine.AddButtonDetect(pros::E_CONTROLLER_DIGITAL_RIGHT, RopoController::Rising, RopoAuto::Auto_Find);

	ButtonDetectLine.Enable();

	while (true) {
		
		FloatType XInput =  XVelocityInput.GetAxisValue();
		FloatType WInput = -WVelocityInput.GetAxisValue();
		FloatType RopoWc = RopoWcLimit-fabs(XInput) * 3.3;			
		FloatType RopoVx = VelocityMax-fabs(WInput) * 0.7;	
		if(opTime - pros::millis() > 55000) VelocityMax = 2.1;
		if (fabs(XInput) <= 0.06 && fabs(WInput) <= 0.06) {
			if(ChassisMove == true){
				RopoDevice::Motors::MoveOpControll(0.0, 0.0);
				ChassisMove = false;
			}
		} 
		else {
			RopoDevice::Chassis.StartChassisOpControll();//底盘MoveType设置为OpMove
			RopoDevice::Motors::MoveOpControll(XInput * RopoVx, WInput * RopoWc);
			ChassisMove = true;
		}
		pros::delay(4);
	}
}

void test(){
	RopoDevice::Chassis.StartChassisAutoControll();//底盘MoveType设置为AutoMove
	// RopoDevice::Chassis.AutoPositionMove(1.0,0,-90);
	// while(!RopoDevice::Chassis.IfArrived()) pros::delay(50);
	RopoAuto::Auto_Find();
	RopoDevice::Chassis.MoveVelocity(-1.0,0);
	RopoDevice::gpsAddPosition.SetUpdateFlag(0);
	RopoDevice::Chassis.AutoPositionMoveBack(1.2,-1.5,-90);
	RopoDevice::ThreeWire::IntakerPneumatic.set_value(true);
	RopoDevice::gpsAddPosition.SetUpdateFlag(10);
	RopoDevice::Chassis.MoveVelocity(1.4,0);
	pros::delay(600);
	RopoDevice::Chassis.MoveVelocity(0.0,0);
	pros::delay(50);
	RopoDevice::Chassis.MoveVelocity(-1.4,0);
	pros::delay(600);
	RopoDevice::Chassis.MoveVelocity(0.0,0);
}

void delay(){
	while(!RopoDevice::Chassis.IfArrived()){
		pros::delay(20);
	}
	return;
}


void autonomous_1(){
	RopoDevice::Chassis.StartChassisAutoControll();//底盘MoveType设置为AutoMove
	// --------- begin ------------


	// ControllerModule::ChangeIntakerPneumatic();
	RopoDevice::ThreeWire::SpadePneumatic.set_value(true);
	pros::delay(30);
	RopoDevice::ThreeWire::SpadePneumatic.set_value(false);
	RopoDevice::Chassis.AutoDirectMove(1.11,0,0);
	while(!RopoDevice::Chassis.IfArrived()){
		pros::delay(20);
	}
	RopoDevice::Chassis.AutoRotateAbs(87);
	while(!RopoDevice::Chassis.IfArrived()){
		pros::delay(20);
	}
	RopoDevice::ThreeWire::SpadePneumatic.set_value(true);
	RopoDevice::Chassis.MoveVelocity(0.2,0);
	pros::delay(230);
	ControllerModule::TogetherPush();
	// RopoDevice::Chassis.AutoDirectMove(1.11,-0.69,1);
	// while(!RopoDevice::Chassis.IfArrived()){
	// 	pros::delay(20);
	// }
	RopoDevice::Chassis.MoveVelocity(-1.3,0);
	pros::delay(850);
	RopoDevice::Position_Motor::MyPosition.Set_XY(1.11, -0.70);
	RopoDevice::Chassis.MoveVelocity(0.35,0);
	pros::delay(120);
	ControllerModule::TogetherUnpush();
	pros::delay(130);
	RopoDevice::Chassis.MoveVelocity(0.3,0);
	pros::delay(150);
	RopoDevice::Chassis.AutoPositionMove(-0.01,0.38);
	while(!RopoDevice::Chassis.IfArrived()){
		pros::delay(20);
	}

//-----------------------------------------------------
	
	RopoDevice::Chassis.AutoRotateAbs(135);
	while(!RopoDevice::Chassis.IfArrived()){
		pros::delay(20);
	}
	RopoDevice::Chassis.MoveVelocity(0.35,0);
	pros::delay(150);
	RopoDevice::Chassis.MoveVelocity(0,0);
	ControllerModule::Lift();
	pros::delay(700);
	for (int i = 1; i <= 2; i++) {

		// RopoDevice::Chassis.AutoRotateAbs(182);
		// while(!RopoDevice::Chassis.IfArrived()){
		// 	pros::delay(20);
		// }

		RopoDevice::Chassis.MoveVelocity(0,2.3);
		pros::delay(480);
		RopoDevice::Chassis.MoveVelocity(0,0);
		RopoDevice::Chassis.AutoRotateAbs(135);
		while(!RopoDevice::Chassis.IfArrived()){
			pros::delay(20);
		}
		RopoDevice::Chassis.MoveVelocity(0.35,0);
		pros::delay(250);
		RopoDevice::Chassis.MoveVelocity(0,0);
		pros::delay(20);
	}

//-------------------------------------------------

	ControllerModule::Hide();
	pros::delay(200);
	RopoDevice::Chassis.MoveVelocity(-0.4,0);
	pros::delay(400);
	RopoDevice::Chassis.MoveVelocity(0,0);
	RopoDevice::Chassis.AutoRotateAbs(45);
	delay();
	RopoDevice::Chassis.MoveVelocity(-0.50,0);
	pros::delay(600);
	RopoDevice::Chassis.MoveVelocity(0,0);
	RopoDevice::Chassis.AutoRotateAbs(86);
	while(!RopoDevice::Chassis.IfArrived()){
		pros::delay(20);
	}
	pros::delay(100);

//-----------------------------------------------
	// RopoDevice::Chassis.AutoRotateAbs(-90);
	// // delay();
	// RopoDevice::Position_Motor::MyPosition.Set_XY(0, 0);
	// pros::delay(300);
	// RopoDevice::Chassis.MoveVelocity(0.5,0);
	// pros::delay(900);
	// FloatType radspeed = -0.5;
	// FloatType sign_flag = -1;
	// while (RopoDevice::Position_Motor::MyPosition.Get_Y() > -2.0) {
	// 	if (fabs(RopoDevice::Position_Motor::MyPosition.Get_X()) > 0.03 && RopoMath::Sign(RopoDevice::Position_Motor::MyPosition.Get_X()) == sign_flag) {
	// 		radspeed = radspeed * -1;
	// 		sign_flag = sign_flag * -1;
	// 		RopoDevice::Chassis.MoveVelocity(0,0);
	// 		pros::delay(1000);
	// 	}
	// 	RopoDevice::Chassis.MoveVelocity(0.5,radspeed);
	// 	pros::delay(10);
	// }
	// RopoDevice::Chassis.MoveVelocity(0,0);
	// RopoDevice::Chassis.AutoRotateAbs(-90);

//----------------------------------------------------------------

	ControllerModule::TogetherPush();
	RopoDevice::Chassis.MoveVelocity(-0.8,0);
	pros::delay(2410);
	RopoDevice::Chassis.MoveVelocity(0,0);
	// RopoDevice::Chassis.AutoRotateAbs(90);
	RopoDevice::Chassis.MoveVelocity(-0.6,0.95);
	pros::delay(2050);
	ControllerModule::TogetherUnpush();
	RopoDevice::Chassis.MoveVelocity(0,0);
	pros::delay(100);

	// RopoDevice::Chassis.MoveVelocity(0.5,0);
	// pros::delay(400);
	// RopoDevice::Chassis.MoveVelocity(-0.8,0);
	// pros::delay(700);
	// RopoDevice::Chassis.MoveVelocity(0.2,0);
	// pros::delay(200);

//-------------------------------------------------------

	// RopoDevice::Chassis.AutoRotateAbs(180);
	RopoDevice::Chassis.MoveVelocity(0.65,0.1);
	pros::delay(900);
	RopoDevice::Chassis.MoveVelocity(0,0);
	RopoDevice::gpsAddPosition.SetUpdateFlag(0);
	pros::delay(30);
	RopoDevice::Position_Motor::MyPosition.Set_XY(0, 0);
	ControllerModule::Lift();
	pros::delay(850);
	RopoDevice::Chassis.MoveVelocity(-0.4,0);
	pros::delay(800);
	ControllerModule::Intake();
	RopoDevice::Chassis.MoveVelocity(0,0);
	ControllerModule::Hide();
	RopoDevice::Chassis.MoveVelocity(0.3,0);
	pros::delay(300);
	RopoDevice::Chassis.MoveVelocity(0,0);
	RopoDevice::Chassis.AutoPositionMove(0.6,-0.23);
	delay();
	RopoDevice::Chassis.AutoRotateAbs(0);
	delay();
	RopoDevice::Chassis.MoveVelocity(0.6,0);
	pros::delay(500);
	RopoDevice::Chassis.MoveVelocity(0,0);

//-------------------------------------------------------------

	// RopoDevice::Chassis.AutoRotateAbs(180);
	// RopoDevice::Chassis.MoveVelocity(0.5,-0.93);
	// pros::delay(1450);
	// RopoDevice::Chassis.MoveVelocity(0,0);
	// pros::delay(3000);
	// // pros::delay(3000);
	// // RopoDevice::Chassis.AutoRotateAbs(180);
	// // delay();
	// // pros::delay(2000);
	// RopoDevice::Chassis.MoveVelocity(0.6,0.2);
	// pros::delay(2000);

	// RopoDevice::Chassis.MoveVelocity(0,0);
	// ControllerModule::Lift();
	// pros::delay(600);
	// RopoDevice::Position_Motor::MyPosition.Set_XY(0, 0);
	// RopoDevice::Chassis.MoveVelocity(-0.3,0);
	// pros::delay(700);
	// ControllerModule::Intake();
	// RopoDevice::Chassis.MoveVelocity(0.2,0);
	// pros::delay(400);
	// RopoDevice::Chassis.AutoPositionMove(0.4,-0.1);
	// RopoDevice::Chassis.AutoRotateAbs(0);
	// ControllerModule::Outtake();
	// RopoDevice::Chassis.MoveVelocity(0.6,0);
	// pros::delay(700);
	// RopoDevice::Chassis.MoveVelocity(0,0);










}

void skill(){
	RopoDevice::Chassis.StartChassisAutoControll();//底盘MoveType设置为AutoMove
	// --------- begin ------------
}
