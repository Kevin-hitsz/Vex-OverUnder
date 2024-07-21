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
void autonomous_C1b();
void autonomous_C2();

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

	void StartGps(){
		RopoDevice::gpsAddPosition.SetUpdateFlag(3);//开gps
	}

	void StopGps(){
		RopoDevice::gpsAddPosition.SetUpdateFlag(0);//开gps
	}

	bool wing_status = false;
	bool spade_status = true;
	void TogetherPush(){
		wing_status = true;
		spade_status = false;
		RopoDevice::ThreeWire::WingPneumatic.set_value(true);
		RopoDevice::ThreeWire::SpadePneumatic.set_value(false);
	}
	void TogetherUnpush(){
		wing_status = false;
		spade_status = true;
		RopoDevice::ThreeWire::WingPneumatic.set_value(false);
		RopoDevice::ThreeWire::SpadePneumatic.set_value(true);
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
		if(spade_status == true) {
			wing_status ^= 1;
			RopoDevice::ThreeWire::WingPneumatic.set_value(wing_status);
		}
		else {
			wing_status = false;
			spade_status = true;
			RopoDevice::ThreeWire::WingPneumatic.set_value(false);
			RopoDevice::ThreeWire::SpadePneumatic.set_value(true);
		}
	}

	void ChangeTogetherWingStatus(){
		if(wing_status == true) {
			wing_status = false;
			spade_status = true;
			RopoDevice::ThreeWire::WingPneumatic.set_value(false);
			RopoDevice::ThreeWire::SpadePneumatic.set_value(true);
		}
		else {
			wing_status = true;
			spade_status = false;
			RopoDevice::ThreeWire::WingPneumatic.set_value(true);
			RopoDevice::ThreeWire::SpadePneumatic.set_value(false);
		}
	}

	bool intaker_status = false;  
	void Intake(){
		RopoDevice::Motors::IntakeMotor.move_velocity(-500);
		if(intaker_status == true){
			intaker_status ^= 1;
			RopoDevice::ThreeWire::IntakerPneumatic.set_value(intaker_status);
		}
	}
	void Outtake(){
		RopoDevice::Motors::IntakeMotor.move_velocity(400);
		pros::delay(400);
		RopoDevice::Motors::IntakeMotor.move_voltage(2000);
	}
	void IntakerStop(){
		RopoDevice::Motors::IntakeMotor.move_voltage(0);
	}

	  
	void ChangeIntakerPneumatic(){
		
		intaker_status ^= 1;
		RopoDevice::ThreeWire::IntakerPneumatic.set_value(intaker_status);
		pros::delay(100);
		if(intaker_status)Outtake();
	}

	void ChangeIntakerPneumatic0(){
		
		intaker_status ^= 1;
		RopoDevice::ThreeWire::IntakerPneumatic.set_value(intaker_status);
		if(intaker_status){
			RopoDevice::Motors::IntakeMotor.move_velocity(400);
		}
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

	void ChangeLift1(){
		if (catch_1 == 1) {
			Hide();
		} else {
			Hold();
			Intake();
		}
	}


	void TurnAround(){
		RopoDevice::Chassis.AutoRotateRelative(180);
	}

	void GpsUpdate(){
		RopoDevice::gpsAddPosition.GpsUpdate();
	}

	void AutoLift(){
		RopoDevice::LiftMotors.Wait2();
		RopoDevice::Chassis.StartChassisAutoControll();//底盘MoveType设置为AutoMove
		RopoDevice::ChassisBrake();
		RopoDevice::ThreeWire::IntakerPneumatic.set_value(true);
		//RopoDevice::Chassis.AutoRotateAbs(-90);
		//pros::delay(500);
		RopoDevice::Chassis.MoveVelocity(-0.3,-0.00);
		while(RopoDevice::Sensors::distance.get()  > 150 || RopoDevice::Sensors::distance.get() < 10 || RopoDevice::Sensors::distance.get() == PROS_ERR) pros::delay(20);
		RopoDevice::Chassis.MoveVelocity(0,0);
		pros::delay(300);
		ChangeExtern();
		pros::delay(200);
		ChangeExtern();
		pros::delay(300);
		RopoDevice::Chassis.MoveVelocity(-0.4,-0.00);
		pros::delay(360);
		RopoDevice::Chassis.MoveVelocity(-0.2,-0.00);
		pros::delay(200);
		
		RopoDevice::Chassis.MoveVelocity(0,0);
		pros::delay(300);
		RopoDevice::Chassis.MoveVelocity(-0.0,-1.00);
		pros::delay(110);
		RopoDevice::Chassis.MoveVelocity(0,0);
		ChangeExtern();
		

	
	}

	void ControllerPrint(){

		while(true) {
			pros::Controller MasterController(pros::E_CONTROLLER_MASTER);
			// Position ******************************************************************************************************
			MasterController.print(0,1,"X: %.2lf Y:%.2lf   ",(RopoDevice::GetTransformedPosition())[1],(RopoDevice::GetTransformedPosition())[2]);
			pros::delay(50); 
			MasterController.print(1,1,"degree: %.3lf    ",RopoDevice::GetPosition()[3]);
			pros::delay(50); 
			MasterController.print(2,1,"Deg:%.2f",RopoDevice::Sensors::My_openMV.Get_Ball_Deg());
			pros::delay(50);
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
	RopoDevice::ThreeWire::SpadePneumatic.set_value(true);
}

void disabled() {}

void competition_initialize() {}

void autonomous(){
	RopoDevice::ChassisBrake();
	autonomous_C1b();
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
	FloatType VelocityMax = 1.77;//1.4
	FloatType RopoWcLimit = 9;
	bool ChassisMove = false;
	
	RopoController::AxisValueCast XVelocityInput(MasterController,pros::E_CONTROLLER_ANALOG_LEFT_Y,RopoController::Linear);
	RopoController::AxisValueCast WVelocityInput(MasterController,pros::E_CONTROLLER_ANALOG_RIGHT_X,RopoController::Exp);

	Vector Velocity(RopoMath::ColumnVector,2),ResVelocity;

	MasterController.clear();
	ButtonDetectLine.AddButtonDetect(pros::E_CONTROLLER_DIGITAL_R1, RopoController::Rising , ControllerModule::Intake);
	ButtonDetectLine.AddButtonDetect(pros::E_CONTROLLER_DIGITAL_R1, RopoController::Falling, ControllerModule::IntakerStop);
	ButtonDetectLine.AddButtonDetect(pros::E_CONTROLLER_DIGITAL_X, RopoController::Rising, ControllerModule::ChangeIntakerPneumatic);
	ButtonDetectLine.AddButtonDetect(pros::E_CONTROLLER_DIGITAL_R2, RopoController::Rising , ControllerModule::ChangeLift1);
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
	ButtonDetectLine.AddButtonDetect(pros::E_CONTROLLER_DIGITAL_Y , RopoController::Rising , ControllerModule::AutoLift);	
	ButtonDetectLine.AddButtonDetect(pros::E_CONTROLLER_DIGITAL_B , RopoController::Rising , ControllerModule::Switch);
	ButtonDetectLine.AddButtonDetect(pros::E_CONTROLLER_DIGITAL_A , RopoController::Rising , RopoDevice::ChassisHold);
	
	ButtonDetectLine.AddButtonDetect(pros::E_CONTROLLER_DIGITAL_UP   , RopoController::Rising,  autonomous);
	ButtonDetectLine.AddButtonDetect(pros::E_CONTROLLER_DIGITAL_LEFT , RopoController::Rising,  test);
	ButtonDetectLine.AddButtonDetect(pros::E_CONTROLLER_DIGITAL_DOWN , RopoController::Rising,  autonomous_C1b);
	ButtonDetectLine.AddButtonDetect(pros::E_CONTROLLER_DIGITAL_RIGHT , RopoController::Rising,  ControllerModule::StartGps);
	ButtonDetectLine.AddButtonDetect(pros::E_CONTROLLER_DIGITAL_RIGHT , RopoController::Falling,  ControllerModule::StopGps);
	ButtonDetectLine.Enable();
	while (true) {
		
		FloatType XInput =  XVelocityInput.GetAxisValue();
		FloatType WInput = -WVelocityInput.GetAxisValue();
		FloatType RopoWc = RopoWcLimit-fabs(XInput) * 2.7;			
		FloatType RopoVx = VelocityMax-fabs(WInput) * 0.47;	
		if(opTime - pros::millis() > 55000) {
			VelocityMax = 1.77;
			RopoWcLimit = 8.5;
			RopoDevice::ChassisBrake();
		}
		if (fabs(XInput) <= 0.06 && fabs(WInput) <= 0.06) {
			if(ChassisMove == true){
				RopoDevice::Chassis.StartChassisOpControll();//底盘MoveType设置为OpMove
				RopoDevice::Motors::MoveOpControll(0.0, 0.0);
				ChassisMove = false;
			}
		} 
		else {
			RopoDevice::Chassis.StartChassisOpControll();//底盘MoveType设置为OpMove
			RopoDevice::Motors::MoveOpControll(XInput * RopoVx * 0.9, WInput * RopoWc * 0.9);

			ChassisMove = true;
		}
		pros::delay(2);
	}
}

void test(){
	RopoDevice::Chassis.StartChassisAutoControll();//底盘MoveType设置为AutoMove
	RopoDevice::ChassisBrake();
	RopoDevice::Chassis.AutoRotateAbs(90);
	while(!RopoDevice::Chassis.IfDegArrived() )pros::delay(20);
	RopoDevice::Chassis.AutoRotateAbs(-90);
	while(!RopoDevice::Chassis.IfDegArrived() )pros::delay(20);
	/*
	RopoDevice::Chassis.MoveVelocity(0.0,0);
	ControllerModule::Intake();
	RopoDevice::gpsAddPosition.SetUpdateFlag(10);//开启gps
	pros::delay(200);
	RopoDevice::gpsAddPosition.SetUpdateFlag(10);
	RopoAuto::Auto_Find();
	RopoDevice::Chassis.MoveVelocity(-1.0,0);
	pros::delay(100);
	RopoDevice::gpsAddPosition.SetUpdateFlag(0);
	if( !(RopoDevice::GetTransformedPosition()[2] > -1.5 && RopoDevice::GetTransformedPosition()[2] < -1.2)){
		RopoDevice::Chassis.AutoPositionMoveBack(-1.60,-1.35,-180);
	}
	else {
		RopoDevice::Chassis.AutoRotateAbs(177);
		pros::delay(800);
	}
	//ControllerModule::ChangeIntakerPneumatic();
	RopoDevice::gpsAddPosition.SetUpdateFlag(10);
	RopoDevice::Chassis.MoveVelocity(1.5,0);
	pros::delay(650);
	RopoDevice::Chassis.MoveVelocity(-1.1,0);
	pros::delay(700);
	//RopoDevice::Chassis.AutoRotateAbs(00);
	RopoDevice::Chassis.MoveVelocity(0.0,0);
	//ControllerModule::ChangeIntakerPneumatic();
	*/

}

void delay(){
	while(!RopoDevice::Chassis.IfArrived()){pros::delay(20);}
	return;
}

void delayDeg(){
	double aa = pros::millis();
	while(!RopoDevice::Chassis.IfDegArrived() && aa - pros::millis() < 2500)pros::delay(20);
}

void delayL(){
	while(!RopoDevice::Chassis.IfDisArrived()){pros::delay(20);}
	return;
}

void autonomous_C1b(){
	RopoDevice::ChassisBrake();
	RopoDevice::Chassis.StartChassisAutoControll();//底盘MoveType设置为AutoMove
	RopoDevice::gpsAddPosition.SetUpdateFlag(0);//关闭gps
//start 勾球
//	导己方四球
	ControllerModule::ChangeLift();
	pros::delay(750);
	RopoDevice::Motors::IntakeMotor.move_voltage(1000);
	for(int i = 0; i < 13; i++){
		RopoDevice::Chassis.MoveVelocity(0.0,4);
		pros::delay(410);
		RopoDevice::Chassis.MoveVelocity(0.0,0.0);
		pros::delay(50);
		RopoDevice::Chassis.AutoRotateAbs(-5);
		pros::delay(600);
		RopoDevice::Chassis.MoveVelocity(0.0,0.0);
		pros::delay(50);
	}
	RopoDevice::Position_Motor::MyPosition.Set_XY(0, 0.0);
	RopoDevice::Chassis.AutoRotateAbs(50);
	pros::delay(550);
	ControllerModule::ChangeLift();
	RopoDevice::Chassis.MoveVelocity(0,0);
	pros::delay(300);
//推球	

	RopoDevice::Chassis.MoveVelocity(0,5);
	pros::delay(500);
	RopoDevice::Chassis.AutoRotateAbs(-91);
	pros::delay(600);
	ControllerModule::WingPush();
	RopoDevice::Chassis.MoveVelocity(-0.6,-0.1);
	pros::delay(660);
	RopoDevice::Chassis.MoveVelocity(-0.6,1.3);
	pros::delay(230);
	RopoDevice::Chassis.MoveVelocity(0,0);
	pros::delay(100);
	ControllerModule::WingUnpush();
	RopoDevice::Chassis.AutoRotateAbs(-53);
	pros::delay(450);
	
	RopoDevice::Chassis.MoveVelocity(-0.8,-0.02);
	pros::delay(600);
	ControllerModule::WingPush();
	RopoDevice::Chassis.MoveVelocity(0,0);
	pros::delay(200);
	RopoDevice::Chassis.MoveVelocity(-1.0,0.03);
	pros::delay(830);
	RopoDevice::Chassis.MoveVelocity(-0.8,0.35);
	pros::delay(900);
	// RopoDevice::Chassis.AutoDirectMove(-1.32,1.71,1);
	// pros::delay(300);

	RopoDevice::gpsAddPosition.SetUpdateFlag(1);


	RopoDevice::Chassis.AutoRotateAbs(-38);
	pros::delay(500);


	RopoDevice::Chassis.MoveVelocity(-0.5,0.9);
	pros::delay(750);



	RopoDevice::Chassis.MoveVelocity(-0.5,0);
	pros::delay(380);

	//弧线推球
	
	RopoDevice::Chassis.MoveVelocity(-0.6,1.2);
	pros::delay(700);
	ControllerModule::WingUnpush();
	RopoDevice::Chassis.AutoRotateAbs(45);
	pros::delay(400);


	
	pros::delay(100);
	RopoDevice::Chassis.MoveVelocity(0.4,0);
	pros::delay(230);
	RopoDevice::Chassis.MoveVelocity(0.0,0);
	pros::delay(20);
	//推进网
	RopoDevice::Chassis.MoveVelocity(-1.5,0.0);
	pros::delay(380);

	RopoDevice::Chassis.MoveVelocity(0.6,0);
	pros::delay(600);
	RopoDevice::Chassis.AutoRotateAbs(45);
	pros::delay(400);
	RopoDevice::Chassis.MoveVelocity(-1.5,0.0);
	pros::delay(350);

	RopoDevice::Chassis.MoveVelocity(0.5,0);
	pros::delay(700);
	RopoDevice::Chassis.AutoRotateAbs(-55);
	pros::delay(650);
	RopoDevice::Position_Motor::MyPosition.Set_XY(-2.33,1.65);
	RopoDevice::gpsAddPosition.SetUpdateFlag(1);
	pros::delay(500);
	RopoDevice::gpsAddPosition.SetUpdateFlag(0);
	RopoDevice::Chassis.AutoRotateAbs(0);
	pros::delay(600);
	RopoDevice::Chassis.AutoPositionMove(-1.56,1.64,10000,4000);
	RopoDevice::Chassis.AutoRotateAbs(-38);
	pros::delay(700);
	RopoDevice::Chassis.AutoDirectMove(-0.3,0.3,0);
	pros::delay(300);
	delayL();
	RopoDevice::Chassis.MoveVelocity(0.8,-3);
	pros::delay(450);
	RopoDevice::Chassis.MoveVelocity(0.0,0.0);
	pros::delay(100);
	RopoDevice::Chassis.AutoRotateAbs(0);
	pros::delay(800);
	//2
	ControllerModule::ChangeLift();
	pros::delay(750);
	RopoDevice::Motors::IntakeMotor.move_voltage(1000);
	for(int i = 0; i < 11; i++){
		RopoDevice::Chassis.MoveVelocity(0.0,4);
		pros::delay(410);
		RopoDevice::Chassis.MoveVelocity(0.0,0.0);
		pros::delay(50);
		RopoDevice::Chassis.AutoRotateAbs(-5);
		pros::delay(600);
		RopoDevice::Chassis.MoveVelocity(0.0,0.0);
		pros::delay(50);
	}
	RopoDevice::Position_Motor::MyPosition.Set_XY(0, 0.0);
	RopoDevice::Chassis.AutoRotateAbs(50);
	pros::delay(550);
	ControllerModule::ChangeLift();
	RopoDevice::Chassis.MoveVelocity(0,0);
	pros::delay(300);
//推球	

	RopoDevice::Chassis.MoveVelocity(0,5);
	pros::delay(500);
	RopoDevice::Chassis.AutoRotateAbs(-91);
	pros::delay(600);
	ControllerModule::WingPush();
	RopoDevice::Chassis.MoveVelocity(-0.6,-0.1);
	pros::delay(660);
	RopoDevice::Chassis.MoveVelocity(-0.6,1.3);
	pros::delay(230);
	RopoDevice::Chassis.MoveVelocity(0,0);
	pros::delay(100);
	ControllerModule::WingUnpush();
	RopoDevice::Chassis.AutoRotateAbs(-51);
	pros::delay(450);
	
	RopoDevice::Chassis.MoveVelocity(-0.8,-0.02);
	pros::delay(600);
	ControllerModule::WingPush();
	RopoDevice::Chassis.MoveVelocity(0,0);
	pros::delay(200);
	RopoDevice::Chassis.MoveVelocity(-1.0,0.03);
	pros::delay(830);
	RopoDevice::Chassis.MoveVelocity(-0.8,0.35);
	pros::delay(900);
	// RopoDevice::Chassis.AutoDirectMove(-1.32,1.71,1);
	// pros::delay(300);

	RopoDevice::gpsAddPosition.SetUpdateFlag(1);


	RopoDevice::Chassis.AutoRotateAbs(-38);
	pros::delay(500);


	RopoDevice::Chassis.MoveVelocity(-0.5,0.9);
	pros::delay(750);



	RopoDevice::Chassis.MoveVelocity(-0.5,0);
	pros::delay(380);

	//弧线推球
	
	RopoDevice::Chassis.MoveVelocity(-0.6,1.2);
	pros::delay(700);
	ControllerModule::WingUnpush();
	RopoDevice::Chassis.AutoRotateAbs(45);
	pros::delay(400);


	
	pros::delay(100);
	RopoDevice::Chassis.MoveVelocity(0.4,0);
	pros::delay(230);
	RopoDevice::Chassis.MoveVelocity(0.0,0);
	pros::delay(20);
	//推进网
	RopoDevice::Chassis.MoveVelocity(-1.5,0.0);
	pros::delay(380);

	RopoDevice::Chassis.MoveVelocity(0.6,0);
	pros::delay(600);
	RopoDevice::Chassis.AutoRotateAbs(45);
	pros::delay(400);
	RopoDevice::Chassis.MoveVelocity(-1.5,0.0);
	pros::delay(500);
	RopoDevice::Chassis.MoveVelocity(0.6,0);
	pros::delay(700);
	//结束
	ControllerModule::IntakerStop();
	RopoDevice::Chassis.MoveVelocity(0.0,0);
	pros::delay(50);
}


void skill(){
	RopoDevice::Chassis.StartChassisAutoControll();//底盘MoveType设置为AutoMove
	// --------- begin ------------
}
