#include "main.h"
#include "RopoApi.hpp"
#include "RopoController.hpp"
#include "RopoControllerModule.hpp"
#include "RopoDevice.hpp"
#include "RopoGpsAddPosition.hpp"
#include "RopoMath/Misc.hpp"
#include "RopoPosition.hpp"
#include "RopoAutonomous.hpp"
#include "pros/misc.h"
#include "pros/rtos.hpp"
#include <algorithm>
#include <cmath>

namespace ControllerModule{

	void BoolSwitch(void * Parameter){
		bool *p = static_cast<bool *>(Parameter);
		(*p) ^= 1;
	}

	void Push(){
		RopoDevice::ThreeWire::ExternPneumatic.set_value(true);
	}

	void Pull(){
		RopoDevice::ThreeWire::ExternPneumatic.set_value(false);
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
		FloatType aa = pros::millis();  //.
		while(pros::millis()-aa<45000){
			pros::delay(100);
		}
		MasterController1.rumble("-.-.-");
		while(pros::millis()-aa<65000){
			pros::delay(100);
		}
		MasterController1.rumble("-.-.-");
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

	void ControllerPrint(){
		while(true) {
			pros::Controller MasterController(pros::E_CONTROLLER_MASTER);
			MasterController.print(0,1,"degree: %.1lf",-RopoDevice::Sensors::Inertial.get_yaw());
			pros::delay(10); 
			MasterController.print(1,1,"X: %.2lf Y:%.2lf",RopoDevice::Position_Motor::MyPosition.Get_X(),RopoDevice::Position_Motor::MyPosition.Get_Y());
			pros::delay(10); 
			MasterController.print(2,1,"%.2lf  %d",RopoDevice::LiftMotors.GetLifterPosition(), RopoDevice::LiftMotors.GetLifterStatus());
			pros::delay(10);
		}
	}
}

void initialize() {
	pros::lcd::initialize();
	pros::delay(50);
	RopoDevice::DeviceInit();
	RopoDevice::MotorsInit();
	RopoDevice::Position_Motor::MyPosition.initial();
	RopoDevice::ThreeWire::IntakerPneumatic1.set_value(false);
	RopoDevice::ThreeWire::IntakerPneumatic2.set_value(false);
	RopoDevice::ThreeWire::RightWingPneumatic.set_value(false);
	RopoDevice::ThreeWire::LeftWingPneumatic.set_value(false);
}

void disabled() {}

void competition_initialize() {}

void autonomous(){
	RopoAutonomous::FinalKnockoutMatch::Final_KnockoutMatch(4);
	// RopoAutonomous::skill_new1();
}

void opcontrol()
{
	//autonomous();
	//pros::Task *RumbleTask = new pros::Task(ControllerModule::RumbleMe);
	RopoDevice::ChassisCoast();
	pros::Task *PrintTask = new pros::Task(ControllerModule::ControllerPrint);
	pros::Controller MasterController(pros::E_CONTROLLER_MASTER);
	RopoController::ButtonTaskLine ButtonDetectLine(MasterController);
	FloatType opTime = pros::millis();
	FloatType RopoVcLimit = 2.0;//1.4
	FloatType RopoWcLimit = 7.5;
	FloatType RopoVcRetrainAmp = 0.75;
	FloatType RopoWcRetrainAmp = 3.1;
	bool ChassisMove = false;
	
	RopoController::AxisValueCast XVelocityInput(MasterController, pros::E_CONTROLLER_ANALOG_LEFT_Y, RopoController::Linear);
	RopoController::AxisValueCast WVelocityInput(MasterController, pros::E_CONTROLLER_ANALOG_RIGHT_X, RopoController::Linear);

	Vector Velocity(RopoMath::ColumnVector, 2), ResVelocity;

	MasterController.clear();
	ButtonDetectLine.AddButtonDetect(pros::E_CONTROLLER_DIGITAL_R1, RopoController::Rising , ControllerModule::Intake);
	ButtonDetectLine.AddButtonDetect(pros::E_CONTROLLER_DIGITAL_R1, RopoController::Falling, ControllerModule::IntakerStop);

	ButtonDetectLine.AddButtonDetect(pros::E_CONTROLLER_DIGITAL_B, RopoController::Rising , ControllerModule::Outtake);
	ButtonDetectLine.AddButtonDetect(pros::E_CONTROLLER_DIGITAL_B, RopoController::Falling, ControllerModule::IntakerStop);
	
	ButtonDetectLine.AddButtonDetect(pros::E_CONTROLLER_DIGITAL_R2, RopoController::Rising , ControllerModule::ChangeIntakerPneumatic);
	ButtonDetectLine.AddButtonDetect(pros::E_CONTROLLER_DIGITAL_L2, RopoController::Rising , ControllerModule::ChangeLeftWingPush);
	ButtonDetectLine.AddButtonDetect(pros::E_CONTROLLER_DIGITAL_L1, RopoController::Rising , ControllerModule::ChangeRightWingPush);

	ButtonDetectLine.AddButtonDetect(pros::E_CONTROLLER_DIGITAL_X, RopoController::Rising , ControllerModule::Hang);
	ButtonDetectLine.AddButtonDetect(pros::E_CONTROLLER_DIGITAL_Y, RopoController::Rising , ControllerModule::Bar);

	// ButtonDetectLine.AddButtonDetect(pros::E_CONTROLLER_DIGITAL_A, RopoController::Rising , RopoAutonomous::autonomous_qualify);
	// ButtonDetectLine.AddButtonDetect(pros::E_CONTROLLER_DIGITAL_RIGHT , RopoController::Rising,  ControllerModule::GpsUpdate);
	// ButtonDetectLine.AddButtonDetect(pros::E_CONTROLLER_DIGITAL_LEFT , RopoController::Rising, RopoAutonomous::skill_new2);
	// ButtonDetectLine.AddButtonDetect(pros::E_CONTROLLER_DIGITAL_UP , RopoController::Rising, RopoAutonomous::FinalKnockoutMatch::Test_Final_KnockoutMatch);
	// ButtonDetectLine.AddButtonDetect(pros::E_CONTROLLER_DIGITAL_DOWN , RopoController::Rising, RopoAutonomous::skill_new1);
	/*end*/

	ButtonDetectLine.Enable();
	ControllerModule::BarRecover();
	ControllerModule::bar_status = 0;
	ControllerModule::ChangeIntakerPneumatic();


	while (true) {
		
		FloatType XInput =  XVelocityInput.GetAxisValue();
		FloatType WInput = -WVelocityInput.GetAxisValue();
		FloatType RopoVx = XInput * (RopoVcLimit - fabs(WInput) * RopoVcRetrainAmp);
		FloatType RopoWc = WInput * (RopoWcLimit - fabs(XInput) * RopoWcRetrainAmp);
		if (fabs(XInput) <= 0.06 && fabs(WInput) <= 0.06) {
			if(ChassisMove == true){
				RopoDevice::Motors::MoveOpControll(0.0, 0.0);
				ChassisMove = false;
			}
		} 
		else {
			RopoDevice::Chassis.StartChassisOpControll();//底盘MoveType设置为OpMove
			RopoDevice::Motors::MoveOpControll(RopoVx, RopoWc);
			ChassisMove = true;
		}
		pros::delay(4);
	}
}