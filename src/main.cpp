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
void autonomous_c3();
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

	bool leftwing_status = false;
	bool rightwing_status = false;
	bool spade_status = false;
	void ChangeLeftWingPush(){
		leftwing_status = true;
		RopoDevice::ThreeWire::LeftWingPneumatic.set_value(leftwing_status);
	}
	void ChangeRightWingPush(){
		rightwing_status = true;
		RopoDevice::ThreeWire::RightWingPneumatic.set_value(rightwing_status);
	}
	void LaunchHang(){
		RopoDevice::ThreeWire::IntakerPneumatic1.set_value(true);
		RopoDevice::ThreeWire::IntakerPneumatic2.set_value(true);
		pros::delay(500);
		RopoDevice::ThreeWire::ExternPneumatic.set_value(true);
	}
	void Hang(){
		RopoDevice::ThreeWire::HangPneumatic.set_value(true);
	}
	void BarRecover(){
		RopoDevice::ThreeWire::BarPneumatic.set_value(false);
	}
	void BarExtend(){
		RopoDevice::ThreeWire::BarPneumatic.set_value(true);
	}

	  
	void Intake(){
		RopoDevice::Motors::LeftIntakeMotor.move_voltage(-550.0 / 600.0 * 12000.0);
		RopoDevice::Motors::RightIntakeMotor.move_voltage(-550.0 / 600.0 * 12000.0);
	}
	void Outtake(){
		RopoDevice::Motors::LeftIntakeMotor.move_velocity(400);
		RopoDevice::Motors::RightIntakeMotor.move_velocity(400);
		pros::delay(400);
		RopoDevice::Motors::LeftIntakeMotor.move_voltage(2000);
		RopoDevice::Motors::RightIntakeMotor.move_voltage(2000);
	}
	void IntakerStop(){
		RopoDevice::Motors::LeftIntakeMotor.move_voltage(0);
		RopoDevice::Motors::RightIntakeMotor.move_voltage(0);
	}

	bool intaker_status = false;  
	void ChangeIntakerPneumatic(){
		
		intaker_status ^= 1;
		RopoDevice::ThreeWire::IntakerPneumatic1.set_value(intaker_status);
		RopoDevice::ThreeWire::IntakerPneumatic2.set_value(intaker_status);
		pros::delay(100);
		//if(intaker_status)Outtake();
	}

	void TurnAround(){
		RopoDevice::Chassis.AutoRotateRelative(180);
	}

	void GpsUpdate(){
		RopoDevice::gpsAddPosition.GpsUpdate();
	}

	

	void ControllerPrint(){
		while(true) {
			pros::Controller MasterController(pros::E_CONTROLLER_MASTER);
			// Position ******************************************************************************************************
			MasterController.print(0,1,"X: %.2lf Y:%.2lf   ",(RopoDevice::GetTransformedPosition())[1],(RopoDevice::GetTransformedPosition())[2]);
			pros::delay(50); 
			MasterController.print(1,1,"degree: %.3lf    ",RopoDevice::GetPosition()[3]);
			pros::delay(50); 
			

			//Openmv**********************************************************************************************************
			// MasterController.print(0,1,"Read:%s",RopoDevice::Sensors::My_openMV.IsReading()?"yes":"no");
			// pros::delay(50); 
			// MasterController.print(1,1,"Deg:%.2f",RopoDevice::Sensors::My_openMV.Get_Ball_Deg());
			// pros::delay(50);
			MasterController.print(2,1,"See:%s    ",RopoDevice::Sensors::My_openMV.If_See()?"yes":"no");
			pros::delay(50); 

			//Distance********************************************************************************************************
			// MasterController.print(1,1,"Vb:%.2f    ",RopoDevice::Sensors::distance.get_object_velocity());
			// pros::delay(50); 
			// MasterController.print(2,1,"Dis:%d    ",RopoDevice::Sensors::distance.get());
			// pros::delay(50); 

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
	RopoDevice::ThreeWire::IntakerPneumatic1.set_value(false);
	RopoDevice::ThreeWire::IntakerPneumatic2.set_value(false);
	RopoDevice::ThreeWire::RightWingPneumatic.set_value(false);
	RopoDevice::ThreeWire::LeftWingPneumatic.set_value(false);
}

void disabled() {}

void competition_initialize() {}

void autonomous(){
	//RopoDevice::ChassisBrake();
	//autonomous_c3();
	//skill();
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
	
	ButtonDetectLine.AddButtonDetect(pros::E_CONTROLLER_DIGITAL_R2, RopoController::Rising , ControllerModule::ChangeIntakerPneumatic);
	ButtonDetectLine.AddButtonDetect(pros::E_CONTROLLER_DIGITAL_L2, RopoController::Rising , ControllerModule::ChangeLeftWingPush);
	ButtonDetectLine.AddButtonDetect(pros::E_CONTROLLER_DIGITAL_L1, RopoController::Rising , ControllerModule::ChangeRightWingPush);
	ButtonDetectLine.AddButtonDetect(pros::E_CONTROLLER_DIGITAL_UP, RopoController::Rising , ControllerModule::LaunchHang);
	ButtonDetectLine.AddButtonDetect(pros::E_CONTROLLER_DIGITAL_X, RopoController::Rising , ControllerModule::Hang);
	ButtonDetectLine.AddButtonDetect(pros::E_CONTROLLER_DIGITAL_Y, RopoController::Rising , ControllerModule::BarExtend);
	ButtonDetectLine.AddButtonDetect(pros::E_CONTROLLER_DIGITAL_Y, RopoController::Falling , ControllerModule::BarRecover);
	
	//ButtonDetectLine.AddButtonDetect(pros::E_CONTROLLER_DIGITAL_UP   , RopoController::Rising,  autonomous);
	ButtonDetectLine.AddButtonDetect(pros::E_CONTROLLER_DIGITAL_LEFT , RopoController::Rising,  test);
	ButtonDetectLine.AddButtonDetect(pros::E_CONTROLLER_DIGITAL_RIGHT , RopoController::Rising,  ControllerModule::GpsUpdate);
	ButtonDetectLine.Enable();

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

void test(){
	

}

void delay(){		//代替pros::delay()进行时间控制
	while(!RopoDevice::Chassis.IfArrived()){
		pros::delay(15);
	}
	return;
}

void autonomous_c3(){
	



}

void autonomous_C2(){
	
}

void skill(){
	RopoDevice::Chassis.StartChassisAutoControll();//底盘MoveType设置为AutoMove
	// --------- begin ------------
}
