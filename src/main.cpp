#include "main.h"
#include "RopoApi.hpp"
#include "RopoPros/RopoController.hpp"
#include "RopoDevice.hpp"
#include "RopoGpsAddPosition.hpp"
#include "RopoMath/Misc.hpp"
#include "RopoPosition.hpp"
#include "pros/misc.h"
#include "pros/rtos.hpp"
#include "RopoOperation.hpp"
#include <cmath>


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
	
	//skill();
}

void opcontrol()
{
	RopoDevice::Chassis.MoveVelocity(0.0,0);
	RopoDevice::Chassis.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
	pros::Task *PrintTask = new pros::Task(ControllerModule::ControllerPrint);
	pros::Controller MasterController(pros::E_CONTROLLER_MASTER);
	RopoController::ButtonTaskLine ButtonDetectLine(MasterController);
	FloatType VelocityMax = 2;
	FloatType RopoWcLimit = 8;
		
	RopoController::AxisValueCast XVelocityInput(MasterController,pros::E_CONTROLLER_ANALOG_LEFT_Y,RopoController::Linear);
	RopoController::AxisValueCast WVelocityInput(MasterController,pros::E_CONTROLLER_ANALOG_RIGHT_X,RopoController::Linear);
	Vector Velocity(RopoMath::ColumnVector,2),ResVelocity;

	MasterController.clear();
	
	ButtonDetectLine.AddButtonDetect(pros::E_CONTROLLER_DIGITAL_Y , RopoController::Rising , InterruptMain_doTask);
    ButtonDetectLine.AddButtonDetect(pros::E_CONTROLLER_DIGITAL_L1 , RopoController::Rising , ControllerModule::switch_both_wing);
    ButtonDetectLine.AddButtonDetect(pros::E_CONTROLLER_DIGITAL_L2 , RopoController::Rising , ControllerModule::switch_climber);
    ButtonDetectLine.AddButtonDetect(pros::E_CONTROLLER_DIGITAL_R1 , RopoController::Rising , ControllerModule::intake);    
    ButtonDetectLine.AddButtonDetect(pros::E_CONTROLLER_DIGITAL_R1 , RopoController::Falling , ControllerModule::intaker_stop);
	ButtonDetectLine.AddButtonDetect(pros::E_CONTROLLER_DIGITAL_R2 , RopoController::Rising , ControllerModule::switch_lead_ball);
    ButtonDetectLine.AddButtonDetect(pros::E_CONTROLLER_DIGITAL_X , RopoController::Rising , ControllerModule::pos_reset);   


	ButtonDetectLine.Enable();

	FloatType XInput=0;
	FloatType WInput=0;
	FloatType RopoWc=0;			
	FloatType RopoVx=0;	
		
	RopoDevice::Chassis.StartChassisOpControll();//底盘MoveType设置为OpMove
	
	while (true) {	
		//主线程断点
		while(main_process==false)
		{
			pros::delay(500);
		}

		XInput =  XVelocityInput.GetAxisValue();
		WInput = -WVelocityInput.GetAxisValue();
		RopoWc = RopoWcLimit-fabs(XInput) * 4.9;			
		RopoVx = VelocityMax-fabs(WInput) * 0.7;	
		if (fabs(XInput) <= 0.06 && fabs(WInput) <= 0.06) 
		{		
			RopoDevice::Motors::MoveOpControll(0.0, 0.0);				
		} 
		else 
		{
			int32_t err=RopoDevice::Motors::MoveOpControll(XInput * RopoVx, WInput * RopoWc);			
		}
		pros::delay(10);
	}
}

