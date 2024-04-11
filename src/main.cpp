#include "main.h"
#include "RopoApi.hpp"
#include "RopoChassis.hpp"
#include "RopoController.hpp"
#include "RopoDevice.hpp"
#include "RopoMath/Misc.hpp"
#include "pros/llemu.hpp"
#include "pros/misc.h"
#include "pros/rtos.hpp"
#include <cmath>

namespace ControllerModule{
	bool ClimberFlag = false;
	bool ExternLeftFlag = false;
	bool ExternRightFlag = false;
	bool ShooterPneumaticFlag = false;
}

void initialize() {
	RopoDevice::DeviceIni();
	pros::lcd::initialize();
	pros::delay(10);
}

namespace RopoFunction{

	void ShooterPneumatic(){
		if(ControllerModule::ShooterPneumaticFlag == false){
			RopoDevice::ThreeWire::ShooterPneumatic.set_value(true);
			ControllerModule::ShooterPneumaticFlag = true;
		}
		else if(ControllerModule::ShooterPneumaticFlag == true){
			RopoDevice::ThreeWire::ShooterPneumatic.set_value(false);
			ControllerModule::ShooterPneumaticFlag = false;
		}
	}


	void Intake(){
		RopoDevice::Motors::IntakeMotor.move_velocity(-600);
	}
	void Outtake(){
		RopoDevice::Motors::IntakeMotor.move_velocity(600);
	}
	void StopIn(){
		RopoDevice::Motors::IntakeMotor.move_velocity(0);
	}

	
	void Climber(){
		if(ControllerModule::ClimberFlag == false){
			RopoDevice::ThreeWire::ClimberPneumatic.set_value(true);
			ControllerModule::ClimberFlag = true;
		}
		else if(ControllerModule::ClimberFlag == true){
			RopoDevice::ThreeWire::ClimberPneumatic.set_value(false);
			ControllerModule::ClimberFlag = false;
		}
	}

	void ExternLeft(){
		if(ControllerModule::ExternLeftFlag == false){
			RopoDevice::ThreeWire::LExternPneumatic.set_value(true);
			ControllerModule::ExternLeftFlag = true;
		}
		else if(ControllerModule::ExternLeftFlag == true){
			RopoDevice::ThreeWire::LExternPneumatic.set_value(false);
			ControllerModule::ExternLeftFlag = false;
		}
	}

	void ExternRight(){
		if(ControllerModule::ExternRightFlag == false){
			RopoDevice::ThreeWire::RExternPneumatic.set_value(true);
			ControllerModule::ExternRightFlag = true;
		}
		else if(ControllerModule::ExternRightFlag == true){
			RopoDevice::ThreeWire::RExternPneumatic.set_value(false);
			ControllerModule::ExternRightFlag = false;
		}
	}

	
	void IntakerUp(){
		RopoDevice::ThreeWire::IntakerPneumatic.set_value(true);
	}
	void IntakerDown(){
		RopoDevice::ThreeWire::IntakerPneumatic.set_value(false);
	}


	void ShooterInit(){
		RopoDevice::Motors::LShooterMotor.move_velocity(-30);
		RopoDevice::Motors::RShooterMotor.move_velocity(30);
	}
	void ShooterStopInit(){
		RopoDevice::Motors::LShooterMotor.move_velocity(0);
		RopoDevice::Motors::RShooterMotor.move_velocity(0);
	}
	bool DetectLoader(){
		if(RopoDevice::Motors::LShooterMotor.get_torque() < 0.2 && RopoDevice::Motors::RShooterMotor.get_torque() < 0.2) return false;
		else return true;
	}
	void ReLoad(){
		RopoDevice::Motors::LShooterMotor.move_voltage(-12000);
		RopoDevice::Motors::RShooterMotor.move_voltage(12000);
		pros::delay(500);
		while (RopoDevice::Motors::RShooterMotor.get_actual_velocity() > 1) pros::delay(5);
		ShooterStopInit();
	}
	void Shoot(){
		RopoDevice::Motors::LShooterMotor.move_voltage(12000);
		RopoDevice::Motors::RShooterMotor.move_voltage(-12000);
		pros::delay(500);
		while (RopoDevice::Motors::LShooterMotor.get_actual_velocity() > 1) pros::delay(5);
		ShooterStopInit();
	}


	void shootandsweep(int n){

		float x,y,theta;
		x = RopoDevice::Chassis.GetX();
		y = RopoDevice::Chassis.GetY();
		theta = RopoDevice::Chassis.GetTheta();
		ReLoad();
		ShooterPneumatic();
		ExternLeft();

		for(int i = 1; i<=n; i++)
		{
			pros::delay(1000);
			Shoot();
			pros::delay(300);
			RopoDevice::Chassis.AutoSetAimStatus(0, 0, 8);
			pros::delay(800);
			RopoDevice::Chassis.AutoSetAimStatus(0, 0, 0);
			pros::delay(300);
			RopoDevice::Chassis.AutoSetPosition(x, y, theta, 2000);
			while (RopoDevice::Chassis.IfPosition_OK() == false) {pros::delay(20);}
			ReLoad();
		}
		
		ShooterPneumatic();
		Shoot();
		ExternLeft();


	}

	void closemove(float x, float y, float theta,int max_time)
	{
		RopoDevice::Chassis.AutoSetPosition(x, y, theta, max_time);
		while(RopoDevice::Chassis.IfPosition_OK() == false)
		{
			pros::delay(20);
		}
	}



	/*void Hit(){
		RopoDevice::Motors::HitterMotor.move_voltage(-12000);
		pros::delay(200);
		while (std::abs(RopoDevice::Motors::HitterMotor.get_actual_velocity()) > 5) pros::delay(5);
		RopoDevice::Motors::HitterMotor.move_voltage(0);
	}
	void HitterReset(){
		RopoDevice::Motors::HitterMotor.move_voltage(12000);
		pros::delay(200);
		while (RopoDevice::Motors::HitterMotor.get_actual_velocity() > 5) pros::delay(5);
		RopoDevice::Motors::HitterMotor.move_voltage(0);
	}*/

	void Test(){
		RopoDevice::Chassis.AutoSetPosition(0,0,0,2000);
	}
	//bool ControlMode = false;
	
	void ChangeControlMode(){
		//ControlMode = !ControlMode;
		RopoDevice::Chassis.ChangeControlMode();
	}
	void Import(){

	}

	void autonomous_1(){
	//float x,y,theta;
	//x = RopoDevice::Chassis.GetX();
	//y = RopoDevice::Chassis.GetY();
	//theta = RopoDevice::Chassis.GetTheta();

	
	/*	step_1 拨出联队球推入球网	*/
	RopoFunction::ExternRight();
	pros::delay(700);
	RopoDevice::Chassis.AutoSetAimStatus(-1, 0, -6);
	pros::delay(600);
	RopoDevice::Chassis.AutoSetAimStatus(0, 0, 0);
	pros::delay(500);
	RopoFunction::ExternRight();
	RopoFunction::closemove(-0.38, 0.09, -45, 500);
	RopoDevice::Chassis.AutoSetAimStatus(-1.57, 0, 0);
	pros::delay(800);
	RopoDevice::Chassis.AutoSetAimStatus(0, 0, 0);
	/*	step_1 end	*/ 


	/*  step_2 到中间撞球并返回导球点  */
	pros::delay(200);
	RopoDevice::Chassis.AutoSetAimStatus(1, -1, 0);
	pros::delay(500);
	RopoDevice::Chassis.AutoSetAimStatus(0, 0, 0);
	pros::delay(50);
	RopoFunction::closemove( -0.06, 0.72, -35.33, 800);
	RopoFunction::closemove( -0.27, 0.86, -22.29, 500);
	RopoFunction::closemove( -0.46, 0.96, -36.49, 500);
	RopoFunction::closemove( -0.58, 0.98, -45.48, 500);
	pros::delay(100);
	RopoDevice::Chassis.AutoSetAimStatus(0, -1.57, 0);
	pros::delay(1500);
	RopoDevice::Chassis.AutoSetAimStatus(0, 0, 0);
	RopoFunction::closemove(0.03, 0.6, 66.98, 1500);
	RopoFunction::closemove(0.10, 0.01, 72.46, 1500);
	/*	step_2 end	*/ 


	/*  step_3 抛球x3 导球x7  */
	pros::delay(500);
	RopoFunction::ReLoad();
	pros::delay(200);
	RopoFunction::ShooterPneumatic();
	pros::delay(200);
	for(int i = 1; i <= 3; i++){
		RopoFunction::Shoot();
		pros::delay(800);
		RopoFunction::ReLoad();
		pros::delay(50);
	} 
	RopoFunction::ShooterPneumatic();
	pros::delay(200);
	RopoFunction::Shoot();
	pros::delay(500);
	RopoFunction::closemove(0.08, 0.07, 18.0, 1000);
	pros::delay(500);
	RopoFunction::ExternRight();
	pros::delay(500);
	for (int i =1;i <= 7; i++) {
		if(i%3 == 0)
		{
			RopoFunction::closemove(0.08, 0.07, 21.4, 800);
		}
		RopoDevice::Chassis.AutoSetAimStatus(0, 0, 8);
		pros::delay(300);
		RopoDevice::Chassis.AutoSetAimStatus(0, 0, 0);
		pros::delay(200);
		RopoDevice::Chassis.AutoSetAimStatus(0, 0, -8);
		pros::delay(300);
		RopoDevice::Chassis.AutoSetAimStatus(0, 0, 0);
		pros::delay(600);
	}
	RopoFunction::ExternRight();
	RopoFunction::closemove(0.37, 0.10, -134.33, 2000);
	pros::delay(500);
	/*	step_3 end	*/ 


	/*  step_4 将导出的球推入球网  */
	RopoDevice::Chassis.AutoSetAimStatus(-1.7, 0, 0);
	pros::delay(800);
	RopoFunction::closemove(1.28, 0.80, -139.0, 2000);
	RopoFunction::ExternRight();
	RopoFunction::closemove(1.80, 1.52, -135.0, 800);
	RopoFunction::closemove(1.98, 2.11, -81.0, 800);
	RopoFunction::closemove(1.69, 2.44, -50.0, 800);
	RopoDevice::Chassis.AutoSetAimStatus(-2.00, 0, 0);
	pros::delay(1000);
	RopoDevice::Chassis.AutoSetAimStatus(0, 0, 0);
	RopoFunction::ExternRight();
	pros::delay(200);
	/* RopoFunction::closemove(1.60, 2.37, -47.377, 2000);
	RopoDevice::Chassis.AutoSetAimStatus(-1.57, 0, 0);
	pros::delay(1000);
	RopoDevice::Chassis.AutoSetAimStatus(0, 0, 0);
	RopoFunction::ExternRight();
	pros::delay(200); */
	/*	step_4 end	*/ 
	

	/*  step_5 碰杆  */
	RopoFunction::closemove(1.69, 1.91, -88.733, 1000);
	RopoFunction::closemove(1.65, 1.49, -122.77, 800);
	RopoFunction::closemove(1.36, 1.20, -135.52, 800);
	RopoFunction::closemove(1.09, 0.83, -135.52, 800);
	RopoFunction::ReLoad(); 
	/*	step_5 end	*/






















	/*
	//pros::delay(3000);
	RopoFunction::closemove(-0.44, 1.05, -44.80, 2000);
	//pros::delay(2000);
	
	RopoFunction::closemove(-0.05, 0.03, 90.03, 4000);
	//pros::delay(2000);

	RopoFunction::shootandsweep(3);*/

	/*RopoFunction::closemove(x, y, theta, 3000);
	RopoFunction::shootandsweep(3);
	RopoDevice::Chassis.AutoSetPosition(0,0,-135,1500);
	RopoDevice::Chassis.AutoSetPosition(0,0,180,1000);
	//while ( RopoDevice::Chassis.IfPosition_OK()) pros::delay(20);
	RopoFunction::ExternRight();
	RopoDevice::Chassis.AutoSetPosition(1,0.15,-90,2000);
	RopoDevice::Chassis.AutoSetAimStatus(1, 0, 0);
	pros::delay(1000);*/
	RopoDevice::Chassis.Operator();
}
}






void autonomous_A1(){
	

	/*RopoDevice::Chassis.OpSetAimStatus(XInput, YInput, WInput);
	RopoDevice::Chassis.AutoSetPosition();*/
	float x,y,theta;
	RopoFunction::ReLoad();
	//RopoFunction::HitterReset();

	pros::delay(1000);


	RopoFunction::Intake();
 	RopoDevice::Chassis.AutoSetPosition(0,5,0,100);
	RopoFunction::StopIn();
	RopoFunction::ExternRight();
	RopoDevice::Chassis.AutoSetPosition(0,-40,0,100);
	RopoDevice::Chassis.AutoSetPosition(0,-40,45,100);
	RopoFunction::ExternLeft();
	RopoDevice::Chassis.AutoSetPosition(5,-45,45,100);
	RopoDevice::Chassis.AutoSetPosition(5,-45,90,100);
	RopoDevice::Chassis.AutoSetAimStatus(0, -20, 0);
	RopoDevice::Chassis.AutoSetAimStatus(0, 10, 0);
	RopoDevice::Chassis.AutoSetAimStatus(0, -20, 0);
	RopoDevice::Chassis.AutoSetAimStatus(0, 10, 0);

	x = RopoDevice::Chassis.GetX();
	y = RopoDevice::Chassis.GetY();
	RopoDevice::Chassis.AutoSetPosition(x,y,-90,100);
	
	RopoDevice::Chassis.AutoSetAimStatus(0, 20, 0);




















}






void disabled() {}

void competition_initialize() {}

void skill() {
	
}

void autonomous(){
}

void opcontrol() {
	pros::Controller MasterController(pros::E_CONTROLLER_MASTER);
	RopoController::ButtonTaskLine ButtonDetectLine(MasterController);

	FloatType VelocityMax = 1.57;
	FloatType RopoWcLimit = 8;

	/*FloatType LastXInput = 0;
	FloatType LastYInput = 0;
	FloatType LastWInput = 0;
	FloatType XInput = 0;
	FloatType YInput = 0;
	FloatType WInput = 0;*/
	
	RopoController::AxisValueCast XVelocityInput(MasterController,pros::E_CONTROLLER_ANALOG_LEFT_Y,RopoController::Ln);
	RopoController::AxisValueCast YVelocityInput(MasterController,pros::E_CONTROLLER_ANALOG_LEFT_X,RopoController::Ln);
	RopoController::AxisValueCast WVelocityInput(MasterController,pros::E_CONTROLLER_ANALOG_RIGHT_X,RopoController::Linear);
	Vector Velocity(RopoMath::ColumnVector,2),ResVelocity;
	MasterController.clear();

	//ButtonDetectLine.AddButtonDetect(pros::E_CONTROLLER_DIGITAL_A , RopoController::Rising, autonomous_A1);

	ButtonDetectLine.AddButtonDetect(pros::E_CONTROLLER_DIGITAL_R1, RopoController::Rising, RopoFunction::Intake);
	ButtonDetectLine.AddButtonDetect(pros::E_CONTROLLER_DIGITAL_R1, RopoController::Falling, RopoFunction::StopIn);

	ButtonDetectLine.AddButtonDetect(pros::E_CONTROLLER_DIGITAL_R2, RopoController::Rising, RopoFunction::Outtake);
	ButtonDetectLine.AddButtonDetect(pros::E_CONTROLLER_DIGITAL_R2, RopoController::Falling, RopoFunction::StopIn);
	

	ButtonDetectLine.AddButtonDetect(pros::E_CONTROLLER_DIGITAL_L2, RopoController::Rising, RopoFunction::ExternLeft);
	ButtonDetectLine.AddButtonDetect(pros::E_CONTROLLER_DIGITAL_L1, RopoController::Rising, RopoFunction::ExternRight);

	ButtonDetectLine.AddButtonDetect(pros::E_CONTROLLER_DIGITAL_UP, RopoController::Rising, RopoFunction::Climber);

	//ButtonDetectLine.AddButtonDetect(pros::E_CONTROLLER_DIGITAL_Y, RopoController::Rising, RopoFunction::IntakerUp);
	//ButtonDetectLine.AddButtonDetect(pros::E_CONTROLLER_DIGITAL_Y, RopoController::Falling, RopoFunction::IntakerDown);


	ButtonDetectLine.AddButtonDetect(pros::E_CONTROLLER_DIGITAL_RIGHT, RopoController::Rising, RopoFunction::ShooterInit);
	ButtonDetectLine.AddButtonDetect(pros::E_CONTROLLER_DIGITAL_RIGHT, RopoController::Falling, RopoFunction::ShooterStopInit);
	ButtonDetectLine.AddButtonDetect(pros::E_CONTROLLER_DIGITAL_B, RopoController::Rising, RopoFunction::ReLoad);
	ButtonDetectLine.AddButtonDetect(pros::E_CONTROLLER_DIGITAL_X, RopoController::Rising, RopoFunction::Shoot);

	//ButtonDetectLine.AddButtonDetect(pros::E_CONTROLLER_DIGITAL_LEFT, RopoController::Rising, RopoFunction::Hit);
	//ButtonDetectLine.AddButtonDetect(pros::E_CONTROLLER_DIGITAL_LEFT, RopoController::Falling, RopoFunction::HitterReset);
	ButtonDetectLine.AddButtonDetect(pros::E_CONTROLLER_DIGITAL_DOWN, RopoController::Rising, RopoFunction::ShooterPneumatic);
	ButtonDetectLine.AddButtonDetect(pros::E_CONTROLLER_DIGITAL_A, RopoController::Rising, RopoFunction::Test);
	ButtonDetectLine.AddButtonDetect(pros::E_CONTROLLER_DIGITAL_Y, RopoController::Rising, RopoFunction::autonomous_1);
	//ButtonDetectLine.AddButtonDetect(pros::E_CONTROLLER_DIGITAL_R2, RopoController::Rising, RopoFunction::Import);
	//ButtonDetectLine.AddButtonDetect(pros::E_CONTROLLER_DIGITAL_LEFT, RopoController::Rising, RopoFunction::ChangeControlMode);
	ButtonDetectLine.Enable();
	RopoDevice::Chassis.Operator();
	while (true) {

		/*XInput = RopoMath::LowPassFilter(3 * XVelocityInput.GetAxisValue(),LastXInput,50,1000);
		YInput = RopoMath::LowPassFilter(3 * YVelocityInput.GetAxisValue(),LastYInput,50,1000);
		WInput = RopoMath::LowPassFilter(-5 * WVelocityInput.GetAxisValue(),LastWInput,50,1000);*/



		FloatType XInput =  VelocityMax * XVelocityInput.GetAxisValue();
		FloatType YInput =  VelocityMax * YVelocityInput.GetAxisValue();
		FloatType WInput = -RopoWcLimit * WVelocityInput.GetAxisValue();
		/* if(RopoFunction::ControlMode){
                    double a =  XInput * cos(RopoDevice::Chassis.GetTheta()) - YInput * sin(RopoDevice::Chassis.GetTheta());
                    double b =  YInput * cos(RopoDevice::Chassis.GetTheta()) + XInput * sin(RopoDevice::Chassis.GetTheta());
                    XInput = a;
                    YInput = b;
                } */
		/*FloatType YInput = 1;
		FloatType XInput = 0;
		FloatType WInput = 0;*/

		if(RopoDevice::Chassis.IsOpcontrol()) RopoDevice::Chassis.OpSetAimStatus(XInput, YInput, WInput);

		/*LastXInput = XInput;
		LastYInput = YInput;
		LastWInput = WInput;*/

		//MasterController.print(0,0,"%.2f, %.2f, %.2f", RopoDevice::Chassis.GetAimStatus()[1][1], RopoDevice::Chassis.GetAimStatus()[2][1], RopoDevice::Chassis.GetTheta() * 180 / RopoMath::Pi);
		//MasterController.print(0,0,"%.2f, %.2f", 180 * RopoDevice::LF.get_Angle() / RopoMath::Pi, 180 * RopoDevice::LB.get_Angle() / RopoMath::Pi);
		//MasterController.print(1,0,"%.2f, %.2f", 180 * RopoDevice::RF.get_Angle() / RopoMath::Pi, 180 * RopoDevice::RB.get_Angle() / RopoMath::Pi);
		//MasterController.print(0,0,"%.2f, %.2f, %.2f", RopoDevice::Chassis.GetSwerveAimStatus(2,1) * 180 / RopoMath::Pi,RopoDevice::Chassis.GetSwerveAimStatus(4,1) * 180 / RopoMath::Pi,RopoDevice::Chassis.GetSwerveAimStatus(6,1) * 180 / RopoMath::Pi);
		//MasterController.print(0,0,"%.2f, %.2f", RopoDevice::Chassis.GetSwerveAimStatus(2,1) * 180 /RopoMath::Pi,RopoDevice::LF.x * 180 /RopoMath::Pi);
		//MasterController.print(0,0,"%.2f",RopoDevice::LF.x);
		//MasterController.print(0,0,"%.2f, %.2f, %.2f", RopoDevice::LF.GetStatusError(1),RopoDevice::LF.GetStatusError(2),RopoDevice::LF.GetStatusError(3));
		MasterController.print(0,0,"%.2f, %.2f, %.2f", RopoDevice::Sensors::Encoder.GetPosX()/1000, RopoDevice::Sensors::Encoder.GetPosY()/1000, RopoDevice::Chassis.GetTheta() * 180 / RopoMath::Pi);
		pros::delay(5);
		//MasterController.print(1,0,"%s", RopoDevice::Sensors::Encoder.IsReading()?"1":"2");
		//pros::delay(5);
	}
}