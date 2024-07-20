#include "main.h"
#include "RopoApi.hpp"
#include "RopoController.hpp"
#include "RopoDevice.hpp"
#include "RopoGpsAddPosition.hpp"
#include "RopoMath/Misc.hpp"
#include "RopoPosition.hpp"
#include "pros/misc.h"
#include "pros/rtos.hpp"
#include <algorithm>
#include <cmath>
void test();
void skill();
void autonomous_c3();
void autonomous_C2();
void autonomous_KnockoutMatch();
void PositionInit();
void autonomous_KnockoutMatch_1();
void autonomous_qualify();
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
	/*int catch_1 = 0;
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
	}*/

	bool leftwing_status = false;
	bool rightwing_status = false;
	bool spade_status = false;
	bool hang_status = false;
	void ChangeLeftWingPush(){
		leftwing_status ^= 1;
		RopoDevice::ThreeWire::LeftWingPneumatic.set_value(leftwing_status);
	}
	void ChangeRightWingPush(){
		rightwing_status ^= 1;
		RopoDevice::ThreeWire::RightWingPneumatic.set_value(rightwing_status);
	}
	void LaunchHang(){
		RopoDevice::ThreeWire::IntakerPneumatic1.set_value(true);
		RopoDevice::ThreeWire::IntakerPneumatic2.set_value(true);
		pros::delay(500);
		RopoDevice::ThreeWire::ExternPneumatic.set_value(true);
	}
	void Hang(){
		RopoDevice::ThreeWire::BarPneumatic.set_value(true);
		RopoDevice::ThreeWire::IntakerPneumatic1.set_value(true);
		RopoDevice::ThreeWire::IntakerPneumatic2.set_value(true);
		RopoDevice::ThreeWire::BarPneumatic.set_value(true);
		if (hang_status == 1) {
			pros::delay(500);
			RopoDevice::ThreeWire::HangPneumatic.set_value(true);	
		}
		hang_status ^= 1;
		RopoDevice::ThreeWire::ExternPneumatic.set_value(hang_status);
	}
	void BarRecover(){
		RopoDevice::ThreeWire::BarPneumatic.set_value(false);
	}
	void BarExtend(){
		RopoDevice::ThreeWire::BarPneumatic.set_value(true);
	}
	bool bar_status = 0;
	void Bar(){
		if (bar_status == 0) {
			BarExtend();
		}
		else {
			BarRecover();
		}
		bar_status ^= 1;
	}

	bool intaker_status = false; 
	void Intake(){
		RopoDevice::Motors::LeftIntakeMotor.move_voltage(-550.0 / 600.0 * 12000.0);
		RopoDevice::Motors::RightIntakeMotor.move_voltage(-550.0 / 600.0 * 12000.0);
	}
	void Outtake(){
		RopoDevice::Motors::LeftIntakeMotor.move_velocity(400);
		RopoDevice::Motors::RightIntakeMotor.move_velocity(400);
		pros::delay(400);
		RopoDevice::Motors::LeftIntakeMotor.move_voltage(8000);
		RopoDevice::Motors::RightIntakeMotor.move_voltage(8000);
	}
	void IntakerStop(){
		if (intaker_status == 0) {
			RopoDevice::Motors::LeftIntakeMotor.move_voltage(0);
			RopoDevice::Motors::RightIntakeMotor.move_voltage(0);
		}
	}

	  
	void ChangeIntakerPneumatic(){
		
		intaker_status ^= 1;
		RopoDevice::ThreeWire::IntakerPneumatic1.set_value(intaker_status);
		RopoDevice::ThreeWire::IntakerPneumatic2.set_value(intaker_status);
		if(intaker_status == 1){Intake();}
		else {IntakerStop();}
		//pros::delay(100);
		//if(intaker_status)Outtake();
	}

	void TurnAround(){
		RopoDevice::Chassis.AutoRotateRelative(180);
	}

	void GpsUpdate(){
		RopoDevice::Chassis.MoveVelocity(0,0);
		pros::delay(600);
		RopoDevice::gpsAddPosition.SetUpdateFlag(1);
		pros::delay(200);
		RopoDevice::gpsAddPosition.SetUpdateFlag(0);
	}

	

	void ControllerPrint(){
		while(true) {
			pros::Controller MasterController(pros::E_CONTROLLER_MASTER);
			// Position ******************************************************************************************************
			MasterController.print(0,1,"X: %.2lf Y:%.2lf   ",(RopoDevice::GetTransformedPosition())[1],(RopoDevice::GetTransformedPosition())[2]);
			pros::delay(50); 
			MasterController.print(1,1,"degree: %.3lf    ",RopoDevice::GetPosition()[3]);
			pros::delay(50); 
			// MasterController.print(2,1,"%.2lf    ",RopoDevice::Sensors::Inertial.get_pitch());
			// pros::delay(50);

			

			//Openmv**********************************************************************************************************
			// MasterController.print(0,1,"Read:%s",RopoDevice::Sensors::My_openMV.IsReading()?"yes":"no");
			// pros::delay(50); 
			// MasterController.print(1,1,"Deg:%.2f",RopoDevice::Sensors::My_openMV.Get_Ball_Deg());
			// pros::delay(50);


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
	autonomous_qualify();
	// autonomous_KnockoutMatch();
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
	//ButtonDetectLine.AddButtonDetect(pros::E_CONTROLLER_DIGITAL_A, RopoController::Rising , autonomous_qualify);
	
	ButtonDetectLine.AddButtonDetect(pros::E_CONTROLLER_DIGITAL_RIGHT , RopoController::Rising,  ControllerModule::GpsUpdate);
	ButtonDetectLine.AddButtonDetect(pros::E_CONTROLLER_DIGITAL_A , RopoController::Rising,  skill);
	/*end*/

	ButtonDetectLine.Enable();
	ControllerModule::BarRecover();
	ControllerModule::bar_status = 0;

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

void delay(){		//代替pros::delay()进行时间控制
	while(!RopoDevice::Chassis.IfArrived()){
		pros::delay(5);
	}
	return;
}

void test(){
	RopoDevice::Chassis.StartChassisAutoControll();   // 设置为自动状态
	RopoDevice::ChassisBrake();						  // 自动赛时需要设置刹车状态为brake

	ControllerModule::BarExtend();	// 打开导入杆
	RopoDevice::Chassis.AutoPositionMove(1.10,0);  // 移动到扫球位置
	RopoDevice::Chassis.AutoRotateAbs(-100.0);	// 扫球
	delay();
	ControllerModule::BarRecover();

	RopoDevice::Chassis.AutoPositionMove(0.92,-0.32);  // 中间过渡点
	
	RopoDevice::Chassis.AutoRotateAbs(-45.0);
	delay();
	ControllerModule::Intake();
	ControllerModule::ChangeIntakerPneumatic();	

	RopoDevice::Chassis.AutoPositionMove(1.11,-0.49);  // 中间吃球点
	ControllerModule::BarExtend();
	pros::delay(800);

	RopoDevice::Chassis.MoveVelocity(0,-4);
	pros::delay(400);
	RopoDevice::Chassis.MoveVelocity(0,0);
	ControllerModule::ChangeIntakerPneumatic();
	ControllerModule::BarRecover();
	

	// RopoDevice::Chassis.MoveVelocity(0.5,0);
	// pros::delay(300);
	// RopoDevice::Chassis.MoveVelocity(0,0);

	// RopoDevice::Chassis.AutoPositionMove(0.74,-0.48,-180);

	// ControllerModule::ChangeLeftWingPush();
	// ControllerModule::Outtake();

	// RopoDevice::Chassis.MoveVelocity(0.5,0);
	// int count = 0 ;
	// while (fabs(RopoDevice::Sensors::Inertial.get_pitch()) < 18 && count <= 2500) {
	//  	pros::delay(10);
	// 	count = count + 10;
	// }
	// RopoDevice::Chassis.MoveVelocity(0,0);

	// RopoDevice::Chassis.MoveVelocity(-1,0);
	// while (fabs(RopoDevice::Sensors::Inertial.get_pitch()) > 3) {
	//  	pros::delay(10);
	// }
	// RopoDevice::Chassis.MoveVelocity(0,0);
	// pros::delay(200);

	// /* 前往导球点_选择其中一个方案 */

	// /* 方案一.前方推球并抵达导球点 */
	// RopoDevice::Chassis.AutoRotateAbs(90);
	// delay();
	// RopoDevice::Chassis.MoveVelocity(0,0);
	// ControllerModule::GpsUpdate();
}

void autonomous_c3(){
}

void autonomous_C2(){
}

void autonomous_qualify(){
	RopoDevice::Chassis.StartChassisAutoControll();   // 设置为自动状态
	RopoDevice::ChassisBrake();						  // 自动赛时需要设置刹车状态为brake
	//吃预装
	ControllerModule::Intake();
	RopoDevice::Chassis.MoveVelocity(0.6,0); //0.4
	pros::delay(150);//200
	RopoDevice::Chassis.MoveVelocity(0,0);
	pros::delay(500);//700
	//推球
	RopoDevice::Chassis.MoveVelocity(-1.1,0);
	pros::delay(1350);
	RopoDevice::Chassis.MoveVelocity(-1.0,1.3);
	pros::delay(1450);

	RopoDevice::Chassis.MoveVelocity(-0.5,0);
	pros::delay(300);
	//持球入网
	RopoDevice::Chassis.MoveVelocity(0.8,0); //0.5 600
	pros::delay(400);
	RopoDevice::Chassis.AutoRotateAbs(-90);
	delay();
	ControllerModule::Outtake();
	RopoDevice::Chassis.MoveVelocity(0.8,0); // 0.5 550
	pros::delay(300);
	RopoDevice::Chassis.MoveVelocity(0,0);
	//吃联队粽球---------------
	//gps定位，退出至合适位置
	RopoDevice::Chassis.MoveVelocity(-0.5,0);   // -0.35 300
	pros::delay(200);
	RopoDevice::Chassis.MoveVelocity(0,0);
	ControllerModule::IntakerStop();
	RopoDevice::Chassis.AutoRotateAbs(-90);
	delay();
	//ControllerModule::GpsUpdate();
	//pros::delay(200);
	RopoDevice::Chassis.MoveVelocity(-0.3,-1.5);
	pros::delay(1800);//1300
	RopoDevice::Chassis.MoveVelocity(-0.7,0);
	pros::delay(400);
	// RopoDevice::Chassis.MoveVelocity(0,0);
	// pros::delay(100);

	RopoDevice::Chassis.AutoRotateAbs(0);
	delay();
	ControllerModule::GpsUpdate();
	RopoDevice::gpsAddPosition.SetUpdateFlag(1);
	pros::delay(200);
	RopoDevice::gpsAddPosition.SetUpdateFlag(0);

	RopoDevice::Chassis.AutoPositionMove(-1.99,-0.27,135);//-1.97 -0.41
	delay();
	//吃联队球
	ControllerModule::ChangeIntakerPneumatic();
	ControllerModule::Intake();
	pros::delay(100);
	RopoDevice::Chassis.MoveVelocity(-0.25,0);
	pros::delay(400);
	RopoDevice::Chassis.MoveVelocity(0.3,0);
	pros::delay(1250);//950
	RopoDevice::Chassis.MoveVelocity(0,0);
	pros::delay(1000);
	RopoDevice::Chassis.MoveVelocity(-0.7,0);
	pros::delay(900);
	ControllerModule::ChangeIntakerPneumatic();
	ControllerModule::Intake();
	pros::delay(300);
	RopoDevice::Chassis.MoveVelocity(0,0);
	pros::delay(200);
	RopoDevice::Chassis.MoveVelocity(-0.5,0);
	pros::delay(300);
	//送入网
	RopoDevice::Chassis.AutoPositionMove(-2.18,-0.48);//-2.20,-0.51,-90
	delay();
	pros::delay(100);
	RopoDevice::Chassis.AutoRotateAbs(-90);
	delay();
	ControllerModule::Outtake();
	RopoDevice::Chassis.MoveVelocity(0.7,0); //0.4
	pros::delay(600);
	RopoDevice::Chassis.MoveVelocity(0,0);
	
	//处理场地球-----------------
	//吃边缘
	RopoDevice::Chassis.MoveVelocity(-1,0);
	pros::delay(350);
	RopoDevice::Chassis.AutoRotateAbs(-90);
	delay();
	RopoDevice::Chassis.MoveVelocity(0,0);
	ControllerModule::IntakerStop();
	ControllerModule::GpsUpdate();
	ControllerModule::Intake();
	RopoDevice::Chassis.AutoPositionMove(-1.05,-0.85);//-0.95 -1
	delay();
	pros::delay(100);
	//吐球
	RopoDevice::Chassis.AutoRotateAbs(-120);
	delay();
	RopoDevice::Chassis.MoveVelocity(0.4,0);
	pros::delay(400);
	ControllerModule::Outtake();
	RopoDevice::Chassis.MoveVelocity(-0.8,0);
	pros::delay(250);
	RopoDevice::Chassis.AutoRotateAbs(120);
	delay();
	//转身开翅膀推
	RopoDevice::Chassis.MoveVelocity(-0.5,-1.0);
	pros::delay(700);
	RopoDevice::Chassis.MoveVelocity(-0.4,0);
	pros::delay(100);
	RopoDevice::Chassis.MoveVelocity(-0.4,-1.3);
	pros::delay(700);
	ControllerModule::ChangeRightWingPush();
	ControllerModule::ChangeLeftWingPush();
	pros::delay(700);
	RopoDevice::Chassis.AutoRotateAbs(0);
	delay();
	RopoDevice::Chassis.MoveVelocity(-0.8,0); //-0.7
	pros::delay(800); //700
	ControllerModule::ChangeRightWingPush();
	ControllerModule::ChangeLeftWingPush();
	//送入吃到的
	// RopoDevice::Chassis.MoveVelocity(0.4,0);
	// pros::delay(500);
	// RopoDevice::Chassis.AutoRotateAbs(180);
	// delay();
	// ControllerModule::Outtake();
	// RopoDevice::Chassis.MoveVelocity(0.5,0); //0.5
	// pros::delay(600);
	// RopoDevice::Chassis.MoveVelocity(0,0);
	// ControllerModule::IntakerStop();
	RopoDevice::Chassis.MoveVelocity(0.5,0);
	pros::delay(300);
	RopoDevice::Chassis.MoveVelocity(0,0);

	// RopoDevice::Chassis.AutoPositionMoveBack(-1.02,-1.30);
	// pros::delay(700);
	// RopoDevice::Chassis.AutoRotateAbs(-75);
	// pros::delay(600);
	// ControllerModule::GpsUpdate();

	// RopoDevice::Chassis.AutoPositionMove(-1.00,-1.27);
	// delay();
	// RopoDevice::Chassis.AutoRotateAbs(-75);
	// delay();
	// ControllerModule::ChangeIntakerPneumatic();
	// ControllerModule::Intake();
	// RopoDevice::Chassis.MoveVelocity(0.3,0);
	// pros::delay(700);
	// RopoDevice::Chassis.MoveVelocity(0,0);
	// pros::delay(500);
	// ControllerModule::ChangeIntakerPneumatic();



	RopoDevice::Chassis.AutoRotateAbs(40);
	delay();

	RopoDevice::Chassis.MoveVelocity(0.8,0);
	pros::delay(800);
	RopoDevice::Chassis.MoveVelocity(0,0);
	RopoDevice::Chassis.AutoRotateAbs(-75);
	delay();
	ControllerModule::GpsUpdate();
	//pros::delay(100);
	ControllerModule::ChangeIntakerPneumatic();
	ControllerModule::Intake();
	RopoDevice::Chassis.AutoPositionMove(-1,-1.42); //-1 -1.44
	delay();
	pros::delay(500);
	ControllerModule::ChangeIntakerPneumatic();
	pros::delay(200);
	RopoDevice::Chassis.AutoRotateAbs(180);
	delay();
	ControllerModule::ChangeLeftWingPush();
	ControllerModule::ChangeRightWingPush();
	RopoDevice::Chassis.MoveVelocity(0.6,0);
	pros::delay(600);
	ControllerModule::Outtake();
	pros::delay(400);
	ControllerModule::IntakerStop();
	RopoDevice::Chassis.MoveVelocity(-0.3,0);
	pros::delay(300);
	ControllerModule::ChangeLeftWingPush();
	ControllerModule::ChangeRightWingPush();
	//碰杆-----------
	RopoDevice::Chassis.MoveVelocity(-0.5,-0.3);
	pros::delay(400);
	RopoDevice::Chassis.AutoPositionMove(-1.1,-0.65); //-0.97,-0.65
	RopoDevice::Chassis.AutoRotateAbs(50);
	delay();
	ControllerModule::BarExtend();
	ControllerModule::ChangeIntakerPneumatic();
	// ControllerModule::IntakerStop();
	RopoDevice::Motors::LeftIntakeMotor.move_voltage(0);
	RopoDevice::Motors::RightIntakeMotor.move_voltage(0);
	RopoDevice::Chassis.MoveVelocity(0.1,0.1);
	pros::delay(1500); // 3000
	RopoDevice::Chassis.MoveVelocity(0.1,5);
	pros::delay(1000);
	RopoDevice::Chassis.MoveVelocity(0,0);


}

void skill(){
	RopoDevice::Chassis.StartChassisAutoControll();   			// 设置为自动状态
	RopoDevice::ChassisBrake();						  			// 自动赛时需要设置刹车状态为brake

	RopoDevice::Chassis.MoveVelocity(-2,0);
	pros::delay(300);
	RopoDevice::Chassis.MoveVelocity(0,0);
	pros::delay(50);
	RopoDevice::Chassis.MoveVelocity(2,0);
	pros::delay(200);
	RopoDevice::Chassis.MoveVelocity(0.8,0);
	pros::delay(200);
	RopoDevice::Chassis.AutoPositionMove(0.34,-0.12,45.0);
	ControllerModule::BarExtend();
	pros::delay(400);

	for(int i = 1 ; i <= 8 ; i++){										// 导球*8

		RopoDevice::Chassis.MoveVelocity(0,-7.5);
		pros::delay(320);
		RopoDevice::Chassis.MoveVelocity(0,0);
		pros::delay(50);
		RopoDevice::Chassis.AutoRotateAbs(45);
		delay();
		pros::delay(250);
	}
	ControllerModule::BarRecover();
	RopoDevice::Chassis.AutoRotateAbs(180);
	delay();
	RopoDevice::Chassis.MoveVelocity(1,0);
	pros::delay(300);
	RopoDevice::Chassis.MoveVelocity(0,0);
	ControllerModule::ChangeLeftWingPush();
	ControllerModule::ChangeRightWingPush();
	pros::delay(400);
	RopoDevice::Chassis.MoveVelocity(-0.6,-1);
	pros::delay(2000);
	ControllerModule::ChangeRightWingPush();
	RopoDevice::Chassis.MoveVelocity(-0.7,0.15);
	pros::delay(1000);
	RopoDevice::Chassis.MoveVelocity(-0.5,0.15);
	pros::delay(1300);
	RopoDevice::Chassis.MoveVelocity(0,0);
	pros::delay(50);
	RopoDevice::Chassis.MoveVelocity(-0.25,-0.5);
	pros::delay(2000);
	RopoDevice::Chassis.MoveVelocity(0,0);
	pros::delay(50);
	RopoDevice::Chassis.MoveVelocity(-0.5,0);
	pros::delay(900);
	RopoDevice::Chassis.MoveVelocity(0,-1.5);
	pros::delay(200);
	ControllerModule::ChangeLeftWingPush();
	pros::delay(400);
	RopoDevice::Chassis.MoveVelocity(2,0);
	pros::delay(100);
	RopoDevice::Chassis.MoveVelocity(0,0);
	pros::delay(50);
	RopoDevice::Chassis.MoveVelocity(-2,0);
	pros::delay(400);
	RopoDevice::Chassis.MoveVelocity(0,0);
	pros::delay(100);
	RopoDevice::Chassis.MoveVelocity(2,0);
	pros::delay(200);
	RopoDevice::Chassis.AutoRotateAbs(20);
	delay();
	pros::delay(100);
	RopoDevice::Chassis.MoveVelocity(-2.0,0);
	pros::delay(400);
	RopoDevice::Chassis.MoveVelocity(0,0);
	pros::delay(100);
	RopoDevice::Chassis.MoveVelocity(2,0);
	pros::delay(200);
	RopoDevice::Chassis.MoveVelocity(0,0);
	pros::delay(100);
	RopoDevice::Chassis.MoveVelocity(0.5,1.5);
	pros::delay(500);
	RopoDevice::Chassis.AutoRotateAbs(90);
	delay();
	RopoDevice::Chassis.MoveVelocity(2,0);
	pros::delay(300);
	ControllerModule::GpsUpdate();
	RopoDevice::gpsAddPosition.SetUpdateFlag(1);
	pros::delay(200);
	RopoDevice::gpsAddPosition.SetUpdateFlag(0);

	RopoDevice::Chassis.AutoPositionMove(0.64,-2.41);
	RopoDevice::Chassis.AutoPositionMove(0.58,-1.5);
	ControllerModule::GpsUpdate();
	RopoDevice::Chassis.AutoPositionMove(0.54,-0.50,180);
	RopoDevice::Chassis.MoveVelocity(2,0);
	pros::delay(350);
	ControllerModule::GpsUpdate();
	RopoDevice::Chassis.AutoPositionMove(0.25,-0.08,45.0);

	ControllerModule::BarExtend();
	pros::delay(400);

	for(int i = 1 ; i <= 8 ; i++){										// 导球*8

		RopoDevice::Chassis.MoveVelocity(0,-7.5);
		pros::delay(320);
		RopoDevice::Chassis.MoveVelocity(0,0);
		pros::delay(50);
		RopoDevice::Chassis.AutoRotateAbs(45);
		delay();
		pros::delay(250);
	}
	ControllerModule::BarRecover();
	RopoDevice::Chassis.AutoRotateAbs(180);
	delay();
	RopoDevice::Chassis.MoveVelocity(1,0);
	pros::delay(300);
	RopoDevice::Chassis.MoveVelocity(0,0);
	ControllerModule::ChangeLeftWingPush();
	ControllerModule::ChangeRightWingPush();
	pros::delay(400);
	RopoDevice::Chassis.MoveVelocity(-0.6,-1);
	pros::delay(2000);
	ControllerModule::ChangeRightWingPush();
	RopoDevice::Chassis.MoveVelocity(-0.7,0.15);
	pros::delay(1000);
	RopoDevice::Chassis.MoveVelocity(-0.5,0.15);
	pros::delay(1300);
	RopoDevice::Chassis.MoveVelocity(0,0);
	pros::delay(50);
	RopoDevice::Chassis.MoveVelocity(-0.25,-0.5);
	pros::delay(2000);
	RopoDevice::Chassis.MoveVelocity(0,0);
	pros::delay(50);
	RopoDevice::Chassis.MoveVelocity(-0.5,0);
	pros::delay(900);
	RopoDevice::Chassis.MoveVelocity(0,-1.5);
	pros::delay(200);
	ControllerModule::ChangeLeftWingPush();
	pros::delay(400);
	RopoDevice::Chassis.MoveVelocity(2,0);
	pros::delay(100);
	RopoDevice::Chassis.MoveVelocity(0,0);
	pros::delay(50);
	RopoDevice::Chassis.MoveVelocity(-2,0);
	pros::delay(400);
	RopoDevice::Chassis.MoveVelocity(0,0);
	pros::delay(100);
	RopoDevice::Chassis.MoveVelocity(2,0);
	pros::delay(200);
	RopoDevice::Chassis.AutoRotateAbs(20);
	delay();
	pros::delay(100);
	RopoDevice::Chassis.MoveVelocity(-2.0,0);
	pros::delay(400);
	RopoDevice::Chassis.MoveVelocity(0,0);
	pros::delay(100);
	RopoDevice::Chassis.MoveVelocity(2,0);
	pros::delay(200);
	RopoDevice::Chassis.MoveVelocity(0,0);
	pros::delay(100);
	RopoDevice::Chassis.MoveVelocity(0.5,1.5);
	pros::delay(500);
	RopoDevice::Chassis.AutoRotateAbs(90);
	delay();
	RopoDevice::Chassis.MoveVelocity(2,0);
	pros::delay(300);
	ControllerModule::GpsUpdate();
	RopoDevice::gpsAddPosition.SetUpdateFlag(1);
	pros::delay(200);
	RopoDevice::gpsAddPosition.SetUpdateFlag(0);

	RopoDevice::Chassis.AutoPositionMove(0.55,-2.37);
	RopoDevice::Chassis.AutoPositionMove(0.55,-1.82);
	ControllerModule::Hang();
	ControllerModule::BarRecover();
	pros::delay(1000);
	RopoDevice::Chassis.MoveVelocity(0.5,0);
	pros::delay(500);
	ControllerModule::Hang();
	ControllerModule::BarExtend();
	RopoDevice::Chassis.MoveVelocity(0,0);

}

void autonomous_KnockoutMatch(){	// 机创赛-淘汰赛

	// 初始化部分
	RopoDevice::Chassis.StartChassisAutoControll();   			// 设置为自动状态
	RopoDevice::ChassisBrake();						  			// 自动赛时需要设置刹车状态为brake

	ControllerModule::BarExtend();								// 打开导入杆
	RopoDevice::Chassis.AutoPositionMove(1.05,0);  	 // 移动到扫球位置
	RopoDevice::Chassis.MoveVelocity(0,-5);				// 扫球
	pros::delay(500);

	ControllerModule::BarRecover();								// 收导入杆
	RopoDevice::Chassis.AutoPositionMove(0.92,-0.32,-45.0);  // 中间过渡点

	ControllerModule::Intake();	
	ControllerModule::ChangeIntakerPneumatic();				      // 弹出吃球部分
	RopoDevice::Chassis.AutoPositionMove(1.03,-0.42);  // 中间吃球点
	ControllerModule::BarExtend();								  // 打开导入杆，准备扫对面半场棕球
	pros::delay(400);

	// ControllerModule::Intake();	
	// ControllerModule::ChangeIntakerPneumatic();				      // 弹出吃球部分
	// RopoDevice::Chassis.AutoPositionMove(1.04,-0.43);  			  // 中间吃球点
	// RopoDevice::Chassis.MoveVelocity(0.1,0);
	// ControllerModule::BarExtend();
	// pros::delay(200);

	RopoDevice::Chassis.MoveVelocity(0,-4);				   // 扫对面半场棕球
	pros::delay(200);
	ControllerModule::BarRecover();
	pros::delay(200);
	RopoDevice::Chassis.MoveVelocity(0,0);

	ControllerModule::ChangeIntakerPneumatic();					  // 收回吃球部分与导入杆
	ControllerModule::BarRecover();
	
	RopoDevice::Chassis.AutoPositionMove(0.84,-0.48,-180);  // 瞄准推球方向

	ControllerModule::ChangeRightWingPush();						// 打开双翅并置为吐球模式
	ControllerModule::ChangeLeftWingPush();
	ControllerModule::Outtake();

	RopoDevice::Chassis.MoveVelocity(0.5,0);					// 开始推球，角度达到或是超时后停止
	int count = 0 ;
	while (fabs(RopoDevice::Sensors::Inertial.get_pitch()) < 18 && count <= 2500) {
	 	pros::delay(10);
		count = count + 10;
	}
	RopoDevice::Chassis.MoveVelocity(0,0);
	ControllerModule::ChangeLeftWingPush();				// 暂时收起翅膀，防止被卡
	ControllerModule::ChangeRightWingPush();
	pros::delay(200);


	RopoDevice::Chassis.MoveVelocity(-1,0);					// 后退，直到车子回到水平面
	while (fabs(RopoDevice::Sensors::Inertial.get_pitch()) > 3) {
	 	pros::delay(10);
	}
	RopoDevice::Chassis.MoveVelocity(0,0);
	RopoDevice::Chassis.AutoRotateAbs(180);			// 先转至180度，方便使用侧翼扫出角落里推不过去的球
	delay();
	ControllerModule::ChangeLeftWingPush();						// 打开翅膀，以便扫出落里推不过去的球


	RopoDevice::Chassis.AutoRotateAbs(90);			// 扫球，并且准备使用gps更新坐标
	delay();
	RopoDevice::Chassis.MoveVelocity(0,0);
	ControllerModule::GpsUpdate();
	ControllerModule::Intake();									// 打开吃球装置

	RopoDevice::Chassis.MoveVelocity(0.8,0);
	pros::delay(500);
	RopoDevice::Chassis.MoveVelocity(0,0);
	pros::delay(50);


	RopoDevice::Chassis.AutoPositionMove(0.26,0.56,-135);	// 移动到三角区前
	ControllerModule::ChangeRightWingPush();
	pros::delay(200);
	RopoDevice::Chassis.AutoPositionMove(0.00,0.25,-90);	// 将球尽可能扫到通道方向，防止前往导球点时被卡
	ControllerModule::ChangeRightWingPush();				// 收起所有翅膀
	ControllerModule::ChangeLeftWingPush();
	pros::delay(200);
	ControllerModule::GpsUpdate();


	RopoDevice::Chassis.AutoPositionMoveBack(-0.04,0.36,160);		// 导球点
	ControllerModule::BarExtend();
	pros::delay(400);

	for(int i = 1 ; i <= 7 ; i++){										// 导球*8

		RopoDevice::Chassis.MoveVelocity(0,7.5);
		pros::delay(320);
		RopoDevice::Chassis.MoveVelocity(0,0);
		pros::delay(50);
		RopoDevice::Chassis.AutoRotateAbs(160);
		delay();
		pros::delay(400);
	}

	RopoDevice::Chassis.AutoRotateAbs(-90);
	delay();
	ControllerModule::BarRecover();
	ControllerModule::ChangeRightWingPush();
	ControllerModule::ChangeLeftWingPush();
	ControllerModule::Intake();
	ControllerModule::GpsUpdate();

	RopoDevice::Chassis.AutoPositionMoveBack(-0.32,-0.10);
	RopoDevice::Chassis.MoveVelocity(0,0);
	ControllerModule::ChangeLeftWingPush();
	ControllerModule::ChangeRightWingPush();
	RopoDevice::Chassis.MoveVelocity(0,-5);
	pros::delay(100);
	RopoDevice::Chassis.MoveVelocity(0,0);
	ControllerModule::ChangeRightWingPush();
	RopoDevice::Chassis.AutoPositionMoveBack(-0.44,-0.75,90);
	RopoDevice::Chassis.MoveVelocity(-0.7,-0.1);
	pros::delay(500);
	RopoDevice::Chassis.MoveVelocity(-0.5,-0.1);
	pros::delay(1000);
	RopoDevice::Chassis.MoveVelocity(0,0);
	pros::delay(100);
	RopoDevice::Chassis.MoveVelocity(-0.25,0.5);
	pros::delay(2000);
	RopoDevice::Chassis.MoveVelocity(0,0);
	pros::delay(50);
	RopoDevice::Chassis.MoveVelocity(-0.5,0);
	pros::delay(400);
	pros::delay(500);
	RopoDevice::Chassis.MoveVelocity(0,1.5);
	pros::delay(200);
	ControllerModule::ChangeRightWingPush();
	pros::delay(400);
	RopoDevice::Chassis.MoveVelocity(2,0);
	pros::delay(100);
	RopoDevice::Chassis.MoveVelocity(0,0);
	pros::delay(50);
	RopoDevice::Chassis.MoveVelocity(-2,0);
	pros::delay(400);
	RopoDevice::Chassis.MoveVelocity(0,0);
	pros::delay(100);
	RopoDevice::Chassis.MoveVelocity(2,0);
	pros::delay(250);
	RopoDevice::Chassis.MoveVelocity(0,0);
	RopoDevice::Chassis.AutoRotateAbs(156);
	delay();
	pros::delay(100);
	RopoDevice::Chassis.MoveVelocity(-2.0,0);
	pros::delay(400);
	RopoDevice::Chassis.MoveVelocity(0,0);
	pros::delay(100);
	RopoDevice::Chassis.MoveVelocity(2,0);
	pros::delay(300);
	RopoDevice::Chassis.MoveVelocity(0,0);
	ControllerModule::IntakerStop();
}

void autonomous_KnockoutMatch_1(){ // 仅自动赛后半段测试

	/*初始化部分*/
	RopoDevice::Chassis.StartChassisAutoControll();   // 设置为自动状态
	RopoDevice::ChassisBrake();						  // 自动赛时需要设置刹车状态为brake
	RopoDevice::Chassis.AutoRotateAbs(-90);
	delay();
	ControllerModule::GpsUpdate();
	RopoDevice::Chassis.AutoPositionMoveBack(-0.04,0.36,160);
	/*end*/
	
	RopoDevice::Chassis.AutoRotateAbs(-90);
	delay();
	ControllerModule::BarRecover();
	ControllerModule::ChangeRightWingPush();
	ControllerModule::ChangeLeftWingPush();
	ControllerModule::Intake();
	ControllerModule::GpsUpdate();

	RopoDevice::Chassis.AutoPositionMoveBack(-0.44,-0.10);
	ControllerModule::ChangeLeftWingPush();
	ControllerModule::ChangeRightWingPush();
	RopoDevice::Chassis.AutoRotateAbs(90);
	delay();
	ControllerModule::ChangeRightWingPush();
	RopoDevice::Chassis.AutoPositionMoveBack(-0.44,-0.75);
	RopoDevice::Chassis.MoveVelocity(-0.7,-0.1);
	pros::delay(500);
	RopoDevice::Chassis.MoveVelocity(-0.5,-0.1);
	pros::delay(1100);
	RopoDevice::Chassis.MoveVelocity(0,0);
	pros::delay(50);
	RopoDevice::Chassis.MoveVelocity(-0.25,0.5);
	pros::delay(2000);
	RopoDevice::Chassis.MoveVelocity(0,0);
	pros::delay(50);
	RopoDevice::Chassis.MoveVelocity(-0.5,0);
	pros::delay(400);
	pros::delay(500);
	RopoDevice::Chassis.MoveVelocity(0,1.5);
	pros::delay(200);
	ControllerModule::ChangeRightWingPush();
	pros::delay(400);
	RopoDevice::Chassis.MoveVelocity(2,0);
	pros::delay(100);
	RopoDevice::Chassis.MoveVelocity(0,0);
	pros::delay(50);
	RopoDevice::Chassis.MoveVelocity(-2,0);
	pros::delay(400);
	RopoDevice::Chassis.MoveVelocity(0,0);
	pros::delay(100);
	RopoDevice::Chassis.MoveVelocity(2,0);
	pros::delay(250);
	RopoDevice::Chassis.MoveVelocity(0,0);
	RopoDevice::Chassis.AutoRotateAbs(156);
	delay();
	pros::delay(100);
	RopoDevice::Chassis.MoveVelocity(-2.0,0);
	pros::delay(400);
	RopoDevice::Chassis.MoveVelocity(0,0);
	pros::delay(100);
	RopoDevice::Chassis.MoveVelocity(2,0);
	pros::delay(300);
	RopoDevice::Chassis.MoveVelocity(0,0);
	ControllerModule::IntakerStop();
}
void PositionInit(){

	RopoDevice::Chassis.StartChassisAutoControll();
	RopoDevice::Chassis.AutoPositionMove(0,0,0);
}



