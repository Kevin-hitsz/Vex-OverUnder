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
	FloatType VelocityMax = 1.6;//1.4
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
		RopoDevice::Motors::LeftIntakeMotor.move_velocity(-500);
		RopoDevice::Motors::RightIntakeMotor.move_velocity(-500);
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
		RopoDevice::LiftMotors.Wait2();
		RopoDevice::Chassis.StartChassisAutoControll();//底盘MoveType设置为AutoMove
		//RopoDevice::Chassis.AutoRotateAbs(-90);
		//pros::delay(500);
		RopoDevice::Chassis.MoveVelocity(-0.3,-0.00);
		while(RopoDevice::Sensors::distance.get()  > 150) pros::delay(20);
		RopoDevice::Chassis.MoveVelocity(0,0);
		pros::delay(300);
		ChangeExtern();
		pros::delay(200);
		ChangeExtern();
		RopoDevice::Chassis.MoveVelocity(-0.5,-0.00);
		pros::delay(700);
		RopoDevice::Chassis.AutoRotateAbs(90);
		pros::delay(400);
		ChangeExtern();
		

		pros::delay(100);
		RopoDevice::Chassis.MoveVelocity(0,0);
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
	RopoDevice::ThreeWire::IntakerPneumatic.set_value(false);
	RopoDevice::ThreeWire::WingPneumatic.set_value(false);
}

void disabled() {}

void competition_initialize() {}

void autonomous(){
	RopoDevice::ChassisBrake();
	autonomous_c3();
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
	FloatType RopoWcLimit = 8;
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
	ButtonDetectLine.AddButtonDetect(pros::E_CONTROLLER_DIGITAL_Y , RopoController::Rising , ControllerModule::AutoLift);	
	ButtonDetectLine.AddButtonDetect(pros::E_CONTROLLER_DIGITAL_B , RopoController::Rising , ControllerModule::Switch);

	ButtonDetectLine.AddButtonDetect(pros::E_CONTROLLER_DIGITAL_UP   , RopoController::Rising,  autonomous);
	ButtonDetectLine.AddButtonDetect(pros::E_CONTROLLER_DIGITAL_LEFT , RopoController::Rising,  test);
	ButtonDetectLine.AddButtonDetect(pros::E_CONTROLLER_DIGITAL_DOWN , RopoController::Rising,  autonomous_C2);
	ButtonDetectLine.AddButtonDetect(pros::E_CONTROLLER_DIGITAL_RIGHT , RopoController::Rising,  ControllerModule::GpsUpdate);
	ButtonDetectLine.Enable();

	while (true) {
		
		FloatType XInput =  XVelocityInput.GetAxisValue();
		FloatType WInput = -WVelocityInput.GetAxisValue();
		FloatType RopoWc = RopoWcLimit-fabs(XInput) * 4.9;			
		FloatType RopoVx = ControllerModule::VelocityMax-fabs(WInput) * 0.7;	
		if(opTime - pros::millis() > 55000) {
			ControllerModule::VelocityMax = 1.77;
			RopoWcLimit = 8.5;
		}
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
	RopoDevice::Chassis.MoveVelocity(0.0,0);
	RopoDevice::gpsAddPosition.SetUpdateFlag(10);//开启gps
	pros::delay(300);
	RopoDevice::gpsAddPosition.SetUpdateFlag(10);
	RopoAuto::Auto_Find();
	RopoDevice::Chassis.MoveVelocity(-1.0,0);
	pros::delay(100);
	RopoDevice::gpsAddPosition.SetUpdateFlag(0);
	if( !(RopoDevice::GetTransformedPosition()[1] < 1.5 && RopoDevice::GetTransformedPosition()[1] > 1.2)){
		RopoDevice::Chassis.AutoPositionMoveBack(1.05,-1.5,-90);
	}
	else {
		RopoDevice::Chassis.AutoRotateAbs(-90);
		pros::delay(700);
	}
	ControllerModule::ChangeIntakerPneumatic();
	RopoDevice::gpsAddPosition.SetUpdateFlag(10);
	RopoDevice::Chassis.MoveVelocity(1.4,0);
	pros::delay(700);
	RopoDevice::Chassis.MoveVelocity(0.0,0);
	pros::delay(50);
	RopoDevice::Chassis.MoveVelocity(-1.1,0);
	pros::delay(700);
	RopoDevice::Chassis.AutoRotateAbs(90);
	RopoDevice::Chassis.MoveVelocity(0.0,0);
	ControllerModule::ChangeIntakerPneumatic();
	// RopoDevice::Chassis.AutoRotateAbs(-90);
	// while(!RopoDevice::Chassis.IfArrived()){
	// 	pros::delay(20);
	// }
	// RopoDevice::Chassis.AutoRotateAbs(90);
	// while(!RopoDevice::Chassis.IfArrived()){
	// 	pros::delay(20);
	// }

}

void delay(){		//代替pros::delay()进行时间控制
	while(!RopoDevice::Chassis.IfArrived()){
		pros::delay(15);
	}
	return;
}

void autonomous_c3(){
	RopoDevice::Chassis.StartChassisAutoControll();//底盘MoveType设置为AutoMove
	// --------- begin ------------

//---暖机，推场地中间4个球，并回到，导入位置
	RopoDevice::gpsAddPosition.SetUpdateFlag(0);
	ControllerModule::ChangeIntakerPneumatic();
	RopoDevice::ThreeWire::SpadePneumatic.set_value(true);
	pros::delay(30);
	RopoDevice::ThreeWire::SpadePneumatic.set_value(false);
	RopoDevice::Chassis.AutoDirectMove(1.1,0,0);
	delay();
	RopoDevice::gpsAddPosition.SetUpdateFlag(10);
	RopoDevice::Chassis.AutoRotateAbs(90);
	delay();
	RopoDevice::ThreeWire::SpadePneumatic.set_value(true);
	RopoDevice::Chassis.MoveVelocity(0.2,0);
	pros::delay(230);
	ControllerModule::TogetherPush();
	RopoDevice::Chassis.MoveVelocity(-1.25,0);
	pros::delay(920);
	// RopoDevice::Position_Motor::MyPosition.Set_XY(1.11, -0.70);
	RopoDevice::Chassis.MoveVelocity(0.4,0);
	pros::delay(280);
	ControllerModule::TogetherUnpush();
	RopoDevice::gpsAddPosition.SetUpdateFlag(0);
	// RopoDevice::Chassis.MoveVelocity(0.3,0);
	// pros::delay(150);
	RopoDevice::Chassis.AutoPositionMove(-0.07,0.42);//-0.01,0.38;-0.06,0.51
	delay();

//---对准角度，通过转动导入球
	
	RopoDevice::gpsAddPosition.SetUpdateFlag(10);
	// RopoDevice::Chassis.AutoRotateAbs(135);
	// delay();
	RopoDevice::Chassis.MoveVelocity(0.35,0);		//靠杆
	pros::delay(100);
	RopoDevice::Chassis.MoveVelocity(0,0);
	ControllerModule::Lift();
	pros::delay(700);
	for (int i = 1; i <= 6; i++) {		//此处选择开环加闭环的方式
		// RopoDevice::Chassis.AutoRotateAbs(182);
		// while(!RopoDevice::Chassis.IfArrived()){
		// 	pros::delay(20);
		// }
		RopoDevice::Chassis.MoveVelocity(0,2.45);
		pros::delay(480);
		RopoDevice::Chassis.MoveVelocity(0,0);
		RopoDevice::Chassis.AutoRotateAbs(135);
		delay();
		RopoDevice::Chassis.MoveVelocity(0.35,0);		//旋转后前进靠杆
		pros::delay(250);
		RopoDevice::Chassis.MoveVelocity(0,0);
		pros::delay(20);
	}

//---调整位置，对准通道（由于GPS实装，正在考虑用闭环代替）

	ControllerModule::Hide();
	// pros::delay(200);
	RopoDevice::Chassis.MoveVelocity(-0.4,0);
	pros::delay(100);
	RopoDevice::Chassis.MoveVelocity(0,0);
	RopoDevice::Chassis.AutoRotateAbs(45);
	delay();
	RopoDevice::Chassis.MoveVelocity(-0.50,0);
	pros::delay(700);
	RopoDevice::Chassis.MoveVelocity(0,0);
	// RopoDevice::Chassis.AutoPositionMove(-0.26,-0.07,90);
	// delay();
	RopoDevice::Chassis.AutoRotateAbs(91);
	delay();

//---直线加曲线推球入网（考虑加入GPS以精准控制）

	ControllerModule::TogetherPush();
	RopoDevice::Chassis.MoveVelocity(-0.75,0);
	while (RopoDevice::Position_Motor::MyPosition.Get_Y() > -1.45) {
		pros::delay(15);
	}
	// pros::delay(2410); v = 0.8
	RopoDevice::Chassis.MoveVelocity(0,0);
	//RopoDevice::Chassis.AutoRotateAbs(90);
	RopoDevice::Chassis.MoveVelocity(-0.67,0.95);
	pros::delay(1650);
	ControllerModule::TogetherUnpush();
	pros::delay(500);
	RopoDevice::Chassis.AutoRotateAbs(180);
	RopoDevice::Chassis.MoveVelocity(0.6,0);
	pros::delay(600);
	RopoDevice::Chassis.MoveVelocity(-0.75,0);
	pros::delay(500);
	RopoDevice::Chassis.MoveVelocity(0,0);
	// RopoDevice::Chassis.MoveVelocity(0.5,0);
	// pros::delay(600);
	// RopoDevice::Chassis.AutoRotateAbs(-30);
	// pros::delay(500);
	// RopoDevice::Chassis.MoveVelocity(-0.7,0.3);
	// pros::delay(600);


//---开到三角区，勾出联队粽球并送入网（由于GPS实装，正在考虑用闭环代替）

	RopoDevice::Chassis.AutoRotateAbs(175);
	delay();
	RopoDevice::Chassis.MoveVelocity(0.55,0.20);
	pros::delay(2000);
	RopoDevice::Chassis.MoveVelocity(0,0);
	RopoDevice::gpsAddPosition.SetUpdateFlag(0);		//停用GPS并设置相对坐标
	pros::delay(150);
	ControllerModule::ChangeIntakerPneumatic();
	RopoDevice::Position_Motor::MyPosition.Set_XY(0, 0);
	pros::delay(150);
	ControllerModule::Lift();
	pros::delay(800);
	ControllerModule::Intake();
	RopoDevice::Chassis.MoveVelocity(-0.4,0);
	pros::delay(800);
	RopoDevice::Chassis.MoveVelocity(0,0);
	ControllerModule::Hide();
	pros::delay(300);
	RopoDevice::Chassis.MoveVelocity(0.3,0);
	pros::delay(300);
	RopoDevice::Chassis.MoveVelocity(0,0);
	RopoDevice::Chassis.AutoRotateAbs(-45);
	delay();
	RopoDevice::Chassis.MoveVelocity(0.55,0.5);
	pros::delay(600);
	RopoDevice::Chassis.AutoRotateAbs(0);
	delay();
	ControllerModule::IntakerStop();
	ControllerModule::ChangeIntakerPneumatic();
	RopoDevice::gpsAddPosition.SetUpdateFlag(10);
	RopoDevice::Chassis.MoveVelocity(0.7,0);
	pros::delay(750);
	RopoDevice::Chassis.MoveVelocity(0,0);

//---回过道碰爬升杆

	RopoDevice::Chassis.MoveVelocity(-0.62,-0.9);
	pros::delay(2050);
	RopoDevice::Chassis.MoveVelocity(0,0);
	pros::delay(30);
	RopoDevice::Chassis.AutoRotateAbs(-90);
	RopoDevice::Chassis.MoveVelocity(-0.6,0);
	pros::delay(1600);
	RopoDevice::Chassis.MoveVelocity(0,0);
	pros::delay(30);
	// RopoDevice::Chassis.AutoPositionMoveBack(-0.23,-0.61);
	// pros::delay(5000);
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

void autonomous_C2(){
	RopoDevice::ChassisBrake();
	RopoDevice::Chassis.StartChassisAutoControll();//底盘MoveType设置为AutoMove
	//start--推己方四球
	RopoDevice::gpsAddPosition.SetUpdateFlag(0);//关闭gps
	RopoDevice::ThreeWire::SpadePneumatic.set_value(true);
	pros::delay(30);
	RopoDevice::ThreeWire::SpadePneumatic.set_value(false);
	ControllerModule::Intake();
	RopoDevice::Chassis.AutoDirectMove(1.10,-0.00,0);
	while(!RopoDevice::Chassis.IfArrived()){
		pros::delay(20);
	}
	RopoDevice::Chassis.AutoRotateAbs(87);
	pros::delay(800);
	RopoDevice::ThreeWire::SpadePneumatic.set_value(true);
	RopoDevice::Chassis.MoveVelocity(0.3,0);
	pros::delay(300);
	ControllerModule::TogetherPush();
	RopoDevice::Chassis.MoveVelocity(-0.7,0);
	pros::delay(1800);

	//----------------------转身越障，推球入网
	RopoDevice::Chassis.MoveVelocity(0.0,0);
	pros::delay(100);
	RopoDevice::Chassis.MoveVelocity(0.8,0);
	pros::delay(300);
	RopoDevice::Chassis.MoveVelocity(0.0,0);
	pros::delay(150);
	RopoDevice::Chassis.AutoRotateAbs(-90);
	while(!RopoDevice::Chassis.IfArrived())pros::delay(20);
	RopoDevice::Chassis.MoveVelocity(1.3,0);
	ControllerModule::ChangeIntakerPneumatic();//收
	//ControllerModule::Intake();
	RopoDevice::Chassis.MoveVelocity(0.6,0);
	pros::delay(800);
	//越过后进行转向，多推球入网
	RopoDevice::Chassis.AutoRotateAbs(-70);
	pros::delay(500);
	RopoDevice::Chassis.MoveVelocity(0.2,0.7);
	pros::delay(800);
	RopoDevice::Chassis.MoveVelocity(0,0);
	pros::delay(100);
	RopoDevice::Chassis.MoveVelocity(0.43,-0.15);
	pros::delay(500);
	
	RopoDevice::Chassis.MoveVelocity(0.3,-1.2);
	pros::delay(100);
	pros::delay(500);
	
	RopoDevice::Chassis.AutoRotateAbs(-95);
	pros::delay(200);
	
	RopoDevice::gpsAddPosition.SetUpdateFlag(3);//开启gps
	RopoDevice::Chassis.MoveVelocity(1.0,-0.2);
	pros::delay(1000);

	//从网里退出吃两个球
	ControllerModule::TogetherUnpush();
	RopoDevice::Chassis.AutoRotateAbs(-85);
	pros::delay(1000);
	RopoDevice::Chassis.MoveVelocity(-1.2,0);
	pros::delay(1000);
	RopoDevice::Chassis.MoveVelocity(0,0);
	pros::delay(200);
	RopoDevice::gpsAddPosition.SetUpdateFlag(0);//关闭gps
	RopoDevice::Chassis.AutoRotateAbs(0);
	pros::delay(500);
	//吃中立区45°球
	
	ControllerModule::ChangeIntakerPneumatic();//放
	ControllerModule::Intake();
	RopoDevice::Chassis.AutoPositionMove(1.28,-1.27,45);
	RopoDevice::Chassis.MoveVelocity(0.0,0);
	ControllerModule::ChangeLift();
	pros::delay(200);
	RopoDevice::Chassis.MoveVelocity(0.0,0);
	pros::delay(1800);
	RopoDevice::Chassis.MoveVelocity(-0.4,0);
	pros::delay(400);
	
	RopoDevice::Chassis.AutoRotateAbs(90);
	pros::delay(600);
	ControllerModule::ChangeLift();
	while(!RopoDevice::Chassis.IfArrived())pros::delay(20);
	RopoDevice::Chassis.MoveVelocity(0.3,0);
	pros::delay(200);
	RopoDevice::Chassis.MoveVelocity(0.0,0);
	pros::delay(100);
	ControllerModule::WingPush();
	RopoDevice::gpsAddPosition.SetUpdateFlag(10);//开启gps
	RopoDevice::Chassis.MoveVelocity(-1.0,0);
	pros::delay(1050);
	RopoDevice::Chassis.MoveVelocity(0.0,0);
	pros::delay(100);
	RopoDevice::Chassis.MoveVelocity(1.0,0);
	pros::delay(500);
	RopoDevice::Chassis.MoveVelocity(0.0,0);
	pros::delay(100);
	ControllerModule::WingUnpush();
	RopoDevice::Chassis.AutoRotateAbs(-100);
	while(!RopoDevice::Chassis.IfArrived())pros::delay(20);
	ControllerModule::ChangeIntakerPneumatic();//收
	RopoDevice::Chassis.MoveVelocity(1.0,0);
	pros::delay(700);
	RopoDevice::Chassis.MoveVelocity(0.0,0);
	pros::delay(100);
	RopoDevice::Chassis.AutoRotateAbs(-89);
	pros::delay(700);
	RopoDevice::Chassis.MoveVelocity(-1.0,0);
	pros::delay(750);
	RopoDevice::Chassis.MoveVelocity(0.0,0);
	pros::delay(100);
	RopoDevice::Chassis.MoveVelocity(-0.6,2);
	pros::delay(600);
	RopoDevice::Chassis.MoveVelocity(-1.3,0);
	pros::delay(200);
	RopoDevice::Chassis.MoveVelocity(1.3,0);
	pros::delay(200);
	ControllerModule::ChangeIntakerPneumatic();//放
	//自动吃球
	//test();
	RopoDevice::Chassis.AutoPositionMove(0.36,-1.3,135);
	ControllerModule::Intake();
	RopoDevice::Chassis.MoveVelocity(0.6,0);
	pros::delay(300);
	RopoDevice::Chassis.MoveVelocity(0.0,0);
	pros::delay(100);
	RopoDevice::Chassis.AutoPositionMoveBack(1.22,-1.3,90);
	ControllerModule::WingPush();
	RopoDevice::Chassis.MoveVelocity(-1.1,0);
	pros::delay(700);
	RopoDevice::Chassis.MoveVelocity(0.0,0);
	pros::delay(100);
	RopoDevice::Chassis.MoveVelocity(1.0,0);
	pros::delay(750);
	RopoDevice::Chassis.MoveVelocity(0.0,0);
	pros::delay(100);
	RopoDevice::Chassis.MoveVelocity(0.6,2);
	pros::delay(400);
	//碰杆
	RopoDevice::Chassis.AutoPositionMove(0.36,-1.3,135);
	ControllerModule::ChangeLift();//放杆
	RopoDevice::Chassis.MoveVelocity(0.1,0);
	pros::delay(300);
	

	RopoDevice::Chassis.MoveVelocity(0.0,0);
	pros::delay(150);
}

void skill(){
	RopoDevice::Chassis.StartChassisAutoControll();//底盘MoveType设置为AutoMove
	// --------- begin ------------
}
