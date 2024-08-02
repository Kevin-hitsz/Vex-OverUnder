/**
 *  fileName:RopoOperation.hpp
 *  description:
 *  常见手柄按键操作定义,自动赛操作定义,其他杂项操作定义等
 * 
*/

#ifndef ROPO_OPERATION_HPP
#define ROPO_OPERATION_HPP

#include "main.h"
#include "RopoApi.hpp"
#include "RopoController.hpp"
#include "RopoDevice.hpp"
#include "RopoGpsAddPosition.hpp"
#include "RopoMath/Misc.hpp"
#include "RopoPosition.hpp"
#include "pros/misc.h"
#include "pros/rtos.hpp"
#include "RopoOperation.hpp"
#include <cmath>



/// @brief 手柄操作定义
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

	void Wait2(){
		RopoDevice::LiftMotors.Wait2();
		catch_1 = 3;
	}

	void ChangeLift(){
		if (catch_1 == 1 || catch_1 == 3) {
			Hide();
		} else {
			Hold();
		}
	}

	void ChangeLift1(){
		if (catch_1 == 1 || catch_1 == 3) {
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
		ControllerModule::Wait2();
		RopoDevice::Chassis.StartChassisAutoControll();//底盘MoveType设置为AutoMove
		RopoDevice::ChassisBrake();
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
		pros::delay(200);
		ChangeExtern();

	}

	void ControllerPrint()
    {
		while(true) 
        {
			pros::Controller MasterController(pros::E_CONTROLLER_MASTER);
			MasterController.print(0,1,"X: %.2lf Y:%.2lf   ",(RopoDevice::GetTransformedPosition())[1],(RopoDevice::GetTransformedPosition())[2]);
			pros::delay(50); 
			MasterController.print(1,1,"degree: %.3lf    ",RopoDevice::GetPosition()[3]);
			pros::delay(50); 
			MasterController.print(2,1,"Extern:%s",externFlag?"yes":"no");
			pros::delay(50);
		}
	}
}



/// @brief 定义自动程序
namespace AutoOperation{
void test(){
	RopoDevice::Chassis.StartChassisAutoControll();//底盘MoveType设置为AutoMove
	RopoDevice::ChassisBrake();
	RopoDevice::Chassis.AutoRotateAbs(90);
	while(!RopoDevice::Chassis.IfDegArrived() )pros::delay(20);
	RopoDevice::Chassis.AutoRotateAbs(-90);
	while(!RopoDevice::Chassis.IfDegArrived() )pros::delay(20);
}


void autonomous_C1b(){
	RopoDevice::ChassisBrake();
	RopoDevice::Chassis.StartChassisAutoControll();//底盘MoveType设置为AutoMove
	RopoDevice::gpsAddPosition.SetUpdateFlag(0);//关闭gps
	
 	//start--导己方四球
	ControllerModule::ChangeLift();//放
	pros::delay(10);
	ControllerModule::ChangeIntakerPneumatic0();
	pros::delay(800);
	for(int i = 0; i < 5; i++){
		RopoDevice::Chassis.MoveVelocity(0.0,3.6);
		pros::delay(430);
		ControllerModule::ChangeLift();
		RopoDevice::Chassis.MoveVelocity(0.0,0.0);
		pros::delay(40);
		RopoDevice::Chassis.MoveVelocity(-1.0,0);
		pros::delay(100);
		RopoDevice::Chassis.MoveVelocity(0.0,0.0);
		pros::delay(40);
		RopoDevice::Chassis.AutoRotateAbs(-8);
		pros::delay(650);
		ControllerModule::ChangeLift();
		pros::delay(100);
		RopoDevice::Chassis.MoveVelocity(0.33,0.05);
		pros::delay(400);
	}
	RopoDevice::Position_Motor::MyPosition.Set_XY(0, 0);//重置原点
	RopoDevice::Chassis.AutoRotateAbs(50);
	pros::delay(600);
	ControllerModule::ChangeLift();//收
	RopoDevice::Chassis.AutoRotateAbs(-90);//-91
	pros::delay(900);
	
	RopoDevice::Chassis.MoveVelocity(-0.55,0.1);
	pros::delay(560);
	RopoDevice::Chassis.AutoRotateAbs(-52);//-51
	pros::delay(650);
	RopoDevice::Chassis.MoveVelocity(-0.8,0.03);//过通道//-0.02
	pros::delay(2500);
	RopoDevice::Chassis.AutoDirectMove(-1.38,1.58,1);//过道//车头对x//y 1.65

	RopoDevice::gpsAddPosition.SetUpdateFlag(1);
	ControllerModule::WingPush();//开翅膀

	RopoDevice::Chassis.AutoRotateAbs(-40);//过场地四角
	pros::delay(700);

	RopoDevice::Chassis.MoveVelocity(-0.5,0.9);
	pros::delay(750);

	RopoDevice::Chassis.MoveVelocity(-0.5,0);
	pros::delay(400);//四角到球门前

	//弧线推球
	RopoDevice::Chassis.MoveVelocity(-0.6,0.8);
	pros::delay(750);

	RopoDevice::Chassis.AutoRotateAbs(45);
	pros::delay(900);

	ControllerModule::WingUnpush();//收翅膀
	pros::delay(200);
	RopoDevice::Chassis.MoveVelocity(0.4,0);
	pros::delay(200);
	RopoDevice::Chassis.MoveVelocity(0.0,0);
	pros::delay(50);
	//推进网
	RopoDevice::Chassis.MoveVelocity(-1.5,0.0);
	pros::delay(320);//360
	//退出网
	RopoDevice::Chassis.MoveVelocity(0.5,0);
	pros::delay(450);

	// RopoDevice::Chassis.AutoRotateAbs(-55);
	// pros::delay(1000);
	RopoDevice::gpsAddPosition.SetUpdateFlag(1);
	pros::delay(400);
	RopoDevice::gpsAddPosition.SetUpdateFlag(0);
	RopoDevice::Chassis.AutoPositionMove(-1.75,1.86,10000,4000);//-1.88 1.66//-1.74 1.90//-1.84 1.88//-1.66
	RopoDevice::Chassis.AutoRotateAbs(89);
	pros::delay(730);

	RopoDevice::Chassis.MoveVelocity(0.24,0);
	pros::delay(550);
	RopoDevice::Chassis.MoveVelocity(-0.24,0);
	pros::delay(150);
	RopoDevice::Chassis.MoveVelocity(0,0);
	pros::delay(400);
	//勾球
	ControllerModule::ChangeLift();
	pros::delay(700);
	
	RopoDevice::Chassis.MoveVelocity(0.0,1.0);//
	pros::delay(300);//
	RopoDevice::Chassis.AutoRotateAbs(133);
	pros::delay(700);
	ControllerModule::ChangeLift();
	pros::delay(500);
	RopoDevice::Chassis.AutoRotateAbs(170);
	pros::delay(800);
	
	//怼
	RopoDevice::Chassis.MoveVelocity(0.5,1.0);
	pros::delay(750);
	RopoDevice::Chassis.AutoRotateAbs(-135);
	pros::delay(700);
	RopoDevice::Chassis.MoveVelocity(1.1,0);
	pros::delay(300);
	
	//退出网
	RopoDevice::Chassis.MoveVelocity(0.0,0.0);
	pros::delay(50);
	RopoDevice::Chassis.MoveVelocity(-1.0,-0.9);
	pros::delay(510);//700
	RopoDevice::Chassis.MoveVelocity(0.0,0.0);
	pros::delay(50);

	RopoDevice::gpsAddPosition.SetUpdateFlag(0);//关


 	//中间吃球+推球
	ControllerModule::Intake();
	RopoDevice::Chassis.AutoPositionMove(-1.78,0.15,10000,4500);//定位有点问题//y 0.20//0.05 //正中心位//-1.53,0.40
	RopoDevice::gpsAddPosition.SetUpdateFlag(1);
	RopoDevice::Chassis.MoveVelocity(0.3,1.0);
	// pros::delay(600);
	RopoDevice::Chassis.AutoRotateAbs(-46);//-45
	pros::delay(700);
	ControllerModule::WingPush();
	RopoDevice::Chassis.MoveVelocity(0.6,0);
	pros::delay(200);
	
	RopoDevice::Chassis.MoveVelocity(-1.1,0);
	pros::delay(980);
	RopoDevice::Chassis.MoveVelocity(0.0,0);
	pros::delay(50);
	RopoDevice::Chassis.MoveVelocity(1.0,0);
	pros::delay(500);
	ControllerModule::WingUnpush();
	RopoDevice::Chassis.MoveVelocity(0.0,0);
	pros::delay(200);
	RopoDevice::Chassis.MoveVelocity(0.0,-4);
	pros::delay(500);
	RopoDevice::Chassis.AutoRotateAbs(135);
	pros::delay(800);
	
	RopoDevice::Chassis.MoveVelocity(0.0,0);
	pros::delay(50);
	RopoDevice::Chassis.MoveVelocity(1.2,0);
	ControllerModule::ChangeIntakerPneumatic0();//收
	pros::delay(440);
	RopoDevice::Chassis.MoveVelocity(0.0,0);
	pros::delay(100);
	// --------------

	//方案1 吃单个球
	RopoDevice::Chassis.MoveVelocity(-0.6,1.5);//推完 曲线后退/
	pros::delay(700);
	ControllerModule::ChangeIntakerPneumatic();//放
	ControllerModule::Intake();
	RopoDevice::Chassis.AutoRotateAbs(0);//吃单个的球
	pros::delay(800);
	RopoDevice::Chassis.MoveVelocity(1.0,0);
	pros::delay(550);
	RopoDevice::Chassis.AutoRotateAbs(-45);//送进网里
	pros::delay(600);
	RopoDevice::Chassis.MoveVelocity(-0.5,1.8);
	pros::delay(800);
	RopoDevice::Chassis.MoveVelocity(-0.9,0);
	pros::delay(350);
	RopoDevice::Chassis.AutoRotateAbs(135);//对准网
	pros::delay(800);
	ControllerModule::ChangeIntakerPneumatic0();//收
	RopoDevice::Chassis.MoveVelocity(0.8,0);//顶
	pros::delay(663);
	RopoDevice::Chassis.MoveVelocity(-1.0,0);
	pros::delay(700);
	RopoDevice::Chassis.MoveVelocity(0.0,0);
	pros::delay(100); 
	//碰提升杆 还有点问题
	RopoDevice::Chassis.AutoRotateAbs(13);
	pros::delay(800); 
	RopoDevice::Chassis.MoveVelocity(0.8,0);
	pros::delay(200); 
	ControllerModule::ChangeLift();
	pros::delay(50);
}



void delay(){
	while(!RopoDevice::Chassis.IfArrived()){pros::delay(20);}
	return;
}
void skill(){
	RopoDevice::Chassis.StartChassisAutoControll();//底盘MoveType设置为AutoMove
	// --------- begin ------------
	RopoDevice::ChassisBrake();
	RopoDevice::gpsAddPosition.SetUpdateFlag(0);//关闭gps
	//	start 勾球
	//	导己方13球
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

	// RopoDevice::gpsAddPosition.SetUpdateFlag(1);


	RopoDevice::Chassis.AutoRotateAbs(-38);
	pros::delay(500);
	// RopoDevice::Chassis.MoveVelocity(0.0,0);
	// pros::delay(400);
	// RopoDevice::Chassis.MoveVelocity(-1.4,0.0);
	// pros::delay(300);

	//弧线推球

	RopoDevice::Chassis.MoveVelocity(-0.5,0.9);
	pros::delay(750);



	RopoDevice::Chassis.MoveVelocity(-0.5,0);
	pros::delay(380);

	
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
	RopoDevice::Chassis.AutoPositionMove(-1.55,1.66,10000,4000);
	RopoDevice::Chassis.AutoRotateAbs(-45);
	pros::delay(800);
	// RopoDevice::Chassis.AutoDirectMove(0.15,0.11,0);// -0.3,0.3,0
	// pros::delay(300);
	// delay();
	RopoDevice::Chassis.MoveVelocity(3,-0.1);
	pros::delay(1280);
	RopoDevice::Chassis.MoveVelocity(0.8,-3);
	pros::delay(450);
	RopoDevice::Chassis.MoveVelocity(0.0,0.0);
	pros::delay(100);
	RopoDevice::Chassis.AutoRotateAbs(0);
	pros::delay(800);
	RopoDevice::Chassis.MoveVelocity(0.5,0);
	pros::delay(500);
	RopoDevice::Chassis.MoveVelocity(0.0,0.0);
	pros::delay(300);

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
	pros::delay(640);
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
	pros::delay(920);
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
	pros::delay(440);

	RopoDevice::Chassis.MoveVelocity(0.6,0);
	pros::delay(600);
	RopoDevice::Chassis.AutoRotateAbs(35);
	pros::delay(400);
	RopoDevice::Chassis.MoveVelocity(-1.5,1);
	pros::delay(500);
	RopoDevice::Chassis.MoveVelocity(0.6,0);
	pros::delay(700);
	//结束
	ControllerModule::IntakerStop();
	RopoDevice::Chassis.MoveVelocity(0.0,0);
	pros::delay(50);
}}




void delayDeg(){
	double aa = pros::millis();
	while(!RopoDevice::Chassis.IfDegArrived() && aa - pros::millis() < 2500)pros::delay(20);
}




#endif
