# pragma once

#include "main.h"
#include "RopoApi.hpp"
#include "RopoController.hpp"
#include "RopoControllerModule.hpp"
#include "RopoDevice.hpp"
#include "RopoGpsAddPosition.hpp"
#include "RopoMath/Misc.hpp"
#include "RopoPosition.hpp"
#include "pros/misc.h"
#include "pros/rtos.hpp"
#include <algorithm>
#include <cmath>

namespace RopoAutonomous {

    void delay(){		//代替pros::delay()进行时间控制
	while(!RopoDevice::Chassis.IfArrived()){
		pros::delay(5);
	}
	return;
    }

	void Final_KnockoutMatch(){

		RopoDevice::Chassis.StartChassisAutoControll();   			// 设置为自动状态
		RopoDevice::ChassisBrake();						  			// 自动赛时需要设置刹车状态为brake

		ControllerModule::BarExtend();
		pros::delay(400);
		RopoDevice::Chassis.MoveVelocity(0,7.5);
		pros::delay(250);
		RopoDevice::Chassis.MoveVelocity(0,0);
		ControllerModule::BarRecover();
		ControllerModule::ChangeRightWingPush();
		ControllerModule::ChangeLeftWingPush();
		pros::delay(400);
		RopoDevice::Chassis.AutoPositionMoveBack(-0.06,0.52);
		ControllerModule::ChangeLeftWingPush();
		ControllerModule::ChangeRightWingPush();
		pros::delay(400);
		RopoDevice::Chassis.AutoRotateAbs(-70);
		delay();
		ControllerModule::ChangeRightWingPush();
		RopoDevice::Chassis.MoveVelocity(-0.7,-0.1);
		pros::delay(1800);
		RopoDevice::Chassis.MoveVelocity(-0.5,-0.1);
		pros::delay(750);
		RopoDevice::Chassis.MoveVelocity(-0.5,1);
		pros::delay(1100);
		RopoDevice::Chassis.MoveVelocity(-0.5,-0.1);
		pros::delay(400);

		RopoDevice::Chassis.MoveVelocity(0,0);
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
		pros::delay(350);
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
	pros::delay(1200);
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

	RopoDevice::Chassis.AutoPositionMove(0.58,-2.41);
	RopoDevice::Chassis.AutoPositionMove(0.58,-1.5);
	ControllerModule::GpsUpdate();
	RopoDevice::Chassis.AutoPositionMove(0.54,-0.50,180);
	RopoDevice::Chassis.MoveVelocity(2,0);
	pros::delay(350);
	ControllerModule::GpsUpdate();
	RopoDevice::Chassis.AutoPositionMove(0.49,-0.89,45.0);

	ControllerModule::BarExtend();
	pros::delay(400);

	for(int i = 1 ; i <= 8 ; i++){										// 导球*8

		RopoDevice::Chassis.MoveVelocity(0,-7.5);
		pros::delay(350);
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
	pros::delay(1200);
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

}