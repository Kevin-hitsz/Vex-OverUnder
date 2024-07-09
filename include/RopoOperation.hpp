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
#include "RoPoPros/RopoController.hpp"
#include "RopoDevice.hpp"
#include "RopoGpsAddPosition.hpp"
#include "RopoMath/Misc.hpp"
#include "RopoPosition.hpp"
#include "pros/misc.h"
#include "pros/rtos.hpp"
#include "RopoOperation.hpp"
#include <cmath>

void reset_main();
void block_main();

/// @brief 手柄操作定义
namespace ControllerModule {	

	void RumbleMe(){
		pros::Controller MasterController1(pros::E_CONTROLLER_MASTER);
		MasterController1.rumble("-.-.-");
		//设备启动时间
		FloatType ini_time = pros::millis();
		while(pros::millis()-ini_time<45000){
			pros::delay(500);
		}
		
		MasterController1.rumble("-.-");
		while(pros::millis()-ini_time<55000){
			pros::delay(500);
		}
		MasterController1.rumble("-.-.-");
		while(pros::millis()-ini_time<65000){
			pros::delay(500);
		}
		MasterController1.rumble("-.-.-.-");
	}

	void GpsUpdate(){
		RopoDevice::gpsAddPosition.GpsUpdate();
	}
	void ControllerPrint()
    {
		pros::Controller MasterController(pros::E_CONTROLLER_MASTER);
		while(true) 
        {
			MasterController.print(0,0,"%.2f,%.2f,%.1f ",RopoDevice::Position_Motor::MyPosition.Get_X()
			,RopoDevice::Position_Motor::MyPosition.Get_Y(),RopoDevice::Position_Motor::MyPosition.Get_Angle());
			pros::delay(200); 	
		}
	}
	//中断测试任务
	void Test_Task()
	{
		RopoDevice::Chassis.AutoRotateAbs(90);
	}

	//中断main函数并执行Task
	inline void InterruptMain_doTask()
	{
		//中断
		block_main();
		RopoDevice::Chassis.StartChassisAutoControll();
		pros::delay(500);
		//执行任务
		Test_Task();
		//恢复main
		pros::delay(500);

		RopoDevice::Chassis.StartChassisOpControll();
		reset_main();
	}
}



/// @brief 定义自动程序
namespace AutoOperation{
void Auto(){

}
void skill(){
}
}


/// @brief 其他
void delay(){
	while(!RopoDevice::Chassis.IfArrived()){pros::delay(20);}
	return;
}

void delayDeg(){
	double aa = pros::millis();
	while(!RopoDevice::Chassis.IfDegArrived() && aa - pros::millis() < 2500)pros::delay(20);
}

void TurnAround(){
	RopoDevice::Chassis.AutoRotateRelative(180);
}

bool main_process=true;   //是否中断主线程
//阻塞主线程
inline void block_main()
{
	main_process=false;
}
//恢复主线程
inline void reset_main()
{
	main_process=true;
}

#endif