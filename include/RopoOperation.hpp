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
#include <cmath>

void reset_main();
void block_main();

namespace ControllerModule {	
	
    bool right_wing_pneumaticvalue = false;
    bool climb_Pneumaticvalue = false;
    bool left_wing_pneumaticvalue = false;
    bool lead_ball_pneumaticvalue = false;
    void open_right_wing()
    {
        RopoDevice::ThreeWire::right_wing_pneumatic.set_value(true);
        right_wing_pneumaticvalue = true;
    }

    void close_right_wing()
    {
        RopoDevice::ThreeWire::right_wing_pneumatic.set_value(false);
        right_wing_pneumaticvalue = false;
    }

    void switch_right_wing()
    {
        right_wing_pneumaticvalue ^= 1;
        RopoDevice::ThreeWire::right_wing_pneumatic.set_value(right_wing_pneumaticvalue);
    }

    void open_left_wing()
    {
        RopoDevice::ThreeWire::left_wing_pneumatic.set_value(true);
        left_wing_pneumaticvalue = true;
    }

    void close_left_wing()
    {
        RopoDevice::ThreeWire::left_wing_pneumatic.set_value(false);
        left_wing_pneumaticvalue = false;
    }

    void switch_left_wing()
    {
        left_wing_pneumaticvalue ^= 1;
        RopoDevice::ThreeWire::left_wing_pneumatic.set_value(left_wing_pneumaticvalue);
    }

    void open_both_wing()
    {
        RopoDevice::ThreeWire::right_wing_pneumatic.set_value(true);
        RopoDevice::ThreeWire::left_wing_pneumatic.set_value(true);
        right_wing_pneumaticvalue = true;
        left_wing_pneumaticvalue = true;
    }

    void close_both_wing()
    {
        RopoDevice::ThreeWire::right_wing_pneumatic.set_value(false);
        RopoDevice::ThreeWire::left_wing_pneumatic.set_value(false);
        right_wing_pneumaticvalue = false;
        left_wing_pneumaticvalue = false;
    }

    void switch_both_wing()
    {
        right_wing_pneumaticvalue ^= 1;
        left_wing_pneumaticvalue ^= 1;
        RopoDevice::ThreeWire::right_wing_pneumatic.set_value(right_wing_pneumaticvalue);
        RopoDevice::ThreeWire::left_wing_pneumatic.set_value(left_wing_pneumaticvalue);
    }

    void open_climber()
    {
        RopoDevice::ThreeWire::climb_Pneumatic.set_value(true);
        climb_Pneumaticvalue = true;
    }

    void close_climber()
    {
        RopoDevice::ThreeWire::climb_Pneumatic.set_value(false);
        climb_Pneumaticvalue = false;
    }

    void switch_climber()
    {
        climb_Pneumaticvalue ^= 1;
        RopoDevice::ThreeWire::climb_Pneumatic.set_value(climb_Pneumaticvalue);
    }
    
    void intake()
    {
        RopoDevice::intaker.move_velocity(-200);
    }

    void outtake()
    {
        RopoDevice::intaker.move_velocity(200);
        
    }

    void intaker_stop()
    {
        RopoDevice::intaker.move_voltage(0);
    }
	
    void open_lead_ball()
    {
        RopoDevice::ThreeWire::lead_ball_pneumatic.set_value(true);
        lead_ball_pneumaticvalue = true;
    }

    void close_lead_ball()
    {
        RopoDevice::ThreeWire::lead_ball_pneumatic.set_value(false);
        lead_ball_pneumaticvalue = false;
    }

 void switch_lead_ball()
    {
        lead_ball_pneumaticvalue ^= 1; 
        RopoDevice::ThreeWire::lead_ball_pneumatic.set_value(lead_ball_pneumaticvalue);
    }


    void lead_ball()
    {
        for(int i=1;i<=3;i++)
        {
            pros::delay(500);
            RopoDevice::Chassis.AutoRotateAbs_block(131.9);
            RopoDevice::Chassis.AutoRotateAbs_block(39.5);
        }
    }

    void pos_reset()
    {
        RopoDevice::Position_Motor::MyPosition.initial();
    }

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
	
}


/// @brief 定义自动程序

void Auto(){
	//初始化
	RopoDevice::Chassis.StartChassisAutoControll();

    RopoDevice::Chassis.AutoPositionMove(1.2,0,-90);//前进到场地中段，并转90面向球
    RopoDevice::Chassis.AutoPositionMove(1.2,-0.35);//向前走一点
    ControllerModule::open_both_wing();//开翅膀
    pros::delay(500);
    RopoDevice::Chassis.AutoRotateAbs_block(180);//扫球
    pros::delay(1000);
    RopoDevice::Chassis.MoveVelocity(1,0);//把球推到过道//推不过去
    pros::delay(450);
    ControllerModule::close_both_wing();//关翅膀

    //后面加斜坡 用斜坡撞球
    RopoDevice::Chassis.AutoRotateAbs(0);
    pros::delay(800);
    RopoDevice::Chassis.MoveVelocity(-1,0.9);
    pros::delay(300);
    RopoDevice::Chassis.MoveVelocity(1,-0.9);
    pros::delay(300);

    //RopoDevice::Chassis.AutoPositionMoveBack(0.84,-0.36);//倒退一段距离
    RopoDevice::Chassis.MoveVelocity(1,0);
    pros::delay(300);


    RopoDevice::Chassis.AutoPositionMove(-0.10,0.40,39.5);//到达导球点//-0.06 0.53//不够靠近导入区//-0.06 0.44

    ControllerModule::lead_ball();//导球
    RopoDevice::Chassis.AutoPositionMove(-0.25,0.4);//到达推球点
    RopoDevice::Chassis.AutoPositionMove(-0.24,-1.85);//向前推球
    ControllerModule::open_left_wing();//开左翅膀
    RopoDevice::Chassis.AutoPositionMove(0.22,-2.26,0);//推球至门前
    RopoDevice::Chassis.MoveVelocity(2,0);//推球入网
    pros::delay(100);
    RopoDevice::Chassis.MoveVelocity(-0.8,0);
    pros::delay(200);
    RopoDevice::Chassis.AutoPositionMove(-0.24,-1.85);
    RopoDevice::Chassis.AutoRotateAbs(90);
    pros::delay(800);

    RopoDevice::Chassis.MoveVelocity(1.5,0);
    pros::delay(250);//300
    ControllerModule::switch_climber();//碰杆

}

//中断测试任务
	void Test_Task()
	{

		Auto();
		
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

void skill(){
}



/// @brief 手柄操作定义






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
