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

void GpsUpdate(){
		RopoDevice::Chassis.MoveVelocity(0,0);
        pros::delay(400);
        RopoDevice::gpsAddPosition.SetUpdateFlag(1);
        pros::delay(100);
        RopoDevice::gpsAddPosition.SetUpdateFlag(0);
	}
    void lead_ball()
    {
        for(int i=1;i<=4;i++)
        {
            RopoDevice::Chassis.MoveVelocity(0.3,0);
            pros::delay(330);
            RopoDevice::Chassis.AutoRotateAbs_block(-90);//131.9
            
            if(i==4){
                break;
            }
            RopoDevice::Chassis.AutoRotateAbs_block(129.5);
            
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

	
	void ControllerPrint()
    {
		pros::Controller MasterController(pros::E_CONTROLLER_MASTER);
		while(true) 
        {
			MasterController.print(0,0,"%.2f,%.2f,%.1f ",RopoDevice::Position_Motor::MyPosition.Get_X()
			,RopoDevice::Position_Motor::MyPosition.Get_Y(),RopoDevice::Position_Motor::MyPosition.Get_Angle());
            pros::delay(100);
            MasterController.print(1,0,"%.2f,%.2f ",RopoDevice:: GetTransformedPosition()[1],
			RopoDevice:: GetTransformedPosition()[2]);
            pros::delay(100);
            MasterController.print(2,0,"%.2f,%.2f,%.0f,%.2f ",RopoDevice:: Gpss::vexGps.get_status().x,
			RopoDevice:: Gpss::vexGps.get_status().y,RopoDevice:: Gpss::vexGps.get_status().yaw,RopoDevice:: Gpss::vexGps.get_error());
			pros::delay(100); 	
		}
	}
	
}


/// @brief 定义自动程序

void Auto(){
    //初始化
    RopoDevice::Chassis.StartChassisAutoControll();
    pros::delay(10);
   

    RopoDevice::Chassis.AutoPositionMove(1.2,0,90);//前进到场地中段，并转90面向球
    ControllerModule::open_both_wing();//开翅膀
    pros::delay(100);
    RopoDevice::Chassis.MoveVelocity(0.8,0);//把球往前堆一堆
    pros::delay(600);
    RopoDevice::Chassis.MoveVelocity(0,0);//刹车
    pros::delay(50);

    RopoDevice::Chassis.AutoRotateAbs_block(180);//扫球


    RopoDevice::Chassis.MoveVelocity(0.7,0);//把球推到接近过道
    ControllerModule::outtake();//
    pros::delay(700);
    RopoDevice::Chassis.MoveVelocity(0,0);//刹车

    ControllerModule::close_both_wing();//关翅膀
    pros::delay(50);
    RopoDevice::Chassis.MoveVelocity(-0.8,0);//后撤步
    pros::delay(300);
    ControllerModule::intaker_stop();

    RopoDevice::Chassis.AutoRotateAbs_block(90);//旋转反向调整推球位置
    
    RopoDevice::Chassis.MoveVelocity(0.8,0);//调整推球位置
    pros::delay(200);
    RopoDevice::Chassis.AutoRotateAbs_block(0);//旋转屁股对准球
    RopoDevice::Chassis.MoveVelocity(-0.7,0);//推球过杆
    pros::delay(800);



    RopoDevice::Chassis.MoveVelocity(1,0);
    pros::delay(300);
    RopoDevice::Chassis.AutoRotateAbs_block(90);//往前一点旋转，gps看一眼
    ControllerModule::GpsUpdate();



    RopoDevice::Chassis.AutoPositionMove(0.30,-0.39,-135.6);//到达导球点
    ControllerModule::open_left_wing();//开翅膀准备导球

    ControllerModule::lead_ball();//导球
    ControllerModule::close_left_wing();//关翅膀，导球结束

    RopoDevice::Chassis.MoveVelocity(0.6,0);
    pros::delay(300);
    ControllerModule::GpsUpdate();
    
    RopoDevice::Chassis.AutoRotateAbs_block(-90);
    ControllerModule::GpsUpdate();
    RopoDevice::Chassis.AutoPositionMove(-0.06,0.06,90);//来到过道口

    
    RopoDevice::Chassis.AutoPositionMove(-0.07,1.92,90);//推球至过道另一端


    RopoDevice::Chassis.MoveVelocity(0.4,-0.8);//弧线推球入网
    pros::delay(2300);
    RopoDevice::Chassis.MoveVelocity(0,0);//
    pros::delay(100);

    RopoDevice::Chassis.MoveVelocity(-0.3,0);//后撤步调整角度
    pros::delay(600);
    RopoDevice::Chassis.AutoRotateAbs_block(20);//调整角度准备反复推球入网



     RopoDevice::Chassis.MoveVelocity(1.0,0);//第一次推球入网
    pros::delay(400);
    RopoDevice::Chassis.MoveVelocity(-1.0,0);//后撤步
    pros::delay(300);
    RopoDevice::Chassis.MoveVelocity(1.8,0);//第二次推球入网
    pros::delay(500);
    RopoDevice::Chassis.MoveVelocity(-0.6,0);//后撤步
    pros::delay(400);

    RopoDevice::Chassis.AutoRotateAbs(90);//调整角度看gps
    ControllerModule::GpsUpdate();

    RopoDevice::Chassis.AutoPositionMove(-0.03,-1.75,-90);//-0.24,-1.85
    RopoDevice::Chassis.MoveVelocity(1.8,0);
    pros::delay(550);

    RopoDevice::Chassis.MoveVelocity(0,0);


    
    

    // RopoDevice::Chassis.MoveVelocity(0.8,0);
    // pros::delay(200);//300
    


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
