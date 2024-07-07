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
	

	void RumbleMe(){
		pros::Controller MasterController1(pros::E_CONTROLLER_MASTER);
		MasterController1.rumble("-.-.-");
		FloatType aa = pros::millis();
		while(pros::millis()-aa<45000){
			pros::delay(100);
		}
		
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

	void TurnAround(){
		RopoDevice::Chassis.AutoRotateRelative(180);
	}

	void GpsUpdate(){
		RopoDevice::gpsAddPosition.GpsUpdate();
	}

	
	void ControllerPrint()
    {
		while(true) 
        {
			pros::Controller MasterController(pros::E_CONTROLLER_MASTER);
			// MasterController.print(0,1,"X: %.2lf Y:%.2lf   ",(RopoDevice::GetTransformedPosition())[1],(RopoDevice::GetTransformedPosition())[2]);
			// pros::delay(50); 
			// MasterController.print(1,1,"degree: %.3lf    ",RopoDevice::GetPosition()[3]);
			// pros::delay(50); 
			// MasterController.print(2,1,"Extern:%s",externFlag?"yes":"no");
			// pros::delay(50);
		}
	}
}



/// @brief 定义自动程序
namespace AutoOperation{




void skill(){
}}


/// @brief 其他
void delay(){
	while(!RopoDevice::Chassis.IfArrived()){pros::delay(20);}
	return;
}

void delayDeg(){
	double aa = pros::millis();
	while(!RopoDevice::Chassis.IfDegArrived() && aa - pros::millis() < 2500)pros::delay(20);
}


#endif