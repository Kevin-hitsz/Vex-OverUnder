# pragma once

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
		pros::delay(500);
		RopoDevice::gpsAddPosition.SetUpdateFlag(1);
		pros::delay(200);
		RopoDevice::gpsAddPosition.SetUpdateFlag(0);
		pros::delay(50);
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