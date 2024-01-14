// Code : UTF - 8
#pragma once
#include "RopoDiffySwerve.hpp"
#include "pros/imu.hpp"
#include "pros/adi.hpp"
#include "pros/motors.h"
#include "pros/motors.hpp"
#include "RopoMath/Vector.hpp"
#include "RopoSensor/EncodingDisk.hpp"
#include "RopoControl/Regulator.hpp"
#include "RopoApi.hpp"
#include "RopoChassis.hpp"
#include <vector>

namespace RopoDevice{
	namespace ThreeWire{
		// const char ExternPneumaticPort = 'B';
		// pros::ADIDigitalOut ExternPneumatic(ExternPneumaticPort,false);
		
		// const char CatchPneumaticPort  = 'A';
		// pros::ADIDigitalOut CatchPneumatic(CatchPneumaticPort,false);

	}


	namespace Sensors{
		// const int InertialPort = 7;
		// pros::IMU Inertial(InertialPort);

	}			
	
	namespace Motors{
		const int LFMotorPort  = 3;
		const int LFMotorPort_ = 4;
		const int LBMotorPort  = 5;
		const int LBMotorPort_ = 6;
		const int RFMotorPort  = 7;
		const int RFMotorPort_ = 8;
		const int RBMotorPort  = 9;
		const int RBMotorPort_ =10;
		Motor LFMotor (LFMotorPort,pros::E_MOTOR_GEAR_BLUE, false);
		Motor LFMotor_(LFMotorPort_,pros::E_MOTOR_GEAR_BLUE,false);
		Motor LBMotor (LBMotorPort,pros::E_MOTOR_GEAR_BLUE, true );
		Motor LBMotor_(LBMotorPort_,pros::E_MOTOR_GEAR_BLUE,true );		
		Motor RFMotor (RFMotorPort,pros::E_MOTOR_GEAR_BLUE, false);
		Motor RFMotor_(RFMotorPort_,pros::E_MOTOR_GEAR_BLUE,false);
		Motor RBMotor (RBMotorPort,pros::E_MOTOR_GEAR_BLUE, true );
		Motor RBMotor_(RBMotorPort_,pros::E_MOTOR_GEAR_BLUE,true );		
	}

	Swerve LF(Motors::LFMotor,Motors::LFMotor_);
	Swerve LB(Motors::LBMotor,Motors::LBMotor_);
	Swerve RF(Motors::RFMotor,Motors::RFMotor_);
	Swerve RB(Motors::RBMotor,Motors::RBMotor_);
	Chassis Chassis(LF,LB,RF,RB);
}