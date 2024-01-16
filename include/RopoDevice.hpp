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
		const int LFMotorPort  	= 16;
		const int LFMotorPort_ 	= 17;
		const int LBMotorPort  	= 19;
		const int LBMotorPort_ 	= 20;
		const int RFMotorPort  	= 15;
		const int RFMotorPort_ 	= 14;
		const int RBMotorPort  	= 12;
		const int RBMotorPort_ 	= 11;
		Motor LFMotor (LFMotorPort,pros::E_MOTOR_GEAR_BLUE, false);
		Motor LFMotor_(LFMotorPort_,pros::E_MOTOR_GEAR_BLUE,false);
		Motor LBMotor (LBMotorPort,pros::E_MOTOR_GEAR_BLUE, false );
		Motor LBMotor_(LBMotorPort_,pros::E_MOTOR_GEAR_BLUE,false );		
		Motor RFMotor (RFMotorPort,pros::E_MOTOR_GEAR_BLUE, false);
		Motor RFMotor_(RFMotorPort_,pros::E_MOTOR_GEAR_BLUE,false);
		Motor RBMotor (RBMotorPort,pros::E_MOTOR_GEAR_BLUE, false );
		Motor RBMotor_(RBMotorPort_,pros::E_MOTOR_GEAR_BLUE,false );		
	}

	// RopoDiffySwerve::DiffySwerve LF(Motors::LFMotor,Motors::LFMotor_);
	// RopoDiffySwerve::DiffySwerve LB(Motors::LBMotor,Motors::LBMotor_);
	// RopoDiffySwerve::DiffySwerve RF(Motors::RFMotor,Motors::RFMotor_);
	// RopoDiffySwerve::DiffySwerve RB(Motors::RBMotor,Motors::RBMotor_);
	// Chassis Chassis(LF,LB,RF,RB);
}