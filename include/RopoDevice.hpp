// Code : UTF - 8
#ifndef ROPO_DEVICE_HPP
#define ROPO_DEVICE_HPP

#include "RopoDiffySwerve.hpp"
#include "pros/imu.hpp"
#include "pros/adi.hpp"
#include "pros/motors.h"
#include "pros/motors.hpp"
#include "RopoMath/Vector.hpp"
#include "RopoSensor/EncodingDisk.hpp"
#include "RopoSensor/OpenMv.hpp"
#include "RopoControl/Regulator.hpp"
#include "RopoApi.hpp"
#include "RopoChassis.hpp"
#include "RopoThrower.hpp"
#include "RopoLifter.hpp"

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
	
	// Code 
	namespace Motors{
		const int LeftFrontMotorPort = 3;
		const int LeftFrontMotorPort_= 4;
		pros::Motor LeftFrontMotor(LeftFrontMotorPort,pros::E_MOTOR_GEAR_BLUE,false);
		pros::Motor LeftFrontMotor_(LeftFrontMotorPort_,pros::E_MOTOR_GEAR_BLUE,false);
		RopoDiffySwerve::DiffySwerve LeftFrontSwerve(LeftFrontMotor, LeftFrontMotor_);
	}
}
#endif
