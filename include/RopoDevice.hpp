// Code : UTF - 8
#ifndef ROPO_DEVICE_HPP
#define ROPO_DEVICE_HPP

#include "pros/imu.hpp"
#include "pros/adi.hpp"
#include "pros/motors.hpp"
#include "RopoMath/Vector.hpp"
#include "RopoSensor/EncodingDisk.hpp"
#include "RopoSensor/OpenMv.hpp"
#include "RopoControl/Regulator.hpp"
#include "RopoApi.hpp"
#include "RopoChassis.hpp"


namespace RopoDevice{

	// Api
	typedef RopoApi::FloatType FloatType;

	namespace ThreeWire{
		const char LiftPneumaticPort  = 'C';
		pros::ADIDigitalOut LiftPneumatic(LiftPneumaticPort,false);

		const char ExternPneumaticPort  = 'A';
		pros::ADIDigitalOut ExternPneumatic(ExternPneumaticPort,false);
	}


	namespace Sensors{

		const int InertialPort = 2;
		pros::IMU Inertial(InertialPort);

	}			
	
	// Code 
	namespace Motors{

		const int LeftMotor1Port  	= 11;
		const int LeftMotor2Port  	= 12;
		const int LeftMotor3Port  	= 13;
		const int RightMotor1Port	= 17;
		const int RightMotor2Port	= 18;
		const int RightMotor3Port	= 19;
		

		const pros::motor_gearset_e_t ChassisGearset = pros::E_MOTOR_GEAR_BLUE;

		pros::Motor      LeftMotor1 ( LeftMotor1Port  , 	ChassisGearset, true );
		pros::Motor      LeftMotor2 ( LeftMotor2Port  , 	ChassisGearset, true );
		pros::Motor      LeftMotor3 ( LeftMotor3Port  , 	ChassisGearset, true );
		pros::Motor      RightMotor1( RightMotor1Port ,	ChassisGearset, false);
		pros::Motor      RightMotor2( RightMotor2Port ,	ChassisGearset, false);
		pros::Motor      RightMotor3( RightMotor3Port ,	ChassisGearset, false);
		pros::MotorGroup LeftMotor { Motors::LeftMotor1 ,Motors::LeftMotor2 ,Motors::LeftMotor3   };
		pros::MotorGroup RightMotor{ Motors::RightMotor1,Motors::RightMotor2,Motors::RightMotor3  };

		const FloatType ChassisRatio = 7.0 / 5.0;

		void LeftWheelMove	(FloatType Velocity){
			// LeftMotor1.move_velocity(Velocity );
			// LeftMotor2.move_velocity(Velocity );
			// LeftMotor3.move_velocity(Velocity );
			constexpr FloatType RatioParam = 20;
			LeftMotor1.move_voltage(Velocity * RatioParam);
			LeftMotor2.move_voltage(Velocity * RatioParam);
			LeftMotor3.move_voltage(Velocity * RatioParam);
		}

		void RightWheelMove (FloatType Velocity){
			// RightMotor1.move_velocity(Velocity );
			// RightMotor2.move_velocity(Velocity );
			// RightMotor3.move_velocity(Velocity );
			constexpr FloatType RatioParam = 20;
			RightMotor1.move_voltage(Velocity * RatioParam);
			RightMotor2.move_voltage(Velocity * RatioParam);
			RightMotor3.move_voltage(Velocity * RatioParam);
		}


	}



		

	FloatType GetHeading(){
		return -RopoDevice::Sensors::Inertial.get_yaw();
	}

	RopoMath::Vector<FloatType> GetPosition(){
		RopoMath::Vector<FloatType> PositionVector(RopoMath::ColumnVector,3);
		PositionVector[1] =  0 / 1000.0;
		PositionVector[2] =  0 / 1000.0;
		PositionVector[3] =  GetHeading();

		return PositionVector;
	}

	RopoChassis::TankChassis Chassis( Motors::RightWheelMove , Motors::LeftWheelMove , GetPosition , 1 );

	void DeviceInit(){
		RopoDevice::Chassis.SetVelocityLimits(600);
		Sensors::Inertial.reset(true);
		while(Sensors::Inertial.is_calibrating())pros::delay(20);
		pros::delay(200);
		Motors::LeftMotor1 .set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
		Motors::LeftMotor2 .set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
		Motors::LeftMotor3 .set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
		Motors::RightMotor1.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
		Motors::RightMotor2.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
		Motors::RightMotor3.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
	}

}

#endif
