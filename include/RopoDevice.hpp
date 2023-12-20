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
#include "RopoPosition.hpp"
#include "RopoThrower.hpp"
#include "RopoLifter.hpp"

namespace RopoDevice{
	namespace ThreeWire{
		const char ExternPneumaticPort = 'A';
		pros::ADIDigitalOut ExternPneumatic(ExternPneumaticPort,false);
		
		const char CatchPneumaticPort  = 'H';
		pros::ADIDigitalOut CatchPneumatic(CatchPneumaticPort,false);

	}


	namespace Sensors{
		const int InertialPort = 17;
		pros::IMU Inertial(InertialPort);

	}			
	
	// Code 
	namespace Motors{

		const int LeftMotor1Port  	= 12;
		const int LeftMotor2Port  	= 14;
		const int LeftMotor3Port  	= 18;
		const int LeftMotor4Port    = 19;
		const int RightMotor1Port	= 3;
		const int RightMotor2Port	= 4;
		const int RightMotor3Port	= 5;
		const int RightMotor4Port   = 10;
		

		const pros::motor_gearset_e_t ChassisGearset = pros::E_MOTOR_GEAR_BLUE;

		pros::Motor      LeftMotor1 ( LeftMotor1Port  , 	ChassisGearset, false );
		pros::Motor      LeftMotor2 ( LeftMotor2Port  , 	ChassisGearset, false );
		pros::Motor      LeftMotor3 ( LeftMotor3Port  , 	ChassisGearset, false );
		pros::Motor      LeftMotor4 ( LeftMotor4Port  , 	ChassisGearset, false );
		pros::Motor      RightMotor1( RightMotor1Port ,		ChassisGearset, false);
		pros::Motor      RightMotor2( RightMotor2Port ,		ChassisGearset, false);
		pros::Motor      RightMotor3( RightMotor3Port ,		ChassisGearset, false);
		pros::Motor      RightMotor4( RightMotor3Port ,		ChassisGearset, false);
		pros::MotorGroup LeftMotor { Motors::LeftMotor1 , Motors::LeftMotor2 , Motors::LeftMotor3 , Motors::LeftMotor4   };
		pros::MotorGroup RightMotor{ Motors::RightMotor1, Motors::RightMotor2, Motors::RightMotor3, Motors::RightMotor4  };

		const FloatType ChassisRatio = 56.0 / 44.0;		// 减速比
		bool ChassisControllerMode = false;
		void LeftWheelMove	(FloatType Velocity){
			LeftMotor1.move_velocity(-Velocity );
			LeftMotor2.move_velocity(-Velocity );
			LeftMotor3.move_velocity(-Velocity );
			LeftMotor4.move_velocity(-Velocity );
			// constexpr FloatType RatioParam = 20;
			// LeftMotor1.move_voltage(-Velocity * RatioParam);
			// LeftMotor2.move_voltage(-Velocity * RatioParam);
			// LeftMotor3.move_voltage(-Velocity * RatioParam);
		}

		void RightWheelMove (FloatType Velocity){
			RightMotor1.move_velocity(Velocity );
			RightMotor2.move_velocity(Velocity );
			RightMotor3.move_velocity(Velocity );
			RightMotor4.move_velocity(Velocity );
			// constexpr FloatType RatioParam = 20;
			// RightMotor1.move_voltage(Velocity * RatioParam);
			// RightMotor2.move_voltage(Velocity * RatioParam);
			// RightMotor3.move_voltage(Velocity * RatioParam);
		}

		const int LeftLiftMotorPort		= 15;
		const int RightLiftMotorPort    = 5;
		const pros::motor_gearset_e_t LiftGearset = pros::E_MOTOR_GEAR_RED;
		pros::Motor   LeftLiftMotor ( LeftLiftMotorPort  , 	LiftGearset, false );
		pros::Motor   RightLiftMotor ( RightLiftMotorPort  , 	LiftGearset, true );
		pros::MotorGroup LiftMotor { Motors::LeftLiftMotor ,Motors::RightLiftMotor};
	}

	namespace Position_Motor{
		RopoPosition::Position MyPosition(&Motors::LeftMotor,&Motors::RightMotor,&Sensors::Inertial);
	}

	RopoLifter::LifterModule LiftMotors(&Motors::LiftMotor);

	FloatType GetHeading(){
		return -RopoDevice::Sensors::Inertial.get_yaw();
	}

	RopoMath::Vector<FloatType> GetPosition(){
		RopoMath::Vector<FloatType> PositionVector(RopoMath::ColumnVector,3);
		PositionVector[1] =  RopoDevice::Position_Motor::MyPosition.Get_X();
		PositionVector[2] =  RopoDevice::Position_Motor::MyPosition.Get_Y();
		PositionVector[3] =  GetHeading();

		return PositionVector;
	}

	RopoChassis::TankChassis Chassis( &Motors::RightWheelMove , &Motors::LeftWheelMove , &GetPosition , 1 );

	void DeviceInit(){
		RopoDevice::Chassis.SetVelocityLimits(600);
		Sensors::Inertial.reset(true);
		while(Sensors::Inertial.is_calibrating())pros::delay(20);
		pros::delay(200);
		Motors::LeftMotor1 .set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
		Motors::LeftMotor2 .set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
		Motors::LeftMotor3 .set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
		Motors::LeftMotor4 .set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
		Motors::RightMotor1.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
		Motors::RightMotor2.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
		Motors::RightMotor3.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
		Motors::RightMotor4.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
		Motors::LeftLiftMotor.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
		Motors::RightLiftMotor.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
		Position_Motor::MyPosition.initial();
	}

	void MotorsInit(){
		Motors::LeftMotor1 .set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
		Motors::LeftMotor2 .set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
		Motors::LeftMotor3 .set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
		Motors::LeftMotor4 .set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
		Motors::RightMotor1.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
		Motors::RightMotor2.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
		Motors::RightMotor3.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
		Motors::RightMotor3.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
		Motors::RightMotor4.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
		Motors::LeftLiftMotor.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
		Motors::RightLiftMotor.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
	}

}

#endif
