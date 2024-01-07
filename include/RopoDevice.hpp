// Code : UTF - 8
#ifndef ROPO_DEVICE_HPP
#define ROPO_DEVICE_HPP

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
#include "RopoPosition.hpp"
#include "RopoThrower.hpp"
#include "RopoIntaker.hpp"


namespace RopoDevice{

	// Api
	typedef RopoApi::FloatType FloatType;

	namespace ThreeWire{
		const char LiftPneumaticPort  = 'C';
		pros::ADIDigitalOut LiftPneumatic(LiftPneumaticPort,false);

		const char ExternPneumaticPort  = 'B';
		pros::ADIDigitalOut ExternPneumatic(ExternPneumaticPort,false);

		const char PusherPneumaticPort  = 'A';
		pros::ADIDigitalOut PusherPneumatic(PusherPneumaticPort,false);
		
	}


	namespace Sensors{

		const int InertialPort = 19;
		pros::IMU Inertial(InertialPort);

		const int OpenMvPort = 4;
		RopoSensor::OpenMv myOpenMv(OpenMvPort,115200,10);

		const int receiveID = 1;
		const int sendID = 2;
		RopoSensor::EncodingDisk encodingDisk(receiveID,115200,sendID,115200,10);

	}			
	
	// Code 
	namespace Motors{

		const int LeftMotor1Port  	= 11;
		const int LeftMotor2Port  	= 17;
		const int LeftMotor3Port  	= 20;
		const int RightMotor1Port	= 3;
		const int RightMotor2Port	= 7;
		const int RightMotor3Port	= 10;
		

		const pros::motor_gearset_e_t ChassisGearset = pros::E_MOTOR_GEAR_BLUE;

		pros::Motor      LeftMotor1 ( LeftMotor1Port  , ChassisGearset, true );
		pros::Motor      LeftMotor2 ( LeftMotor2Port  , ChassisGearset, true );
		pros::Motor      LeftMotor3 ( LeftMotor3Port  , ChassisGearset, true );
		pros::Motor      RightMotor1( RightMotor1Port ,	ChassisGearset, false);
		pros::Motor      RightMotor2( RightMotor2Port ,	ChassisGearset, false);
		pros::Motor      RightMotor3( RightMotor3Port ,	ChassisGearset, false);
		pros::MotorGroup LeftMotor { Motors::LeftMotor1 ,Motors::LeftMotor2 ,Motors::LeftMotor3   };
		pros::MotorGroup RightMotor{ Motors::RightMotor1,Motors::RightMotor2,Motors::RightMotor3  };

		const FloatType ChassisRatio = 6.0 / 5.0;

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

		const int  LeftThrowerMotorPort  	= 18;
		const int RightThrowerMotorPort  	= 9;
		const pros::motor_gearset_e_t ThrowerGearset = pros::E_MOTOR_GEAR_RED;
		pros::Motor   LeftThrowerMotor ( LeftThrowerMotorPort  , 	ThrowerGearset, false );
		pros::Motor   RightThrowerMotor ( RightThrowerMotorPort  , 	ThrowerGearset, true );
		pros::MotorGroup ThrowerMotor { Motors::LeftThrowerMotor ,Motors::RightThrowerMotor};

		// const int RaiserMotorPort = 16;
		// const pros::motor_gearset_e_t RaiserGearset = pros::E_MOTOR_GEAR_RED;
		// pros::Motor RaiserMotor ( RaiserMotorPort, RaiserGearset, false );

		// const int LeftRollerMotorPort		= 5;
		// const int RightRollerMotorPort		= 3;
		// const pros::motor_gearset_e_t RollerGearset = pros::E_MOTOR_GEAR_GREEN;
		// pros::Motor   LeftRollerMotor ( LeftRollerMotorPort  , 	RollerGearset, false );
		// pros::Motor   RightRollerMotor ( RightRollerMotorPort  , 	RollerGearset, true );
		// pros::MotorGroup RollerMotor { Motors::LeftRollerMotor ,Motors::RightRollerMotor};
		

		const int LeftIntakerMotorPort      = 12;
		const int RightIntakerMotorPort     = 2 ;
		const pros::motor_gearset_e_t IntakerGearset = pros::E_MOTOR_GEAR_GREEN;
		pros::Motor   LeftIntakerMotor( LeftIntakerMotorPort  , IntakerGearset , true );
		pros::Motor   RightIntakerMotor(RightIntakerMotorPort , IntakerGearset , false );
		pros::MotorGroup IntakerMotor { Motors::LeftIntakerMotor,Motors::RightIntakerMotor};
	}

	namespace Position_Motor{
		RopoPosition::Position MyPosition(&Motors::LeftMotor,&Motors::RightMotor,&Sensors::Inertial);
	}		
	namespace Thrower_Motor{
		RopoThrower::ThrowerModule MyThrower(&Motors::ThrowerMotor);
	}
	namespace Intaker_Motor{
		RopoIntaker::IntakerModule MyIntaker(&Motors::IntakerMotor);
	}

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

	RopoChassis::TankChassis Chassis( Motors::RightWheelMove , Motors::LeftWheelMove , GetPosition , 1 );

	void DeviceInit(){
		RopoDevice::Chassis.SetVelocityLimits(600);
		Sensors::Inertial.reset(true);
		while(Sensors::Inertial.is_calibrating())pros::delay(20);
		pros::delay(700);
		Motors::LeftMotor1 .set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
		Motors::LeftMotor2 .set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
		Motors::LeftMotor3 .set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
		Motors::RightMotor3.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
		Motors::RightMotor1.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
		Motors::RightMotor2.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
		Position_Motor::MyPosition.initial();
	}

	void DeviceInitOp1(){
		Motors::LeftMotor1 .set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
		Motors::LeftMotor2 .set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
		Motors::LeftMotor3 .set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
		Motors::RightMotor3.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
		Motors::RightMotor1.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
		Motors::RightMotor2.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
		pros::Controller MasterController1(pros::E_CONTROLLER_MASTER);
		MasterController1.rumble("-.-.-");
	}

	void DeviceInitOp2(){
		Motors::LeftMotor1 .set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
		Motors::LeftMotor2 .set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
		Motors::LeftMotor3 .set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
		Motors::RightMotor3.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
		Motors::RightMotor1.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
		Motors::RightMotor2.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
		pros::Controller MasterController1(pros::E_CONTROLLER_MASTER);
		MasterController1.rumble("-.-.-");
	}



}

#endif
