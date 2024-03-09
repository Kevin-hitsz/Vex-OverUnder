// Code : UTF - 8
#ifndef ROPO_DEVICE_HPP
#define ROPO_DEVICE_HPP

#include "pros/imu.hpp"
#include "pros/adi.hpp"
#include "pros/motors.hpp"
#include "RopoMath/Vector.hpp"
#include "RopoSensor/EncodingDisk.hpp"
#include "RopoParameter.hpp"
#include "RopoSensor/OpenMv.hpp"
#include "RopoControl/Regulator.hpp"
#include "RopoGpsAddPosition.hpp"
#include "RopoApi.hpp"
#include "RopoChassis.hpp"
#include "RopoPosition.hpp"
#include "RopoLifter.hpp"

namespace RopoDevice{

	//创建三线接口
	namespace ThreeWire{

		const char ExternPneumaticPort = 'G';
		pros::ADIDigitalOut ExternPneumatic(ExternPneumaticPort,false);
		
		const char WingPneumaticPort  = 'B';
		pros::ADIDigitalOut WingPneumatic(WingPneumaticPort,false);

		const char IntakerPneumaticPort = 'A';
		pros::ADIDigitalOut IntakerPneumatic(IntakerPneumaticPort,false);

		const char SpadePneumaticPort = 'H';
		pros::ADIDigitalOut SpadePneumatic(SpadePneumaticPort,false);

	}

//创建惯性传感器
	namespace Sensors{
		const int InertialPort = 19;		// 19
		pros::IMU Inertial(InertialPort);
		const int OpenmvPort = 16;
		RopoSensor::OpenMv My_openMV(OpenmvPort,115200);
	}			
	
	// 创建电机
	namespace Motors{	

		const int LeftChassisMotor1Port  	= 3;	// 3
		const int LeftChassisMotor2Port  	= 1;	// 1
		const int LeftChassisMotor3Port  	= 9;	// 9
        const int LeftChassisMotor4Port  	= 6;	
		const int RightChassisMotor1Port	= 13;
		const int RightChassisMotor2Port	= 12;
		const int RightChassisMotor3Port	= 15;	// 15
		const int RightChassisMotor4Port	= 17;	// 17
		
		const pros::motor_gearset_e_t ChassisGearset = pros::E_MOTOR_GEAR_BLUE;

		pros::Motor      LeftChassisMotor1 (LeftChassisMotor1Port  , 	ChassisGearset, true);
		pros::Motor      LeftChassisMotor2 (LeftChassisMotor2Port  , 	ChassisGearset, true);
		pros::Motor      LeftChassisMotor3 (LeftChassisMotor3Port  , 	ChassisGearset, true);
        pros::Motor      LeftChassisMotor4 (LeftChassisMotor4Port  , 	ChassisGearset, true);
        

		pros::Motor      RightChassisMotor1(RightChassisMotor1Port ,	ChassisGearset, false);
		pros::Motor      RightChassisMotor2(RightChassisMotor2Port ,	ChassisGearset, false);
		pros::Motor      RightChassisMotor3(RightChassisMotor3Port ,	ChassisGearset, false);
        pros::Motor      RightChassisMotor4(RightChassisMotor4Port ,	ChassisGearset, false);
        

		const FloatType ChassisRatio = 23.0 / 22.0;
		bool ChassisControllerMode = false;
		void LeftWheelMove	(FloatType Velocity){
			LeftChassisMotor1.move_velocity(Velocity );
			LeftChassisMotor2.move_velocity(Velocity );
			LeftChassisMotor3.move_velocity(Velocity );
			LeftChassisMotor4.move_velocity(Velocity );
		}

		void RightWheelMove (FloatType Velocity){
			RightChassisMotor1.move_velocity(Velocity );
			RightChassisMotor2.move_velocity(Velocity );
			RightChassisMotor3.move_velocity(Velocity );
			RightChassisMotor4.move_velocity(Velocity );
			
		}

		FloatType LV,RV,Kv;//Kv为速度大于600时的缩小比例
		void MoveOpControll(FloatType CM, FloatType DM){
			CM = CM * RopoParameter::CHASSIS_RATIO * RopoParameter::RAD_TO_RPM / RopoParameter::WHEEL_RAD;
			DM = DM * RopoParameter::CHASSIS_RATIO * RopoParameter::RAD_TO_RPM / RopoParameter::WHEEL_RAD;
			LV = CM - DM * RopoParameter::CHASSIS_PARAMETER / 2.0;
			RV = CM + DM * RopoParameter::CHASSIS_PARAMETER / 2.0;
			if(fabs(LV) > RopoParameter::CHASSIS_SPPED_MAX || fabs(RV) > RopoParameter::CHASSIS_SPPED_MAX) {
				Kv = RopoParameter::CHASSIS_SPPED_MAX / fmax(fabs(LV),fabs(RV));
				LV *= Kv;
				RV *= Kv;
			}
			LeftWheelMove(LV);
			RightWheelMove(RV);
		}

		const int RightLiftMotorPort		= 20;
		const pros::motor_gearset_e_t LiftGearset = pros::E_MOTOR_GEAR_RED;
		pros::Motor   RightLiftMotor ( RightLiftMotorPort  , 	LiftGearset, false );

		const int IntakeMotorPort		= 10;
		const pros::motor_gearset_e_t IntakeGearset = pros::E_MOTOR_GEAR_BLUE;
		pros::Motor   IntakeMotor ( IntakeMotorPort  , 	IntakeGearset, true );
		
	}

	namespace Gpss{
		static pros::Gps vexGps(RopoParameter::GPS_PORT           , RopoParameter::GPSX_INITIAL, RopoParameter::GPSY_INITIAL,
						     RopoParameter::GPS_HEADING_INITIAL, RopoParameter::GPSX_OFFSET , RopoParameter::GPSY_OFFSET);
	}

	// 创建定位模块
	namespace Position_Motor{
		RopoPosition::Position MyPosition(  Motors::LeftChassisMotor1 , Motors::LeftChassisMotor2 , Motors::LeftChassisMotor3 ,Motors::LeftChassisMotor4 ,
            Motors:: RightChassisMotor1,Motors:: RightChassisMotor2 ,Motors:: RightChassisMotor3 ,Motors:: RightChassisMotor4 ,Sensors::Inertial);
	}


// 创建运球模块
	RopoLifter::LifterModule LiftMotors(Motors::RightLiftMotor);
	FloatType GetHeading(){
		return -RopoDevice::Sensors::Inertial.get_rotation()*1.017; 
	}

// 坐标获取函数
	RopoMath::Vector<FloatType> GetPosition(){
		RopoMath::Vector<FloatType> PositionVector(RopoMath::ColumnVector,3);
		PositionVector[1] =  RopoDevice::Position_Motor::MyPosition.Get_X();
		PositionVector[2] =  RopoDevice::Position_Motor::MyPosition.Get_Y();
		PositionVector[3] =  GetHeading();
		PositionVector[3] = (int((PositionVector[3] + 180.0 +14400) * 100.0) % 36000) / 100.0 - 180.0;
		return PositionVector;
	}

	RopoGpsAddPosition::GpsAddPositionModule gpsAddPosition(GetPosition,Gpss::vexGps,20);

	Vector GetTransformedPosition(){
		return gpsAddPosition.GetTransformedPosition();
	}
    
	//	创建底盘
	RopoChassis::TankChassis Chassis( Motors::RightWheelMove , Motors::LeftWheelMove , GetTransformedPosition , 1 );




//初始化
	void DeviceInit(){
		RopoDevice::Chassis.SetVelocityLimits(600);
        Sensors::Inertial.reset(true);
        while(Sensors::Inertial.is_calibrating())pros::delay(20);
		pros::delay(200);
		Position_Motor::MyPosition.initial();
	}

	void MotorsInit(){
		Motors::LeftChassisMotor1 .set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
		Motors::LeftChassisMotor2 .set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
		Motors::LeftChassisMotor3 .set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
		Motors::LeftChassisMotor4 .set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
		Motors::RightChassisMotor1.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
		Motors::RightChassisMotor2.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
		Motors::RightChassisMotor3.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
		Motors::RightChassisMotor4.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);

		Motors::RightLiftMotor.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
		Motors::IntakeMotor.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
	}

	void ChassisHold(){
		Motors::LeftChassisMotor1 .set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
		Motors::LeftChassisMotor2 .set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
		Motors::LeftChassisMotor3 .set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
		Motors::LeftChassisMotor4 .set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
		Motors::RightChassisMotor1.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
		Motors::RightChassisMotor2.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
		Motors::RightChassisMotor3.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
	}

}

#endif
