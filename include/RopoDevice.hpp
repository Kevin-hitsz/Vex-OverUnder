// Code : UTF - 8
#ifndef ROPO_DEVICE_HPP
#define ROPO_DEVICE_HPP

#include "pros/distance.hpp"
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
#include "RoPoPros/RopoInertial.hpp"

namespace RopoDevice{

	//创建三线接口
	namespace ThreeWire{
		//爬杆
		
		pros::ADIDigitalOut climb_Pneumatic(RopoParameter:: climb_pneumatic_port,false);
		//翅膀
		
		pros::ADIDigitalOut left_wing_pneumatic(RopoParameter::left_wing_pneumatic_port,false);
		
		
		pros::ADIDigitalOut right_wing_pneumatic(RopoParameter::right_wing_pneumatic_port,false);


		pros::ADIDigitalOut lead_ball_pneumatic(RopoParameter::lead_ball_pneumatic_port,false);
	}

	//创建惯性传感器
	namespace Sensors{
		
		RopoInertial imu(RopoParameter::InertialPort,RopoParameter::IMU_K);
	}			
	
	// 创建电机
	namespace Motors{	
		pros::Motor      LeftChassisMotor1 (RopoParameter::LeftChassisMotor1Port  , 	RopoParameter::ChassisGearset, true);
		pros::Motor      LeftChassisMotor2 (RopoParameter::LeftChassisMotor2Port  , 	RopoParameter::ChassisGearset, true);
		pros::Motor      LeftChassisMotor3 (RopoParameter::LeftChassisMotor3Port  , 	RopoParameter::ChassisGearset, true);
        pros::Motor      LeftChassisMotor4 (RopoParameter::LeftChassisMotor4Port  , 	RopoParameter::ChassisGearset, true);
        pros::Motor      LeftChassisMotor5 (RopoParameter::LeftChassisMotor5Port  , 	RopoParameter::ChassisGearset, true);

		pros::Motor      RightChassisMotor1(RopoParameter::RightChassisMotor1Port ,	RopoParameter::ChassisGearset, false);
		pros::Motor      RightChassisMotor2(RopoParameter::RightChassisMotor2Port ,	RopoParameter::ChassisGearset, false);
		pros::Motor      RightChassisMotor3(RopoParameter::RightChassisMotor3Port ,	RopoParameter::ChassisGearset, false);
        pros::Motor      RightChassisMotor4(RopoParameter::RightChassisMotor4Port ,RopoParameter::	ChassisGearset, false);
        pros::Motor      RightChassisMotor5(RopoParameter::RightChassisMotor5Port ,	RopoParameter::ChassisGearset, false);

		RopoMotorGroup::MotorGroup LeftMotorGroup({&LeftChassisMotor1,&LeftChassisMotor2,&LeftChassisMotor3,&LeftChassisMotor4,
		&LeftChassisMotor5});
		RopoMotorGroup::MotorGroup RightMotorGroup({&RightChassisMotor1,&RightChassisMotor2,&RightChassisMotor3,&RightChassisMotor4,
		&RightChassisMotor5});


		FloatType LV,RV,Kv;//Kv为速度大于600时的缩小比例
		int32_t MoveOpControll(FloatType CM, FloatType DM){
			CM = CM * RopoParameter::CHASSIS_RATIO * RopoParameter::RAD_TO_RPM / RopoParameter::WHEEL_RAD;
			DM = DM * RopoParameter::CHASSIS_RATIO * RopoParameter::RAD_TO_RPM / RopoParameter::WHEEL_RAD;
			LV = CM - DM * RopoParameter::CHASSIS_PARAMETER / 2.0;
			RV = CM + DM * RopoParameter::CHASSIS_PARAMETER / 2.0;
			if(fabs(LV) > RopoParameter::CHASSIS_SPPED_MAX || fabs(RV) > RopoParameter::CHASSIS_SPPED_MAX) {
				Kv = RopoParameter::CHASSIS_SPPED_MAX / fmax(fabs(LV),fabs(RV));
				LV *= Kv;
				RV *= Kv;
			}
			int32_t retl=1;
			int32_t retr=1;
			retl=LeftMotorGroup.move_voltage(LV*20);
			retr=RightMotorGroup.move_voltage(RV*20);
			if(retl==PROS_ERR||retr==PROS_ERR)
			return PROS_ERR;
			else return 1;
		}

	}


	// 创建定位模块
	namespace Position_Motor{
		RopoPosition::Position MyPosition(Motors::LeftMotorGroup,Motors::RightMotorGroup, Sensors::imu);
	}

	RopoMath::Vector<FloatType> GetPosition()
    {
	    RopoMath::Vector<FloatType> PositionVector(RopoMath::ColumnVector,3);
	    PositionVector[1] =  RopoDevice::Position_Motor::MyPosition.Get_X();
	    PositionVector[2] =  RopoDevice::Position_Motor::MyPosition.Get_Y();
	    PositionVector[3] =  RopoDevice::Position_Motor::MyPosition.Get_Angle();
	    return PositionVector;
	}


	//gps定位
	namespace Gpss{
		static pros::Gps vexGps(RopoParameter::GPS_PORT           , RopoParameter::GPSX_INITIAL, RopoParameter::GPSY_INITIAL,
						     RopoParameter::GPS_HEADING_INITIAL, RopoParameter::GPSX_OFFSET , RopoParameter::GPSY_OFFSET);
	}
	
	RopoGpsAddPosition::GpsAddPositionModule gpsAddPosition(GetPosition,Gpss::vexGps,20);

	Vector GetTransformedPosition(){
		return gpsAddPosition.GetTransformedPosition();
	}
    
	//创建底盘
	RopoChassis::TankChassis Chassis( Motors::RightMotorGroup , Motors::LeftMotorGroup , GetTransformedPosition , 1 );

	//Intaker	
	pros::Motor   intaker ( RopoParameter::IntakeMotorPort  , RopoParameter::	IntakeGearset, true );
		
	//初始化
	void DeviceInit(){
		RopoDevice::Chassis.SetVelocityLimits(600);
        Sensors::imu.reset(true);
		while(Sensors::imu.is_calibrating())pros::delay(200);
		pros::delay(200);
		Position_Motor::MyPosition.initial();
		Gpss::vexGps.set_data_rate(5);
	}

	void MotorsInit(){
		Chassis.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
		intaker.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
	}

}
	
#endif
