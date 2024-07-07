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
#include "RopoLifter.hpp"
#include "RopoInertial.hpp"

namespace RopoDevice{

	// //创建三线接口
	// namespace ThreeWire{
	// 	//爬杆钩子
	// 	const char ExternPneumaticPort = 'U';
	// 	pros::ADIDigitalOut ExternPneumatic(ExternPneumaticPort,false);
	// 	//翅膀
	// 	const char WingPneumaticPort  = 'U';
	// 	pros::ADIDigitalOut WingPneumatic(WingPneumaticPort,false);
	// 	//intaker
	// 	const char IntakerPneumaticPort = 'U';
	// 	pros::ADIDigitalOut IntakerPneumatic(IntakerPneumaticPort,false);
	// 	//铲子
	// 	const char SpadePneumaticPort = 'U';
	// 	pros::ADIDigitalOut SpadePneumatic(SpadePneumaticPort,false);
	// }

	//创建惯性传感器
	namespace Sensors{
		const int InertialPort = 0;
		RopoInertial imu(InertialPort);
	}			
	
	// 创建电机
	namespace Motors{	

		const int LeftChassisMotor1Port  	= 0;
		const int LeftChassisMotor2Port  	= 0;
		const int LeftChassisMotor3Port  	= 0;
        const int LeftChassisMotor4Port  	= 0;
		const int RightChassisMotor1Port	= 0;
		const int RightChassisMotor2Port	= 0;
		const int RightChassisMotor3Port	= 0;
		const int RightChassisMotor4Port	= 0;
		
		const pros::motor_gearset_e_t ChassisGearset = pros::E_MOTOR_GEAR_BLUE;

		pros::Motor      LeftChassisMotor1 (LeftChassisMotor1Port  , 	ChassisGearset, true);
		pros::Motor      LeftChassisMotor2 (LeftChassisMotor2Port  , 	ChassisGearset, true);
		pros::Motor      LeftChassisMotor3 (LeftChassisMotor3Port  , 	ChassisGearset, true);
        pros::Motor      LeftChassisMotor4 (LeftChassisMotor4Port  , 	ChassisGearset, true);
        pros::Motor      LeftChassisMotor5 (LeftChassisMotor4Port  , 	ChassisGearset, true);

		pros::Motor      RightChassisMotor1(RightChassisMotor1Port ,	ChassisGearset, false);
		pros::Motor      RightChassisMotor2(RightChassisMotor2Port ,	ChassisGearset, false);
		pros::Motor      RightChassisMotor3(RightChassisMotor3Port ,	ChassisGearset, false);
        pros::Motor      RightChassisMotor4(RightChassisMotor4Port ,	ChassisGearset, false);
        pros::Motor      RightChassisMotor5(RightChassisMotor4Port ,	ChassisGearset, false);

		RopoMotorGroup::MotorGroup LeftMotorGroup({&LeftChassisMotor1,&LeftChassisMotor2,&LeftChassisMotor3,&LeftChassisMotor4,
		&LeftChassisMotor5});
		RopoMotorGroup::MotorGroup RightMotorGroup({&RightChassisMotor1,&RightChassisMotor2,&RightChassisMotor3,&RightChassisMotor4,
		&RightChassisMotor5});


		const FloatType ChassisRatio = 23.0 / 22.0;
		bool ChassisControllerMode = false;
		

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
			LeftMotorGroup.move_voltage(LV*20);
			RightMotorGroup.move_voltage(RV*20);
		}

		const int IntakeMotorPort		= 0;
		const pros::motor_gearset_e_t IntakeGearset = pros::E_MOTOR_GEAR_BLUE;
		pros::Motor   IntakeMotor ( IntakeMotorPort  , 	IntakeGearset, true );
		
	}
	//gps定位
	namespace Gpss{
		static pros::Gps vexGps(RopoParameter::GPS_PORT           , RopoParameter::GPSX_INITIAL, RopoParameter::GPSY_INITIAL,
						     RopoParameter::GPS_HEADING_INITIAL, RopoParameter::GPSX_OFFSET , RopoParameter::GPSY_OFFSET);
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
	RopoGpsAddPosition::GpsAddPositionModule gpsAddPosition(GetPosition,Gpss::vexGps,20);

	Vector GetTransformedPosition(){
		return gpsAddPosition.GetTransformedPosition();
	}
    
	//	创建底盘
	RopoChassis::TankChassis Chassis( Motors::RightMotorGroup , Motors::LeftMotorGroup , GetTransformedPosition , 1 );

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
		Motors::IntakeMotor.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
	}
	
	
}

#endif
