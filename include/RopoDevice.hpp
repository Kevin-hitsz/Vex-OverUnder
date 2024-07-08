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

namespace RopoDevice{

	//创建三线接口
	namespace ThreeWire{

		const char ExternPneumaticPort = 'A';
		pros::ADIDigitalOut ExternPneumatic(ExternPneumaticPort,false);
		
		const char LeftWingPneumaticPort  = 'C';
		pros::ADIDigitalOut LeftWingPneumatic(LeftWingPneumaticPort,false);

		const char RightWingPneumaticPort  = 'B';
		pros::ADIDigitalOut RightWingPneumatic(RightWingPneumaticPort,false);

		const char IntakerPneumaticPort1 = 'F';
		pros::ADIDigitalOut IntakerPneumatic1(IntakerPneumaticPort1,false);
		
		const char IntakerPneumaticPort2 = 'G';
		pros::ADIDigitalOut IntakerPneumatic2(IntakerPneumaticPort2,false);
		
		const char BarPneumaticPort = 'H';
		pros::ADIDigitalOut BarPneumatic(BarPneumaticPort,false);
		
		const char HangPneumaticPort = 'D';
		pros::ADIDigitalOut HangPneumatic(HangPneumaticPort,false);


	}

//创建惯性传感器
	namespace Sensors{
		const int InertialPort = 8;
		pros::IMU Inertial(InertialPort);
		const int OpenmvPort = 19;
		RopoSensor::OpenMv My_openMV(OpenmvPort,115200);
		

	// 	const int EncodingDiskReceivePort = 15;
	// 	const int EncodingDiskSendPort = 16;
	// 	const int EncodingDiskBaudrate = 115200;
	// 	RopoSensor::EncodingDisk EncodingDisk(EncodingDiskSendPort,EncodingDiskBaudrate,EncodingDiskReceivePort,EncodingDiskBaudrate);
	}			
	
	// 创建电机
	namespace Motors{	

		const int LeftChassisMotor1Port  	= 9;
		const int LeftChassisMotor2Port  	= 3;
		const int LeftChassisMotor3Port  	= 5;
        const int LeftChassisMotor4Port  	= 2;
		const int LeftChassisMotor5Port 	= 1;
		const int RightChassisMotor1Port	= 15;
		const int RightChassisMotor2Port	= 16;
		const int RightChassisMotor3Port	= 14;
		const int RightChassisMotor4Port	= 13;
		const int RightChassisMotor5Port	= 12;
		
		const pros::motor_gearset_e_t ChassisGearset = pros::E_MOTOR_GEAR_BLUE;

		pros::Motor      LeftChassisMotor1 (LeftChassisMotor1Port  , 	ChassisGearset, false);
		pros::Motor      LeftChassisMotor2 (LeftChassisMotor2Port  , 	ChassisGearset, true);
		pros::Motor      LeftChassisMotor3 (LeftChassisMotor3Port  , 	ChassisGearset, false);
        pros::Motor      LeftChassisMotor4 (LeftChassisMotor4Port  , 	ChassisGearset, true);
        pros::Motor      LeftChassisMotor5 (LeftChassisMotor5Port  , 	ChassisGearset, true);
        

		pros::Motor      RightChassisMotor1(RightChassisMotor1Port ,	ChassisGearset, true);
		pros::Motor      RightChassisMotor2(RightChassisMotor2Port ,	ChassisGearset, false);
		pros::Motor      RightChassisMotor3(RightChassisMotor3Port ,	ChassisGearset, true);
        pros::Motor      RightChassisMotor4(RightChassisMotor4Port ,	ChassisGearset, false);
        pros::Motor      RightChassisMotor5(RightChassisMotor5Port ,	ChassisGearset, false);
        

		const FloatType ChassisRatio = 47.0 / 43.0;
		bool ChassisControllerMode = false;
		inline void LeftWheelMove	(FloatType Velocity){
			LeftChassisMotor1.move_velocity(Velocity );
			LeftChassisMotor2.move_velocity(Velocity );
			LeftChassisMotor3.move_velocity(Velocity );
			LeftChassisMotor4.move_velocity(Velocity );
			LeftChassisMotor5.move_velocity(Velocity );
		}

		inline void RightWheelMove (FloatType Velocity){

			RightChassisMotor1.move_velocity(Velocity );
			RightChassisMotor2.move_velocity(Velocity );
			RightChassisMotor3.move_velocity(Velocity );
			RightChassisMotor4.move_velocity(Velocity );
			RightChassisMotor5.move_velocity(Velocity );
			
		}

		inline void LeftWheelMove1	(FloatType Velocity){
			LeftChassisMotor1.move_voltage(Velocity * 20);
			LeftChassisMotor2.move_voltage(Velocity * 20);
			LeftChassisMotor3.move_voltage(Velocity * 20);
			LeftChassisMotor4.move_voltage(Velocity * 20);
			LeftChassisMotor5.move_voltage(Velocity * 20);
		}

		inline void RightWheelMove1 (FloatType Velocity){
			RightChassisMotor1.move_voltage(Velocity * 20);
			RightChassisMotor2.move_voltage(Velocity * 20);
			RightChassisMotor3.move_voltage(Velocity * 20);
			RightChassisMotor4.move_voltage(Velocity * 20);
			RightChassisMotor5.move_voltage(Velocity * 20);
		}

		FloatType LV,RV,Kv;//Kv为速度大于600时的缩小比例
		inline void MoveOpControll(FloatType CM, FloatType DM){
			CM = CM * RopoParameter::CHASSIS_RATIO * RopoParameter::RAD_TO_RPM / RopoParameter::WHEEL_RAD;
			DM = DM * RopoParameter::CHASSIS_RATIO * RopoParameter::RAD_TO_RPM / RopoParameter::WHEEL_RAD;
			LV = CM - DM * RopoParameter::CHASSIS_PARAMETER / 2.0;
			RV = CM + DM * RopoParameter::CHASSIS_PARAMETER / 2.0;
			if(fabs(LV) > RopoParameter::CHASSIS_SPPED_MAX || fabs(RV) > RopoParameter::CHASSIS_SPPED_MAX) {
				Kv = RopoParameter::CHASSIS_SPPED_MAX / fmax(fabs(LV), fabs(RV));
				LV *= Kv;
				RV *= Kv;
			}
			LeftWheelMove1(LV);
			RightWheelMove1(RV);
		}


		const int LeftIntakeMotorPort		= 10;
		const int RightIntakeMotorPort		= 20;
		const pros::motor_gearset_e_t IntakeGearset = pros::E_MOTOR_GEAR_BLUE;
		pros::Motor   LeftIntakeMotor ( LeftIntakeMotorPort  , 	IntakeGearset, true );
		pros::Motor   RightIntakeMotor ( RightIntakeMotorPort  , 	IntakeGearset, false );
		
	}

	namespace Gpss{
		static pros::Gps vexGps(RopoParameter::GPS_PORT           , RopoParameter::GPSX_INITIAL, RopoParameter::GPSY_INITIAL,
						     RopoParameter::GPS_HEADING_INITIAL, RopoParameter::GPSX_OFFSET , RopoParameter::GPSY_OFFSET);
	}

	// 创建定位模块
	namespace Position_Motor{
		RopoPosition::Position MyPosition(  Motors::LeftChassisMotor1 , Motors::LeftChassisMotor2 , Motors::LeftChassisMotor3 ,Motors::LeftChassisMotor4 , Motors::LeftChassisMotor5 ,
            Motors:: RightChassisMotor1,Motors:: RightChassisMotor2 ,Motors:: RightChassisMotor3 ,Motors:: RightChassisMotor4 ,Motors:: RightChassisMotor5 ,Sensors::Inertial);
	}


// 创建运球模块
	FloatType GetHeading(){
		return -RopoDevice::Sensors::Inertial.get_rotation() * 1.0184; 
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
		Motors::LeftChassisMotor5 .set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
		Motors::RightChassisMotor1.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
		Motors::RightChassisMotor2.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
		Motors::RightChassisMotor3.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
		Motors::RightChassisMotor4.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
		Motors::RightChassisMotor5.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);

		Motors::LeftIntakeMotor.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
		Motors::RightIntakeMotor.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
	}

	void ChassisHold(){
		Motors::LeftChassisMotor1 .set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
		Motors::LeftChassisMotor2 .set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
		Motors::LeftChassisMotor3 .set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
		Motors::LeftChassisMotor4 .set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
		Motors::LeftChassisMotor5 .set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
		Motors::RightChassisMotor1.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
		Motors::RightChassisMotor2.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
		Motors::RightChassisMotor3.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
		Motors::RightChassisMotor4.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
		Motors::RightChassisMotor5.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
	}

	void ChassisCoast(){
		Motors::LeftChassisMotor1 .set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
		Motors::LeftChassisMotor2 .set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
		Motors::LeftChassisMotor3 .set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
		Motors::LeftChassisMotor4 .set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
		Motors::LeftChassisMotor5 .set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
		Motors::RightChassisMotor1.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
		Motors::RightChassisMotor2.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
		Motors::RightChassisMotor3.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
		Motors::RightChassisMotor4.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
		Motors::RightChassisMotor5.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
	}

	void ChassisBrake(){
		Motors::LeftChassisMotor1 .set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
		Motors::LeftChassisMotor2 .set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
		Motors::LeftChassisMotor3 .set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
		Motors::LeftChassisMotor4 .set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
		Motors::LeftChassisMotor5 .set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
		Motors::RightChassisMotor1.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
		Motors::RightChassisMotor2.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
		Motors::RightChassisMotor3.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
		Motors::RightChassisMotor4.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
		Motors::RightChassisMotor5.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
	}

}

#endif
