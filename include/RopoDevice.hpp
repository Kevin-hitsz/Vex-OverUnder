// Code : UTF - 8
#ifndef ROPO_DEVICE_HPP
#define ROPO_DEVICE_HPP

#include "pros/imu.hpp"
#include "pros/adi.hpp"
#include "pros/motors.h"
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
		const char WideExternPneumaticPort = 'F';
		pros::ADIDigitalOut WideExternPneumatic(WideExternPneumaticPort,false);
		
		const char UnderExternPneumaticPort = 'B';
		pros::ADIDigitalOut UnderExternPneumatic(UnderExternPneumaticPort,false);

		const char LeftExternPneumaticPort = 'D';
		pros::ADIDigitalOut LeftExternPneumatic(LeftExternPneumaticPort,false);

		const char RightExternPneumaticPort = 'E';
		pros::ADIDigitalOut RightExternPneumatic(RightExternPneumaticPort,false);

		const char CatchPneumaticPort  = 'C';
		pros::ADIDigitalOut CatchPneumatic(CatchPneumaticPort,false);

	}

//创建惯性传感器
	namespace Sensors{
		const int InertialPort = 16;
		pros::IMU Inertial(InertialPort);
		const int OpenmvPort = 19;
		RopoSensor::OpenMv My_openMV(OpenmvPort,115200);
	}			
	
	// 创建电机
	namespace Motors{

		const int LeftChassisMotor1Port  	= 4;
		const int LeftChassisMotor2Port  	= 3;
		const int LeftChassisMotor3Port  	= 5;
        const int LeftChassisMotor4Port  	= 1;
		const int RightChassisMotor1Port	= 14;
		const int RightChassisMotor2Port	= 13;
		const int RightChassisMotor3Port	= 12;
		const int RightChassisMotor4Port	= 11;
		
		const pros::motor_gearset_e_t ChassisGearset = pros::E_MOTOR_GEAR_BLUE;

		pros::Motor      LeftChassisMotor1 (LeftChassisMotor1Port  , 	ChassisGearset, false);
		pros::Motor      LeftChassisMotor2 (LeftChassisMotor2Port  , 	ChassisGearset, true);
		pros::Motor      LeftChassisMotor3 (LeftChassisMotor3Port  , 	ChassisGearset, true);
        pros::Motor      LeftChassisMotor4 (LeftChassisMotor4Port  , 	ChassisGearset, false);
        

		pros::Motor      RightChassisMotor1(RightChassisMotor1Port ,	ChassisGearset, false);
		pros::Motor      RightChassisMotor2(RightChassisMotor2Port ,	ChassisGearset, true);
		pros::Motor      RightChassisMotor3(RightChassisMotor3Port ,	ChassisGearset, true);
        pros::Motor      RightChassisMotor4(RightChassisMotor4Port ,	ChassisGearset, false);
		
		void LeftWheelMove(FloatType Velocity){
			LeftChassisMotor1.move_velocity(-Velocity);
			LeftChassisMotor2.move_velocity(-Velocity);
			LeftChassisMotor3.move_velocity(-Velocity);
			LeftChassisMotor4.move_velocity(-Velocity);
		}
		
		void RightWheelMove(FloatType Velocity){
			RightChassisMotor1.move_velocity(Velocity);
			RightChassisMotor2.move_velocity(Velocity);
			RightChassisMotor3.move_velocity(Velocity);
			RightChassisMotor4.move_velocity(Velocity);
		}

		void LeftWheelMoveVoltage(FloatType Velocity){
			static constexpr float VecToVolRatio = RopoParameter::CHASSIS_SPPED_MAX_VOLTAGE / RopoParameter::CHASSIS_SPPED_MAX;
			//FloatType _Velocity = Velocity > RopoParameter::CHASSIS_SPPED_MAX ? RopoParameter::CHASSIS_SPPED_MAX : Velocity;
			LeftChassisMotor1.move_voltage(-Velocity * VecToVolRatio);
			LeftChassisMotor2.move_voltage(-Velocity * VecToVolRatio);
			LeftChassisMotor3.move_voltage(-Velocity * VecToVolRatio);
			LeftChassisMotor4.move_voltage(-Velocity * VecToVolRatio);
		}

		void RightWheelMoveVoltage(FloatType Velocity){
			static constexpr float VecToVolRatio = RopoParameter::CHASSIS_SPPED_MAX_VOLTAGE / RopoParameter::CHASSIS_SPPED_MAX;
			//FloatType _Velocity = Velocity > RopoParameter::CHASSIS_SPPED_MAX ? RopoParameter::CHASSIS_SPPED_MAX : Velocity;
			RightChassisMotor1.move_voltage(Velocity * VecToVolRatio);
			RightChassisMotor2.move_voltage(Velocity * VecToVolRatio);
			RightChassisMotor3.move_voltage(Velocity * VecToVolRatio);
			RightChassisMotor4.move_voltage(Velocity * VecToVolRatio);
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
			LeftWheelMoveVoltage(LV);
			RightWheelMoveVoltage(RV);
		}

		const int LeftLiftMotorPort		= 10;
		const int RightLiftMotorPort		= 8;
		const pros::motor_gearset_e_t LiftGearset = pros::E_MOTOR_GEAR_RED;
		
		pros::Motor   LeftLiftMotor  ( LeftLiftMotorPort  , 	LiftGearset, true );
		pros::Motor   RightLiftMotor ( RightLiftMotorPort  , 	LiftGearset, false );

		const int LeftIntakeMotorPort		= 9;
		const int RightIntakeMotorPort		= 20;
		const pros::motor_gearset_e_t IntakeGearset = pros::E_MOTOR_GEAR_GREEN;
		pros::Motor   LeftIntakeMotor ( LeftIntakeMotorPort  , 	IntakeGearset, true );
		pros::Motor   RightIntakeMotor( RightIntakeMotorPort , 	IntakeGearset, false );

		void IntakerMoveVoltage(FloatType voltage){
			LeftIntakeMotor.move_voltage(voltage);
			RightIntakeMotor.move_voltage(voltage);
		}
		
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
	RopoLifter::LifterModule LiftMotors(Motors::LeftLiftMotor, Motors::RightLiftMotor);

	FloatType GetHeading(){
		return -RopoDevice::Sensors::Inertial.get_rotation()*1.017; 	// 修正每圈6度的误差
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

	RopoGpsAddPosition::GpsAddPositionModule gpsAddPosition(GetPosition,Gpss::vexGps,20,0 );

	Vector GetTransformedPosition(){
		return gpsAddPosition.GetTransformedPosition();
	}
    
	//	创建底盘
	RopoChassis::TankChassis Chassis( Motors::RightWheelMove , Motors::LeftWheelMove , GetTransformedPosition , 1 );

//初始化
	void DeviceInit(){
		RopoDevice::Chassis.SetVelocityLimits(600);
        Sensors::Inertial.reset(true);
        if(Sensors::Inertial.get_yaw() != PROS_ERR_F){
			while(Sensors::Inertial.is_calibrating())pros::delay(20);
		}
		pros::delay(200);
		Position_Motor::MyPosition.initial();
	}

	void MotorsInit(){
		Motors::LeftChassisMotor1 .set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
		Motors::LeftChassisMotor2 .set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
		Motors::LeftChassisMotor3 .set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
		Motors::LeftChassisMotor4 .set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
		Motors::RightChassisMotor1.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
		Motors::RightChassisMotor2.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
		Motors::RightChassisMotor3.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
		Motors::RightChassisMotor4.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);

		Motors::RightLiftMotor.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
		Motors::LeftIntakeMotor.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
		Motors::RightIntakeMotor.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
	}

	void ChassisBrake(){
		Motors::LeftChassisMotor1 .set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
		Motors::LeftChassisMotor2 .set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
		Motors::LeftChassisMotor3 .set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
		Motors::LeftChassisMotor4 .set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
		Motors::RightChassisMotor1.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
		Motors::RightChassisMotor2.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
		Motors::RightChassisMotor3.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
		Motors::RightChassisMotor4.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
	}

	void ChassisHold(){
		Motors::LeftChassisMotor1 .set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
		Motors::LeftChassisMotor2 .set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
		Motors::LeftChassisMotor3 .set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
		Motors::LeftChassisMotor4 .set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
		Motors::RightChassisMotor1.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
		Motors::RightChassisMotor2.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
		Motors::RightChassisMotor3.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
		Motors::RightChassisMotor4.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
	}

	void ChassisCoast(){
		Motors::LeftChassisMotor1 .set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
		Motors::LeftChassisMotor2 .set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
		Motors::LeftChassisMotor3 .set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
		Motors::LeftChassisMotor4 .set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
		Motors::RightChassisMotor1.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
		Motors::RightChassisMotor2.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
		Motors::RightChassisMotor3.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
		Motors::RightChassisMotor4.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
	}

}

#endif
