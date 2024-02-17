// Code : UTF - 8
#pragma once
#include "RopoDevice.hpp"
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
#include "pros/rtos.hpp"
#include <vector>

namespace RopoDevice{


	namespace ThreeWire{
		const char ExternPneumaticPort = 'H';
		pros::ADIDigitalOut ExternPneumatic(ExternPneumaticPort,false);
	}


	namespace Sensors{
		const int InertialPort = 12;
		pros::IMU Inertial(InertialPort);

		const int EncoderReciverPort = 5;
		const int EncoderSenderPort  = 6;
		const int Boundrate = 115200;
		const int SamplingDelay = 1;
		RopoSensor::EncodingDisk Encoder(EncoderReciverPort,Boundrate,EncoderSenderPort,Boundrate,SamplingDelay);
		Matrix GetPosition(){
			Matrix Position(3,1);
			if(RopoDevice::Sensors::Encoder.IsReading() == true){
				Position[1][1] = RopoDevice::Sensors::Encoder.GetPosX() / 1000.0;
				Position[2][1] = RopoDevice::Sensors::Encoder.GetPosY() / 1000.0;
			}
			Position[3][1] = - RopoDevice::Sensors::Inertial.get_yaw() / 180.0 * RopoMath::Pi;
			return Position;
		}
	}			
	
	namespace Motors{
		const int LFMotorPort  	=  1;
		const int LFMotorPort_ 	=  2;
		const int LBMotorPort  	=  3;
		const int LBMotorPort_ 	=  4;
		const int RFMotorPort  	=  9;
		const int RFMotorPort_ 	= 10;
		const int RBMotorPort  	=  7;
		const int RBMotorPort_ 	=  8;
		const int IntakeMotorPort  = 16;
		const int ClimberMotorPort1 = 11;
		const int ClimberMotorPort2 = 20;
		const int ShooterMotorPort = 19;
		Motor LFMotor (LFMotorPort,pros::E_MOTOR_GEAR_BLUE, true);
		Motor LFMotor_(LFMotorPort_,pros::E_MOTOR_GEAR_BLUE,true);
		Motor LBMotor (LBMotorPort,pros::E_MOTOR_GEAR_BLUE, true);
		Motor LBMotor_(LBMotorPort_,pros::E_MOTOR_GEAR_BLUE,true);		
		Motor RFMotor (RFMotorPort,pros::E_MOTOR_GEAR_BLUE, true);
		Motor RFMotor_(RFMotorPort_,pros::E_MOTOR_GEAR_BLUE,true);
		Motor RBMotor (RBMotorPort,pros::E_MOTOR_GEAR_BLUE, true);
		Motor RBMotor_(RBMotorPort_,pros::E_MOTOR_GEAR_BLUE,true);
		Motor IntakeMotor(IntakeMotorPort,pros::E_MOTOR_GEAR_BLUE, true);
		Motor ClimberMotor1(ClimberMotorPort1,pros::E_MOTOR_GEAR_BLUE, true);
		Motor ClimberMotor2(ClimberMotorPort2,pros::E_MOTOR_GEAR_BLUE, false);
		Motor ShooterMotor(ShooterMotorPort,pros::E_MOTOR_GEAR_RED, true);		

	}

	void DeviceIni(){
		Sensors::Inertial.reset(false);
		pros::delay(200);
		Sensors::Encoder.SetZero();
		Motors::ShooterMotor.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
	}

	RopoDiffySwerve::DiffySwerve LF(Motors::LFMotor,Motors::LFMotor_);
	RopoDiffySwerve::DiffySwerve LB(Motors::LBMotor,Motors::LBMotor_);
	RopoDiffySwerve::DiffySwerve RF(Motors::RFMotor,Motors::RFMotor_);
	RopoDiffySwerve::DiffySwerve RB(Motors::RBMotor,Motors::RBMotor_);
	Chassis Chassis(LF,LB,RF,RB,Sensors::GetPosition);
}