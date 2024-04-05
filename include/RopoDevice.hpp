// Code : UTF - 8
#pragma once
#include "RopoDevice.hpp"
#include "RopoDiffySwerve.hpp"
#include "pros/imu.hpp"
#include "pros/adi.hpp"
#include "pros/motors.h"
#include "pros/motors.hpp"
#include "RopoSensor/EncodingDisk.hpp"
#include "RopoApi.hpp"
#include "RopoChassis.hpp"
#include "pros/rtos.hpp"
#include <vector>

namespace RopoDevice{


	namespace ThreeWire{
		const char LExternPneumaticPort = 'H';
		pros::ADIDigitalOut LExternPneumatic(LExternPneumaticPort,false);
        const char RExternPneumaticPort = 'G';
		pros::ADIDigitalOut RExternPneumatic(RExternPneumaticPort,false);
        const char ClimberPneumaticPort = 'A';
		pros::ADIDigitalOut ClimberPneumatic(ClimberPneumaticPort,false);
        const char IntakerPneumaticPort = 'F';
		pros::ADIDigitalOut IntakerPneumatic(IntakerPneumaticPort,false);
        const char ShooterPneumaticPort = 'C';
		pros::ADIDigitalOut ShooterPneumatic(ShooterPneumaticPort,false);
	}


	namespace Sensors{
		const int InertialPort = 5;
		pros::IMU Inertial(InertialPort);

		const int EncoderReciverPort = 9;
		const int EncoderSenderPort  = 19;
		const int Boundrate = 115200;
		const int SamplingDelay = 1;
		RopoSensor::EncodingDisk Encoder(EncoderReciverPort,Boundrate,EncoderSenderPort,Boundrate,SamplingDelay,0,68.0);
		
        /* Matrix GetPosition(){
			Matrix Position(3,1);
			if(RopoDevice::Sensors::Encoder.IsReading() == true){
				Position[2][1] =-RopoDevice::Sensors::Encoder.GetPosX() / 1000.0;
				Position[1][1] = RopoDevice::Sensors::Encoder.GetPosY() / 1000.0;
			}
			Position[3][1] = - RopoDevice::Sensors::Inertial.get_yaw() / 180.0 * RopoMath::Pi;
			return Position;
		} */

        //GPS在19号口
	}			
	
	namespace Motors{
		const int LFMotorPort  	=  10;
		const int LFMotorPort_ 	=  4;
		const int LBMotorPort  	=  7;
		const int LBMotorPort_ 	=  8;
		const int RFMotorPort  	=  20;
		const int RFMotorPort_ 	=  12;
		const int RBMotorPort  	=  18;
		const int RBMotorPort_ 	=  17;

		const int IntakeMotorPort  = 16;
		const int LShooterMotorPort = 1;
        const int RShooterMotorPort = 11;
        const int HitterMotorPort = 9;

		Motor LFMotor (LFMotorPort,pros::E_MOTOR_GEAR_BLUE, true,pros::E_MOTOR_ENCODER_DEGREES);
		Motor LFMotor_(LFMotorPort_,pros::E_MOTOR_GEAR_BLUE,true,pros::E_MOTOR_ENCODER_DEGREES);
		Motor LBMotor (LBMotorPort,pros::E_MOTOR_GEAR_BLUE, true,pros::E_MOTOR_ENCODER_DEGREES);
		Motor LBMotor_(LBMotorPort_,pros::E_MOTOR_GEAR_BLUE,true,pros::E_MOTOR_ENCODER_DEGREES);		
		Motor RFMotor (RFMotorPort,pros::E_MOTOR_GEAR_BLUE, true,pros::E_MOTOR_ENCODER_DEGREES);
		Motor RFMotor_(RFMotorPort_,pros::E_MOTOR_GEAR_BLUE,true,pros::E_MOTOR_ENCODER_DEGREES);
		Motor RBMotor (RBMotorPort,pros::E_MOTOR_GEAR_BLUE, true,pros::E_MOTOR_ENCODER_DEGREES);
		Motor RBMotor_(RBMotorPort_,pros::E_MOTOR_GEAR_BLUE,true,pros::E_MOTOR_ENCODER_DEGREES);
		Motor IntakeMotor(IntakeMotorPort,pros::E_MOTOR_GEAR_BLUE, true);
		Motor LShooterMotor(LShooterMotorPort,pros::E_MOTOR_GEAR_RED, true);	
        Motor RShooterMotor(RShooterMotorPort,pros::E_MOTOR_GEAR_RED, true);
        Motor HitterMotor(HitterMotorPort,pros::E_MOTOR_GEAR_RED, true);		

	}

    void DeviceIni(){
        Sensors::Inertial.reset(false);
        //while(Sensors::Inertial.is_calibrating())pros::delay(20);
        pros::delay(2000);
        Sensors::Encoder.SetZero();
        Motors::LShooterMotor.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
        Motors::RShooterMotor.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
        Motors::HitterMotor.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
}

RopoDiffySwerve::DiffySwerve LF(Motors::LFMotor,Motors::LFMotor_); // Define the variable in a source file
RopoDiffySwerve::DiffySwerve LB(Motors::LBMotor,Motors::LBMotor_);
RopoDiffySwerve::DiffySwerve RF(Motors::RFMotor,Motors::RFMotor_);
RopoDiffySwerve::DiffySwerve RB(Motors::RBMotor,Motors::RBMotor_);
Chassis Chassis(LF,LB,RF,RB,Sensors::Inertial, Sensors::Encoder);

/* bool Position_OK = false;
bool Time_Out = false;
static constexpr float ControlTime = 20; // ms
FloatType XYMinError = 0.01;
FloatType ThetaMinError = 0.01;
int counter_for_error = 0;
const int max_counter = 50;
int counter_for_time = 0;
int max_time = 1000; // ms
Matrix AimPosition = Matrix(3,1);
Matrix Velocity = Matrix(3,1); 
Matrix ActualPosition = Matrix(3,1);
Matrix PositionError = Matrix(3,1);
Matrix Kp = Matrix(3,3);
Matrix Ki = Matrix(3,3);
Matrix Integrator = Matrix(3,1);
Matrix Parameter = Matrix(3,3);

void PositionControl(){
    Kp[1][1] = 3.5; Kp[2][2] = 3.5; Kp[3][3] = 5;
    while(Chassis.Status == Chassis.ChassisStatus::autonomous){
        //ActualPosition = Chassis.UpdatePosition();
        PositionError = AimPosition - ActualPosition;
        // 限定作用域 
        if(fabsf(PositionError[3][1]) > RopoMath::Pi) PositionError[3][1] -= 2 * RopoMath::Pi * RopoMath::Sign(PositionError[3][1]);
        // 减少震荡
        if(fabsf(PositionError[1][1]) < XYMinError && fabsf(PositionError[2][1]) < XYMinError && fabsf(PositionError[3][1]) < ThetaMinError){
            counter_for_error++;
            if (counter_for_error > max_counter){
                Position_OK = true;
                counter_for_error = max_counter;
            }
        }else{
            counter_for_error = 0;
            Position_OK = false;
        }
        if(Position_OK) Velocity[1][1] = Velocity[2][1] = Velocity[3][1] = 0;
        else{
            Velocity = Kp * PositionError;
            Velocity[1][1] = fabsf(Velocity[1][1]) > 1.2 ? 1.2 * RopoMath::Sign(Velocity[1][1]) : Velocity[1][1];
            Velocity[2][1] = fabsf(Velocity[2][1]) > 1.2 ? 1.2 * RopoMath::Sign(Velocity[2][1]) : Velocity[2][1];
            Velocity[3][1] = fabsf(Velocity[3][1]) > (1.5 * RopoMath::Pi) ? (1.5 * RopoMath::Pi * RopoMath::Sign(Velocity[3][1])) : Velocity[3][1];
            // Rotaion Matrix
            Parameter[1][1] = cosf(ActualPosition[3][1]) , Parameter[1][2] = sinf(ActualPosition[3][1]) , Parameter[1][3] = 0;
            Parameter[2][1] =-sinf(ActualPosition[3][1]) , Parameter[2][2] = cosf(ActualPosition[3][1]) , Parameter[2][3] = 0;
            Parameter[3][1] = 0                          , Parameter[3][2] = 0                          , Parameter[3][3] = 1;
            Velocity = Parameter * Velocity;
        }
        Velocity[2][1] = -Velocity[2][1];
        Chassis.SetAimStatus(Velocity);


        if(counter_for_time * ControlTime > max_time) Time_Out = true;
        counter_for_time++;
        pros::delay(ControlTime);
    }
} */
/* void SetPosition(FloatType x, FloatType y, FloatType theta, int _max_time){
    counter_for_time = 0;
    counter_for_error = 0;
    Integrator[1][1] = Integrator[2][1] = Integrator[3][1] = 0;
    max_time = _max_time;
    Position_OK = false;
    Time_Out = false;
    AimPosition[1][1] = x;
    AimPosition[2][1] = y;
    AimPosition[3][1] = theta;
    while (!Position_OK && !Time_Out) pros::delay(20);
} */
}