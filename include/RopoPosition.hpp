#ifndef ROPO_POSITION_HPP
#define ROPO_POSITION_HPP
#include "api.h"
#include "pros/imu.hpp"
#include "RopoApi.hpp"
#include "pros/motors.hpp"
#include "pros/rtos.hpp"
#include <cmath>
#include <vector>
#include "RopoMath/Misc.hpp"
#include"RopoDevice.hpp"

namespace RopoPosition{
    double WheelRad = 0.041275;    //轮子半径
    double ChassisRatio = 7.0 / 5.0;  //底盘宽度
    double Pi = 3.1415;
    pros::Task *BackgroundTask;
    double Delta_S,S_Last_Encoder,S_Encoder,X,Y,LeftMotorEncoder,RightMotorEncoder;//
    double Angle;
    double Get_Delta_MotorsPosition(){
        S_Last_Encoder = S_Encoder;
        std::vector<double> _LeftMotorEncoder  = RopoDevice::Motors:: LeftMotor.get_positions();        //分别得到左边三个电机的编码器位置，储存在一个向量中
        std::vector<double> _RightMotorEncoder = RopoDevice::Motors::RightMotor.get_positions();
        double ResPosition = 0,Cnt = 0;
        S_Encoder = 0;
        for(double i : _LeftMotorEncoder)               //计算三个电机编码器的平均值
            ResPosition += i,Cnt += 1;
        ResPosition /= Cnt;
        S_Encoder += ResPosition;
        LeftMotorEncoder = ResPosition;
        ResPosition = 0,Cnt = 0;
        for(double i : _RightMotorEncoder)
            ResPosition += i,Cnt += 1;
        ResPosition /= Cnt;
        RightMotorEncoder = ResPosition;
        S_Encoder += ResPosition;
        S_Encoder /= 2.0; 
        // pros::lcd::print(2,"P!!!! %.1lf %.1lf",S_Encoder,S_Last_Encoder);   //左右编码器分别取平均值后再取平均值：直走时全正加，转弯时一正一反不加
        pros::delay(20);
        // pros::lcd::print(3,"P!  %.1lf",S_Encoder-S_Last_Encoder); 
        pros::delay(4); 
        return S_Encoder - S_Last_Encoder;        //获得的是电机编码器变化的量
    }
    void BackgroundTaskFunction(){
        X = 0;
        Y = 0;
        RopoDevice::Motors::LeftMotor  .tare_position();           //分别将六个编码器置零
        RopoDevice::Motors::RightMotor .tare_position();
        while(true){
            Angle = -RopoDevice::Sensors::Inertial.get_yaw();
            Delta_S =  Get_Delta_MotorsPosition();
            static constexpr double ChassisRatio = 7.0 / 5.0;
            X += cos(Angle/360*2*Pi)*Delta_S / 360.0 * 2.0 * Pi * WheelRad / ChassisRatio;//cos(This->Angle)//*This->Delta_S/360.0*2.0*Pi*WheelRad/ChassisRatio;
            Y += sin(Angle/360*2*Pi)*Delta_S / 360.0 * 2.0 * Pi * WheelRad / ChassisRatio;//sin(This->Angle)//*This->Delta_S/360.0*2.0*Pi*WheelRad/ChassisRatio;
            pros::delay(20);
        }
    }
    void StartPosition(){
        RopoDevice::Motors::LeftMotor  . set_encoder_units(pros::E_MOTOR_ENCODER_DEGREES);//以度为单位设置编码器单元
        RopoDevice::Motors::RightMotor . set_encoder_units(pros::E_MOTOR_ENCODER_DEGREES);
        BackgroundTask = new pros::Task(BackgroundTaskFunction);    
    };
    double Get_X(){
        return X;
    }
    double Get_Y(){
        return Y;
    }
    double Get_Angle(){
        return Angle;
    }
}

#endif //ROPO_POSITION_HPP