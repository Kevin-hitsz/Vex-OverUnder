#pragma once
#include "api.h"
#include "pros/imu.hpp"
#include "RopoApi.hpp"
#include "pros/motors.hpp"
#include "pros/rtos.hpp"
#include <cmath>
#include <numeric>
#include <vector>
#include "RopoMath/Misc.hpp"

namespace RopoPosition{
    typedef RopoApi::FloatType FloatType;

    class Position{
        private:
            static constexpr double WheelRad = 0.041275;        // 轮半径
            static constexpr double ChassisRatio = 3.0 / 2.0;   // 传动比
            static constexpr double Pi = 3.1415926;
            pros::Motor_Group *LeftMotor;                  // 指向左边三个电机的指针
            pros::Motor_Group *RightMotor;                 // 指向右边电机的指针
            pros::IMU *MyInterial;
            pros::Task *BackgroundTask;
            RopoApi::FloatType Delta_Distance,S_Last_Encoder,S_Encoder,X,Y,LeftMotorEncoder,RightMotorEncoder;
            RopoApi::FloatType Angle;
            double Get_Delta_MotorsPosition(){
                S_Last_Encoder = S_Encoder;

                // 获取两侧三电机的编码器位置
                std::vector<double> _LeftMotorEncoder = LeftMotor -> get_positions();
                std::vector<double> _RightMotorEncoder = RightMotor -> get_positions();
                
                // 平均值
                LeftMotorEncoder  = -1 * std::accumulate( _LeftMotorEncoder.begin(), _LeftMotorEncoder.end(),0) /  (FloatType)_LeftMotorEncoder.size();
                RightMotorEncoder = std::accumulate(_RightMotorEncoder.begin(),_RightMotorEncoder.end(),0) / (FloatType)_RightMotorEncoder.size();

                // 总平均值
                S_Encoder = (LeftMotorEncoder + RightMotorEncoder) / 2.0;       
		        pros::lcd::print(2,"P!  %.1lf",S_Encoder-S_Last_Encoder); 
                pros::delay(10); 
				return S_Encoder - S_Last_Encoder;        // 返回差值
            }
            static void BackgroundTaskFunction(void *Parameter){
                if(Parameter == nullptr) return;
				Position *This = static_cast<Position*>(Parameter);
                double SampleTime = 10;
                This->X = 0;
                This->Y = 0;
                This->LeftMotor ->tare_position();
                This->RightMotor->tare_position();
                while(1){
                    This -> Angle   = -This -> MyInterial -> get_yaw();
                    This -> Delta_Distance = This -> Get_Delta_MotorsPosition() / 180*Pi * WheelRad/ChassisRatio;
                    if(This->Angle <= 180.0 && This->Angle >= -180.0){               
                        This -> X  += cos(This->Angle/180*Pi) * This->Delta_Distance;
                        This -> Y  += sin(This->Angle/180*Pi) * This->Delta_Distance;
                    }
                    pros::delay(SampleTime);
                }
			}
        public:
            Position( pros::Motor_Group *_LeftMotor , pros::Motor_Group *_RightMotor , pros::IMU *_Interial ):
                LeftMotor(_LeftMotor),RightMotor(_RightMotor),MyInterial(_Interial),
                X(0),Y(0),S_Last_Encoder(0),S_Encoder(0),LeftMotorEncoder(0),RightMotorEncoder(0),
                Angle(0){
                // 以 Degree 为单位设置编码器单位
                LeftMotor -> set_encoder_units(pros::E_MOTOR_ENCODER_DEGREES); 
                RightMotor -> set_encoder_units(pros::E_MOTOR_ENCODER_DEGREES);
                BackgroundTask = new pros::Task(BackgroundTaskFunction,this);    
            };
            FloatType Get_X() const{return X;}
            FloatType Get_Y() const{return Y;}
            FloatType Get_Angle() const{return Angle;}
            void initial(){X = Y = 0;}
    };
}