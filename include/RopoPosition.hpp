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
#include <iostream>
#include "RopoParameter.hpp"
#include "RoPoPros/RopoMotorGroup.hpp"
#include "RoPoPros/RopoInertial.hpp"
namespace RopoPosition{
    class Position{
        private:
            static constexpr double WheelRad = RopoParameter::WHEEL_RAD;        // 轮半径
            static constexpr double ChassisRatio = RopoParameter::CHASSIS_RATIO;   // 传动比
            static constexpr double Pi = 3.1415926;
            RopoMotorGroup::MotorGroup& LeftMotorGroup;
            RopoMotorGroup::MotorGroup& RightMotorGroup;

            
            RopoInertial &MyInterial;
            pros::Task *BackgroundTask;
            FloatType Delta_Distance,S_Last_Encoder,S_Encoder,X,Y,LeftMotorEncoder,RightMotorEncoder;
            FloatType Angle;

            double Get_Delta_MotorsPosition(){
                S_Last_Encoder = S_Encoder;
                // 对通信正常的电机的编码器进行求平均值
                LeftMotorEncoder=LeftMotorGroup.get_position();
                RightMotorEncoder=RightMotorGroup.get_position();
                // 总平均值
                S_Encoder = (LeftMotorEncoder + RightMotorEncoder) / 2.0;  
                return S_Encoder - S_Last_Encoder;        // 返回差值
            }

            static void BackgroundTaskFunction(void *Parameter){
                if(Parameter == nullptr) return;
				Position *This = static_cast<Position*>(Parameter);
                //采样间隔
                double SampleTime = 10;
                //初始化
                This->X = 0;
                This->Y = 0;
                This->LeftMotorGroup.tare_position();
                This->RightMotorGroup.tare_position();
             
                while(1){
                    This -> Angle   = This -> MyInterial .get_yaw();
                    This -> Delta_Distance = This -> Get_Delta_MotorsPosition() / 180*Pi * WheelRad/ChassisRatio;
                    if(This->Angle <= 180.0 && This->Angle >= -180.0)
                    {
                        This -> X  += cos(This->Angle/180*Pi) * This->Delta_Distance;
                        This -> Y  += sin(This->Angle/180*Pi) * This->Delta_Distance;
                    }
                    pros::lcd::print(1,"X:%.4f,Y:%.4f,theta:%f",This->X,This->Y,This -> Angle);
                    pros::delay(SampleTime);
                }
			}
        public:
            Position( RopoMotorGroup::MotorGroup &Left ,RopoMotorGroup::MotorGroup &Right,
            RopoInertial & Interial ):
            LeftMotorGroup(Left),
            RightMotorGroup(Right),
            MyInterial(Interial),BackgroundTask(nullptr),
            Delta_Distance(0),S_Last_Encoder(0),S_Encoder(0),X(0),Y(0),LeftMotorEncoder(0),RightMotorEncoder(0),
            Angle(0)
            {
                // 以 Degree 为单位设置编码器单位
                LeftMotorGroup.set_encoder_units(pros::E_MOTOR_ENCODER_DEGREES);
                RightMotorGroup.set_encoder_units(pros::E_MOTOR_ENCODER_DEGREES);
        
                BackgroundTask = new pros::Task(BackgroundTaskFunction,this);    
            };

            FloatType Get_X() const{return X;}
            FloatType Get_Y() const{return Y;}
            FloatType Get_Angle() const{return Angle;}

            RopoMath::Vector<FloatType> GetPosition()
            {
		        RopoMath::Vector<FloatType> PositionVector(RopoMath::ColumnVector,3);
		        PositionVector[1] = Get_X();
		        PositionVector[2] = Get_Y();
		        PositionVector[3] = Get_Angle();
		        return PositionVector;
	        }

            void Set_XY(FloatType x, FloatType y){
                X = x;
                Y = y; 
            }
            void Set_Angle(FloatType angle){
                Angle = angle;
            }
            void initial(){X = Y = 0;}
    };
}