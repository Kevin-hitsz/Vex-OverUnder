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

namespace RopoPosition{
    class Position{
        private:
            static constexpr double WheelRad = 0.034925;        // 轮半径
            static constexpr double ChassisRatio = 56.0 / 44.0;   // 传动比
            static constexpr double Pi = 3.1415926;
            pros::Motor &LeftMotor1;
            pros::Motor &LeftMotor2;
            pros::Motor &LeftMotor3;
            pros::Motor &LeftMotor4;
            
            pros::Motor &RightMotor1;
            pros::Motor &RightMotor2;
            pros::Motor &RightMotor3;
            pros::Motor &RightMotor4;
 
            pros::IMU &MyInterial;
            pros::Task *BackgroundTask;
            FloatType Delta_Distance,S_Last_Encoder,S_Encoder,X,Y,LeftMotorEncoder,RightMotorEncoder;
            FloatType Angle;

            double Get_Delta_MotorsPosition(){
                S_Last_Encoder = S_Encoder;

                // 对通信正常的电机的编码器进行求平均值
                FloatType L1=LeftMotor1.get_position();     //临时变量
                FloatType L2=LeftMotor2.get_position();
                FloatType L3=LeftMotor3.get_position();
                FloatType L4=LeftMotor4.get_position();
                
                FloatType R1=RightMotor1.get_position();
                FloatType R2=RightMotor2.get_position();
                FloatType R3=RightMotor3.get_position();
                FloatType R4=RightMotor4.get_position();

                LeftMotorEncoder=-1*((std::isinf(L1)?0:L1)+(std::isinf(L2)?0:L2)+(std::isinf(L3)?0:L3)+(std::isinf(L4)?0:L4))/
                                    ((std::isinf(L1)?0:1.0)+(std::isinf(L2)?0:1.0)+(std::isinf(L3)?0:1.0)+(std::isinf(L4)?0:1.0));
                RightMotorEncoder=((std::isinf(R1)?0:R1)+(std::isinf(R2)?0:R2)+(std::isinf(R3)?0:R3)+(std::isinf(R4)?0:R4))/
                                 ((std::isinf(R1)?0:1.0)+(std::isinf(R2)?0:1.0)+(std::isinf(R3)?0:1.0)+(std::isinf(R4)?0:1.0));
                
                
                // 总平均值
                S_Encoder = (LeftMotorEncoder + RightMotorEncoder) / 2.0;  
                return S_Encoder - S_Last_Encoder;        // 返回差值
            }

            static void BackgroundTaskFunction(void *Parameter){
                if(Parameter == nullptr) return;
				Position *This = static_cast<Position*>(Parameter);
                //采样间隔
                double SampleTime = 50;
                //初始化
                This->X = 0;
                This->Y = 0;
                This->LeftMotor1.tare_position();
                This->LeftMotor2.tare_position();
                This->LeftMotor3.tare_position();
                This->LeftMotor4.tare_position();
                
                This->RightMotor1.tare_position();
                This->RightMotor2.tare_position();
                This->RightMotor3.tare_position();
                This->RightMotor4.tare_position();
                while(1){
                    This -> Angle   = - This -> MyInterial . get_yaw();
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
            Position( pros::Motor &_LeftMotor1 ,pros::Motor &_LeftMotor2 ,pros::Motor &_LeftMotor3 ,pros::Motor &_LeftMotor4 ,
            pros::Motor &_RightMotor1,pros::Motor &_RightMotor2 ,pros::Motor &_RightMotor3 ,pros::Motor &_RightMotor4 ,
            pros::IMU &_Interial ):
            LeftMotor1(_LeftMotor1),
            LeftMotor2(_LeftMotor2),
            LeftMotor3(_LeftMotor3),
            LeftMotor4(_LeftMotor4),
            
            RightMotor1(_RightMotor1),
            RightMotor2(_RightMotor2),
            RightMotor3(_RightMotor3),
            RightMotor4(_RightMotor4),

            MyInterial(_Interial),BackgroundTask(nullptr),
            Delta_Distance(0),S_Last_Encoder(0),S_Encoder(0),X(0),Y(0),LeftMotorEncoder(0),RightMotorEncoder(0),
            Angle(0)
            {
                // 以 Degree 为单位设置编码器单位
                LeftMotor1.set_encoder_units(pros::E_MOTOR_ENCODER_DEGREES);
                LeftMotor2.set_encoder_units(pros::E_MOTOR_ENCODER_DEGREES);
                LeftMotor3.set_encoder_units(pros::E_MOTOR_ENCODER_DEGREES);
                LeftMotor4.set_encoder_units(pros::E_MOTOR_ENCODER_DEGREES);
                
                RightMotor1.set_encoder_units(pros::E_MOTOR_ENCODER_DEGREES);
                RightMotor2.set_encoder_units(pros::E_MOTOR_ENCODER_DEGREES);
                RightMotor3.set_encoder_units(pros::E_MOTOR_ENCODER_DEGREES);
                RightMotor4.set_encoder_units(pros::E_MOTOR_ENCODER_DEGREES);

                BackgroundTask = new pros::Task(BackgroundTaskFunction,this);    
            };


            FloatType Get_X() const{return X;}
            FloatType Get_Y() const{return Y;}
            FloatType Get_Angle() const{return Angle;}

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