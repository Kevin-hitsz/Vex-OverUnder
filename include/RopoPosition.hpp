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

namespace RopoPosition{
    typedef RopoApi::FloatType FloatType;

    class Position{
        private:
            static constexpr double WheelRad = 0.041275;    //轮子半径
            static constexpr double ChassisRatio = 7.0 / 5.0;  //底盘宽度
            static constexpr double Pi = 3.1415;
            pros::Motor_Group *LeftMotor;                  //指向左边三个电机的指针
            pros::Motor_Group *RightMotor;                 //指向右边电机的指针
            pros::IMU *MyInterial;
            pros::Task *BackgroundTask;
            RopoApi::FloatType Delta_S,S_Last_Encoder,S_Encoder,X,Y,VY,AY,AY0,LeftMotorEncoder,RightMotorEncoder;//
            RopoApi::FloatType Angle;
            double Get_Delta_MotorsPosition(){
                S_Last_Encoder = S_Encoder;
                std::vector<double> _LeftMotorEncoder = LeftMotor -> get_positions();        //分别得到左边三个电机的编码器位置，储存在一个向量中
                std::vector<double> _RightMotorEncoder = RightMotor -> get_positions();
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
                pros::lcd::print(2,"P!!!! %.1lf %.1lf",S_Encoder,S_Last_Encoder);   //左右编码器分别取平均值后再取平均值：直走时全正加，转弯时一正一反不变
                pros::delay(20);
		        pros::lcd::print(3,"P!  %.1lf",S_Encoder-S_Last_Encoder); 
                pros::delay(4); 
				return S_Encoder - S_Last_Encoder;        //获得的是电机编码器变化的值
            }
            static void BackgroundTaskFunction(void *Parameter){
                double SampleTime = 100;
                if(Parameter == nullptr) return;
				Position *This = static_cast<Position *>(Parameter);
                This->X = 0;
                This->Y = 0;
                This->VY = 0;
                This->LeftMotor ->tare_position();           //分别将六个编码器置零
                This->RightMotor->tare_position();

                while(true){
                    This -> Angle   = -This -> MyInterial -> get_yaw();
                    This -> Delta_S = This -> Get_Delta_MotorsPosition();
                    This -> AY      = -This -> MyInterial ->get_accel().y;

                    if(This->Angle <= 181.0 && This->Angle >= -181.0){
      
                        This -> X  += cos(This->Angle/360*2*3.1415)*This->Delta_S/360.0*2.0*Pi*WheelRad/ChassisRatio;
                        pros::delay(10);
		                pros::lcd::print(4,"P!  %.4lf",This -> X); 
                        This -> Y  += sin(This->Angle/360*2*3.1415)*This->Delta_S/360.0*2.0*Pi*WheelRad/ChassisRatio;
                    }
                    pros::delay(SampleTime);
                }
			}
        public:
            Position( pros::Motor_Group *_LeftMotor , pros::Motor_Group *_RightMotor , pros::IMU *_Interial ):
                LeftMotor(_LeftMotor),RightMotor(_RightMotor),MyInterial(_Interial),
                X(0),Y(0),S_Last_Encoder(0),S_Encoder(0),LeftMotorEncoder(0),RightMotorEncoder(0),
                Angle(0){
                LeftMotor -> set_encoder_units(pros::E_MOTOR_ENCODER_DEGREES);//以度为单位设置编码器单元
                RightMotor -> set_encoder_units(pros::E_MOTOR_ENCODER_DEGREES);
                BackgroundTask = new pros::Task(BackgroundTaskFunction,this);    
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
            double Get_VY(){
                return VY;
            }
            void SetXAndY(double _x,double _y){
                X = _x;
                Y = _y;
            }
            void initial(){
                X = 0 ;
                Y = 0 ;
                VY =0 ;
                AY =0 ;
            }
    };
}

#endif //ROPO_POSITION_HPP