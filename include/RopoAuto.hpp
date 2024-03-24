# pragma once

#include "RopoApi.hpp"
#include "RopoControl/Regulator.hpp"
#include "RopoDevice.hpp"
#include "api.h"
#include "RopoMath/Misc.hpp"
#include "pros/rtos.hpp"
#include <cmath>
#include <iostream>
#include "RopoSensor/OpenMv.hpp"
#include "RopoChassis.hpp"
#include <cstdio>
namespace RopoAuto{
    /*
    struct PID{   
        RopoApi::FloatType err = 0;               
        RopoApi::FloatType err_next = 0;          
        RopoApi::FloatType err_last = 0;        
        RopoApi::FloatType Kp = 0.15 ,Ki = 0.08 ,Kd = 0.01;   
        RopoApi::FloatType deltaError = 1.0;
        bool ifArrive = false;
    }pid;

    void Excution_Rotation(RopoApi::FloatType Degree){
        RopoApi::FloatType W_ = (Degree - RopoDevice::Sensors::Inertial.get_heading()) * 1; // / 180 * RopoMath::Pi / 2;
        pros::lcd::print(2, "%.5f", W_);
        RopoDevice::Chassis.MoveVelocity(0, -W_);
    }
    void AutoRotation(RopoApi::FloatType Aim_Degree){
        // Chassis Error!!
        // RopoDevice::Chassis.AutoMoveType = RopoDevice::Chassis.Rotate;
        RopoApi::FloatType Current_Degree = RopoDevice::Sensors::Inertial.get_heading();
        RopoApi::FloatType Error_Degree = Aim_Degree - Current_Degree;

        // not fit for additional pid controller
        // if(Error_Degree > 180 || Error_Degree < -180){(Error_Degree > 0) ? (Error_Degree = 360 - Error_Degree) : (Error_Degree += 360);}

        pid.err=Error_Degree;
        RopoApi::FloatType increment=pid.Kp*(pid.err-pid.err_next)+pid.Ki*pid.err+pid.Kd*(pid.err-2*pid.err_next+pid.err_last);
        pid.err_last=pid.err_next;
        pid.err_next=pid.err;
        RopoApi::FloatType Output = increment + Current_Degree;
        Excution_Rotation(Output);

        // pros::lcd::print(1,"Ready!!! %.5f %.5f ", Output*10e3, Current_Degree);
        
        pros::lcd::print(1, "%.5f", Output);
        pros::delay(20);
    }
    void ReSet(){
        pid.err = 0;               
        pid.err_next = 0;          
        pid.err_last = 0;   
        bool ifArrive = false;     
    }
    */

    double Angle = 0;
    double Ball_x = 0;
    double Ball_y = 0;
    bool Auto_Find_DQ(double Distance , double Roation){
        double k = 1.0;
        double Lim_x = 100;
        double Lim_y = 100;
        double Dis = Distance * k;
        double Roa_para = -1;
        double Dis_para = 1;
        double ExternDis = 0.1;
        double X = RopoDevice::GetTransformedPosition()[1];
        double Y = RopoDevice::GetTransformedPosition()[2];
        Distance = Dis_para * Distance;
        Roation = Roa_para * Roation;
        Angle = Roation + RopoDevice::Position_Motor::MyPosition.Get_Angle();//OpenMV获取角度向左是负，转换为右手系需要*-1
        Ball_x = X + RopoMath::Cos(Angle)* (Dis * k + ExternDis);
        Ball_y = Y + RopoMath::Sin(Angle)* (Dis * k + ExternDis);
        if(Ball_x < 1.5 && Ball_x > 0.3 && Ball_y < -0.9 && Ball_y > -1.5 ){
            return 0;//没有越界就是0
        }
        else{
            return 1; 
        }
    }

    void Update_Ball_Position(double Distance , double Roation){
        double k = 1.0;
        double Roa_para = -1;
        double ExternDis = 0.2;
        double X = RopoDevice::GetTransformedPosition()[1];
        double Y = RopoDevice::GetTransformedPosition()[2];
        Angle = Roa_para * Roation + RopoDevice::Position_Motor::MyPosition.Get_Angle();//OpenMV获取角度向左是负，转换为右手系需要*-1
        Ball_x = X + RopoMath::Cos(Angle) * (Distance * k + ExternDis);
        Ball_y = Y + RopoMath::Sin(Angle) * (Distance * k + ExternDis);
    }

    void Auto_Find(){
        RopoDevice::Chassis.StartChassisAutoControll();//底盘MoveType设置为AutoMove
        RopoDevice::Motors::IntakeMotor.move_velocity(-500);
		RopoDevice::ThreeWire::IntakerPneumatic.set_value(true);
        int See_Flag = 0;
        double Distance = 0;
        double Degree = 0;
        double Aim_Position[2] = {0};
        int Go_Flag = 0;
        See_Flag = RopoDevice::Sensors::My_openMV.If_See();
        Distance = RopoDevice::Sensors::My_openMV.Get_Ball_Dis();
        Degree = RopoDevice::Sensors::My_openMV.Get_Ball_Deg();
        while(!( (See_Flag) && !Auto_Find_DQ(Distance,Degree) ) ) {
            RopoDevice::Chassis.MoveVelocity(0,3.0);
            pros::delay(100);
            See_Flag = RopoDevice::Sensors::My_openMV.If_See();
            Distance = RopoDevice::Sensors::My_openMV.Get_Ball_Dis();
            Degree = RopoDevice::Sensors::My_openMV.Get_Ball_Deg();
        }
        Angle = -RopoDevice::Sensors::My_openMV.Get_Ball_Deg() + RopoDevice::Position_Motor::MyPosition.Get_Angle();
        RopoDevice::Chassis.AutoRotateAbs(Angle);
        pros::delay(30);
        for(int ii = 0;ii<=35;ii++){
            if(!RopoDevice::Chassis.IfArrived()){
                Angle = -RopoDevice::Sensors::My_openMV.Get_Ball_Deg() + RopoDevice::Position_Motor::MyPosition.Get_Angle();
                RopoDevice::Chassis.AutoRotateAbs(Angle);
                pros::delay(20);
            }
        }
        //pros::delay(300);
        // while(!RopoDevice::Chassis.IfArrived()){
        //     // Angle = -RopoDevice::Sensors::My_openMV.Get_Ball_Deg() + RopoDevice::Position_Motor::MyPosition.Get_Angle();
        //     // RopoDevice::Chassis.AutoRotateAbs(Angle);
        //     pros::delay(30);
        // }
        Update_Ball_Position(RopoDevice::Sensors::My_openMV.Get_Ball_Dis() , RopoDevice::Sensors::My_openMV.Get_Ball_Deg());
        RopoDevice::Chassis.AutoPositionMove(Ball_x,Ball_y);

    }
}
