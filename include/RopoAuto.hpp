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
        double ExternDis = 0.2;
        double X = RopoDevice::GetTransformedPosition()[1];
        double Y = RopoDevice::GetTransformedPosition()[2];
        Distance = Dis_para * Distance;
        Roation = Roa_para * Roation;
        Angle = Roation + RopoDevice::Position_Motor::MyPosition.Get_Angle();//OpenMV获取角度向左是负，转换为右手系需要*-1
        Ball_x = X + RopoMath::Cos(Angle)* (Dis * k + ExternDis);
        Ball_y = Y + RopoMath::Sin(Angle)* (Dis * k + ExternDis);
        if(Ball_x < 1.5 && Ball_x > 0.3 && Ball_y < -0.95 && Ball_y > -1.9 ){
            return 0;//没有越界就是0
        }
        else{
            return 1; 
        }
    }

    void Update_Ball_Position(double Distance , double Roation){
        double k = 1.0;
        double Roa_para = -1;
        double ExternDis = 0.3;
        double X = RopoDevice::GetTransformedPosition()[1];
        double Y = RopoDevice::GetTransformedPosition()[2];
        Angle = Roa_para * Roation + RopoDevice::Position_Motor::MyPosition.Get_Angle();//OpenMV获取角度向左是负，转换为右手系需要*-1
        Ball_x = X + RopoMath::Cos(Angle) * (Distance * k + ExternDis);
        Ball_y = Y + RopoMath::Sin(Angle) * (Distance * k + ExternDis);
    }

    void Auto_Find(){
        RopoDevice::Chassis.StartChassisAutoControll();//底盘MoveType设置为AutoMove
        RopoDevice::Motors::IntakeMotor.move_velocity(-500);
		RopoDevice::ThreeWire::IntakerPneumatic.set_value(false);
        int See_Flag = 0;
        double Distance = 0;
        double Degree = 0;
        double Aim_Position[2] = {0};
        int Go_Flag = 0;
        See_Flag = RopoDevice::Sensors::My_openMV.If_See();
        Distance = RopoDevice::Sensors::My_openMV.Get_Ball_Dis();
        Degree = RopoDevice::Sensors::My_openMV.Get_Ball_Deg();
        double time0 = pros::millis();
        while(!( (See_Flag) && !Auto_Find_DQ(Distance,Degree) ) && pros::millis() - time0 < 5000) {
            RopoDevice::Chassis.MoveVelocity(0,1.7);
            pros::delay(100);
            See_Flag = RopoDevice::Sensors::My_openMV.If_See();
            Distance = RopoDevice::Sensors::My_openMV.Get_Ball_Dis();
            Degree = RopoDevice::Sensors::My_openMV.Get_Ball_Deg();
        }
        RopoDevice::gpsAddPosition.SetUpdateFlag(10);
        if(pros::millis() - time0 < 5000){
            Angle = -RopoDevice::Sensors::My_openMV.Get_Ball_Deg() + RopoDevice::Position_Motor::MyPosition.Get_Angle();
            RopoDevice::Chassis.MoveVelocity(0,0.0);
            pros::delay(100);
            RopoDevice::Chassis.AutoRotateAbs(Angle);
            pros::delay(30);
            for(int ii = 0;ii<=10;ii++){
                if(!RopoDevice::Chassis.IfArrived()){
                    Angle = -RopoDevice::Sensors::My_openMV.Get_Ball_Deg() + RopoDevice::Position_Motor::MyPosition.Get_Angle();
                    RopoDevice::Chassis.AutoRotateAbs(Angle);
                    pros::delay(100);
                }
            }
            Update_Ball_Position(RopoDevice::Sensors::My_openMV.Get_Ball_Dis() , RopoDevice::Sensors::My_openMV.Get_Ball_Deg());
            RopoDevice::gpsAddPosition.SetUpdateFlag(0);
            RopoDevice::Chassis.MoveVelocity(1.0,0);
            pros::delay(RopoMath::Distance(Ball_x- RopoDevice::GetTransformedPosition()[1],Ball_y- RopoDevice::GetTransformedPosition()[2])*1000);
            RopoDevice::Chassis.MoveVelocity(0.0,0);
            pros::delay(200);
            //RopoDevice::Chassis.AutoPositionMove(Ball_x,Ball_y,10000,3500);
            RopoDevice::gpsAddPosition.SetUpdateFlag(10);
        }
    }
}
