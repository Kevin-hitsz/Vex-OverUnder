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
        double Lim_x = 9999;
        double Lim_y = 0.1;
        double Dis = Distance * k;
        double Roa_para = -1.0;
        double Dis_para = 1.05;
        double X = RopoDevice::Position_Motor::MyPosition.Get_X();
        double Y = RopoDevice::Position_Motor::MyPosition.Get_Y();
        Distance = Dis_para * Distance;
        Roation = Roa_para * Roation;
        Angle = Roation + RopoDevice::Position_Motor::MyPosition.Get_Angle();//OpenMV获取角度向左是负，转换为右手系需要*-1
        Ball_x = X + RopoMath::Cos(Angle)*Dis;
        Ball_y = Y + RopoMath::Sin(Angle)*Dis;
        if(Ball_x < Lim_x && Ball_y < Lim_y){
            return 0;//没有越界就是0
        }
        else{
            return 1; 
        }
    }

    void Auto_Find()
    {

        int See_Flag = 0;
        double Distance = 0;
        double Degree = 0;
        double Aim_Position[2] = {0};
        int Go_Flag = 0;
        See_Flag = RopoDevice::Sensors::Opmv.If_See();

      
        while(!See_Flag||
            (RopoDevice::Sensors::Opmv.GetBall_Count()>0?
            (Auto_Find_DQ( RopoDevice::Sensors::Opmv.Get_Ball_Dis(), RopoDevice::Sensors::Opmv.Get_Ball_Deg())):1)) 
            {
            RopoDevice::Chassis.MoveVelocity(0,0.4);
            pros::delay(100);
            See_Flag = RopoDevice::Sensors::Opmv.If_See();

            if(RopoDevice::Sensors::Opmv.IsReading())
                pros::lcd::print(2,"hello,  %.2f,%.1f",Distance,Degree);
            else
                pros::lcd::print(2,"not,hello,  %.2f,%.1f",Distance,Degree);

        }
        
        Distance = RopoDevice::Sensors::Opmv.Get_Ball_Dis();
        Degree = RopoDevice::Sensors::Opmv.Get_Ball_Deg();


        if(Distance<0.4)
        {
            if(!Auto_Find_DQ(Distance,Degree*2.5)){
                Go_Flag = 1;
            }
            
            if(Go_Flag){
                RopoDevice::Chassis.AutoRotateAbs(Angle);
                while(RopoDevice::Chassis.IfArrived());
                pros::delay(100);
                Go_Flag = 0;
            }

            Distance = RopoDevice::Sensors::Opmv.Get_Ball_Dis();
            Degree = RopoDevice::Sensors::Opmv.Get_Ball_Deg();

            if(!Auto_Find_DQ(Distance,Degree)){
                Go_Flag = 1;
            }
            
            if(Go_Flag){
                RopoDevice::Chassis.AutoPositionMove(Ball_x,Ball_y,Angle);
                pros::delay(100);
                Go_Flag = 0;
            }

        }
        else
        {
            if(!Auto_Find_DQ(0.5*Distance,Degree)){
                Go_Flag = 1;
            }
            
            if(Go_Flag){
                RopoDevice::Chassis.AutoPositionMove(Ball_x,Ball_y,Angle);
                pros::delay(100);
                Go_Flag = 0;
            }

            Distance = RopoDevice::Sensors::Opmv.Get_Ball_Dis();
            Degree = RopoDevice::Sensors::Opmv.Get_Ball_Deg();

            if(!Auto_Find_DQ(Distance,Degree)){
                Go_Flag = 1;
            }
            

            if(Go_Flag){
                RopoDevice::Chassis.AutoPositionMove(Ball_x,Ball_y,Angle);
                pros::delay(100);
                Go_Flag = 0;
            }
        }
        
    }
}