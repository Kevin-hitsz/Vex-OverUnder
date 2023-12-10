# pragma once

#include "RopoApi.hpp"
#include "RopoControl/Regulator.hpp"
#include "RopoDevice.hpp"
#include "api.h"
#include "RopoMath/Misc.hpp"
#include "pros/rtos.hpp"
#include <cmath>
#include <iostream>

namespace RopoAuto{
    // struct PID{   
    //     RopoApi::FloatType err = 0;               
    //     RopoApi::FloatType err_next = 0;          
    //     RopoApi::FloatType err_last = 0;        
    //     RopoApi::FloatType Kp = 0.15 ,Ki = 0.08 ,Kd = 0.01;   
    //     RopoApi::FloatType deltaError = 1.0;
    //     bool ifArrive = false;
    // }pid;

    // void Excution_Rotation(RopoApi::FloatType Degree){
    //     RopoApi::FloatType W_ = (Degree - RopoDevice::Sensors::Inertial.get_heading()) * 1; // / 180 * RopoMath::Pi / 2;
    //     pros::lcd::print(2, "%.5f", W_);
    //     RopoDevice::Chassis.MoveVelocity(0, -W_);
    // }
    // void AutoRotation(RopoApi::FloatType Aim_Degree){
    //     // Chassis Error!!
    //     // RopoDevice::Chassis.AutoMoveType = RopoDevice::Chassis.Rotate;
    //     RopoApi::FloatType Current_Degree = RopoDevice::Sensors::Inertial.get_heading();
    //     RopoApi::FloatType Error_Degree = Aim_Degree - Current_Degree;

    //     // not fit for additional pid controller
    //     // if(Error_Degree > 180 || Error_Degree < -180){(Error_Degree > 0) ? (Error_Degree = 360 - Error_Degree) : (Error_Degree += 360);}

    //     pid.err=Error_Degree;
    //     RopoApi::FloatType increment=pid.Kp*(pid.err-pid.err_next)+pid.Ki*pid.err+pid.Kd*(pid.err-2*pid.err_next+pid.err_last);
    //     pid.err_last=pid.err_next;
    //     pid.err_next=pid.err;
    //     RopoApi::FloatType Output = increment + Current_Degree;
    //     Excution_Rotation(Output);

    //     // pros::lcd::print(1,"Ready!!! %.5f %.5f ", Output*10e3, Current_Degree);
        
    //     pros::lcd::print(1, "%.5f", Output);
    //     pros::delay(20);
    // }
    // void ReSet(){
    //     pid.err = 0;               
    //     pid.err_next = 0;          
    //     pid.err_last = 0;   
    //     bool ifArrive = false;     
    // }
}