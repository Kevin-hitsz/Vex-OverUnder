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
    struct PID{   
        RopoApi::FloatType err = 0;               
        RopoApi::FloatType err_next = 0;          
        RopoApi::FloatType err_last = 0;        
        RopoApi::FloatType Kp = 0.2 ,Ki = 0.1 ,Kd = 0.03; // 0.15 0.08 0.01 // 0.2 0.1 0.03
        bool ifArrive = false;
    }pid;

    void rotation(RopoApi::FloatType Degree){
        RopoApi::FloatType W_ = (Degree - RopoDevice::Sensors::Inertial.get_heading()) * 1;
        pros::lcd::print(2, "%.5f", W_);
        RopoDevice::Chassis.MoveVelocity(0, -W_);
    }

    void autoRotate(RopoApi::FloatType AimDegree){
        RopoApi::FloatType CurrentDegree = RopoDevice::Sensors::Inertial.get_heading();
        RopoApi::FloatType ErrorDegree = AimDegree - CurrentDegree;

        pid.err=ErrorDegree;
        RopoApi::FloatType increment=pid.Kp*(pid.err-pid.err_next)+pid.Ki*pid.err+pid.Kd*(pid.err-2*pid.err_next+pid.err_last);
        pid.err_last=pid.err_next;
        pid.err_next=pid.err;
        RopoApi::FloatType Output = increment + CurrentDegree;
        rotation(Output);

        pros::lcd::print(1,"Ready!!! %.5f %.5f ", Output*10e3, CurrentDegree);
        
        pros::lcd::print(1, "%.5f", Output);
        pros::delay(20);
    }
}