// Code : UTF - 8
#pragma once

#include "RopoApi.hpp"
#include "RopoControl/Chassis.hpp"
#include "RopoControl/Regulator.hpp"
#include "RopoMath/Misc.hpp"
#include "pros/motors.hpp"
#include "RopoSensor/Debugger.hpp"
#include "RopoDiffySwerve.hpp"
#include <algorithm>

namespace RopoChassis{
    class Chassis{
        private:
            Swerve LF,LB,RF,RB;
            static void ChassisControl(void* param){
                if(param == nullptr) return;
                Chassis *This = static_cast<Chassis *>(param);
                while(true){

                }
            }
        public:
            Chassis(Swerve& LF_,Swerve& LB_,Swerve& RF_,Swerve& RB_)
            :LF(LF_),LB(LB_),RF(RF_),RB(RB_){
                LF.Initialize(),LB.Initialize(),RF.Initialize(),RB.Initialize();
                new Task(ChassisControl,this);
            }
    };
}
typedef RopoChassis::Chassis Chassis;