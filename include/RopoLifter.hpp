#include "pros/motors.h"
#include "pros/motors.hpp"
#include "RopoMath/Misc.hpp"
#include "pros/rtos.hpp"
#ifndef _ROPOLIFTER_HPP_
#define _ROPOLIFTER_HPP_

namespace RopoLifter{

    // Params
    const double HoldingPosition = 105.0;//120.0
    const double WaitingPosition = 93.0;//210.0
    const double HiddenPosition = 0.0;
    const double LifterRatio = 1.0;  
    const int FullSpeedVoltage = 6000;
    const int Deltatime = 20;

    typedef enum{
        HIDDEN = 0,         //when lifter is hidden in the robot
        HOLDING = 1,        //when lifter is holding triball
        WAITING = 2         //when lifter is tending to catch
    }State;

    // Class
    class LifterModule{
        private:
            double LifterPosition;
            State LifterState;
            pros::MotorGroup *Motors; 
            bool ifReady;

        public:
            static void LifterBackGroundFunction(void *Parameter);
            LifterModule(pros::MotorGroup *Mtrs);                 // 构造函数
            void Hold();
            void Hide();
            void Wait();
            bool IfReady();
            State GetLifterStatus();
            double GetLifterPosition();
    };

    LifterModule::LifterModule(pros::MotorGroup *Mtrs) {
        LifterState = HIDDEN;
        Motors = Mtrs;
        LifterPosition = 0;
        new pros::Task(LifterModule::LifterBackGroundFunction,this);
    }

    void LifterModule::LifterBackGroundFunction(void *Parameter) {
    	
        if(Parameter == nullptr) return;
        
        LifterModule *This = static_cast<LifterModule *>(Parameter);
        // This -> Motors -> move_velocity(-FullSpeedVoltage);
        // pros::delay(1500);
        This -> Motors -> move_velocity(0);
        This -> Motors -> set_encoder_units(pros::E_MOTOR_ENCODER_DEGREES);
        This -> Motors -> tare_position();
        std::vector<double> PositionVector;

        This -> ifReady = true;
        while(true) {
        	
            PositionVector = This -> Motors -> get_positions() ;
            This -> LifterPosition = (PositionVector[0] + PositionVector[1])/ 2.0 * LifterRatio;
            
            if (!This->ifReady)
            {
                switch(This -> LifterState){
                    case HIDDEN:
                        This->Motors->set_brake_modes(pros::E_MOTOR_BRAKE_HOLD);
                        This->Motors->move_absolute(HiddenPosition,0.7*(HiddenPosition-This->LifterPosition));
                        while (fabs(This->LifterPosition-HiddenPosition) > 2.0)
                        {
                            
                            PositionVector = This -> Motors -> get_positions() ;
                            This -> LifterPosition = (PositionVector[0] + PositionVector[1])/ 2.0 * LifterRatio;
                            pros::delay(20);
                        }
                        // pros::delay(1000);
                        This -> Motors-> brake();
                        This->ifReady = true;
                        break;    
                    case HOLDING:
                        This->Motors->set_brake_modes(pros::E_MOTOR_BRAKE_HOLD);
                        This->Motors->move_absolute(HoldingPosition,65);
                        while (fabs(This->LifterPosition-HoldingPosition) > 3.0)
                        {
                            if (fabs(This->LifterPosition-HoldingPosition) < 30.0) 
                            {
                                This->Motors->move_absolute(HoldingPosition,30);
                            }
                            PositionVector = This -> Motors -> get_positions() ;
                            This -> LifterPosition = (PositionVector[0] + PositionVector[1])/ 2.0 * LifterRatio;
                            pros::delay(20);
                        }
                        This -> Motors -> brake();
                        This->ifReady = true;
                        break;
                    case WAITING:
                        This->Motors->set_brake_modes(pros::E_MOTOR_BRAKE_BRAKE);
                        This->Motors->move_absolute(WaitingPosition,30);
                        while (fabs(This->LifterPosition-WaitingPosition) > 3.0)
                        {
                            
                            PositionVector = This -> Motors -> get_positions() ;
                            This -> LifterPosition = (PositionVector[0] + PositionVector[1])/ 2.0 * LifterRatio;
                            pros::delay(20);
                        }
                        This -> Motors -> brake();
                        This->ifReady = true;
                        break;
                }
            }
            
            pros::delay(Deltatime);
        }
    }
    void   LifterModule::Hold()    { LifterState = HOLDING; ifReady = false; }
    void   LifterModule::Hide()    { LifterState =  HIDDEN; ifReady = false; }
    void   LifterModule::Wait()    { LifterState = WAITING; ifReady = false; }
    bool   LifterModule::IfReady() { return ifReady; }
    State  LifterModule::GetLifterStatus()   { return    LifterState; }
    double LifterModule::GetLifterPosition() { return LifterPosition; }
}
#endif //_ROPOLiftER_HPP_