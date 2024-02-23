#include "pros/motors.h"
#include "pros/motors.hpp"
#include "RopoMath/Misc.hpp"
#include "pros/rtos.hpp"
#ifndef _ROPOLIFTER_HPP_
#define _ROPOLIFTER_HPP_

namespace RopoLifter{

    // Params
    const double HoldingPosition = 470.0;   // 510
    const double WaitingPosition = 98.0;
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
            pros::Motor &LeftMotor;
             
            bool ifReady;
            bool breaktag;

        public:
            static void LifterBackGroundFunction(void *Parameter);
            LifterModule(pros::Motor&);                 // 构造函数
            void Hold();
            void Hide();
            void Wait();
            bool IfReady();
            State GetLifterStatus();
            double GetLifterPosition();
    };

    LifterModule::LifterModule(pros::Motor& ml):LeftMotor(ml)
     {
        LifterState = HIDDEN;
        LifterPosition = 0;
        new pros::Task(LifterModule::LifterBackGroundFunction,this);
    }

    void LifterModule::LifterBackGroundFunction(void *Parameter) {
    	
        if(Parameter == nullptr) return;
        
        LifterModule *This = static_cast<LifterModule *>(Parameter);
        
        This->LeftMotor.move_velocity(0);
        
        This -> LeftMotor.set_encoder_units(pros::E_MOTOR_ENCODER_DEGREES);
        
        This ->LeftMotor. tare_position();

        Vector PositionVector(RopoMath::ColumnVector,1) ;

        This -> ifReady = true;
        This -> breaktag = false;
        while(true) {
        	
            PositionVector[1] = This -> LeftMotor.get_position() ;
            
            This -> LifterPosition = PositionVector[1] * LifterRatio;
            
            if (!This->ifReady)
            {
                This -> breaktag = false;
                switch(This -> LifterState){
                    case HIDDEN:
                        pros::lcd::print(1,"hid");
                        This -> LeftMotor.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);

                        
                        This -> LeftMotor.move_absolute(HiddenPosition,0.7*(HiddenPosition-This->LifterPosition));
                        
                        while (fabs(This->LifterPosition-HiddenPosition) > 2.0)
                        {
                            if (This -> breaktag) break;
                            
                            PositionVector[1] = This -> LeftMotor.get_position() ;
                            This -> LifterPosition = PositionVector[1] * LifterRatio;
                            pros::delay(20);
                        }
                        if (!This -> breaktag)
                        {
                            
                            This -> LeftMotor.brake();
                            This->ifReady = true;
                        }
                        break;    
                    case HOLDING:
                        pros::lcd::print(1,"hold");
                        This -> LeftMotor.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);              
                        This -> LeftMotor.move_absolute(HoldingPosition,200);
                        while (fabs(This->LifterPosition-HoldingPosition) > 3.0)
                        {
                            if (This -> breaktag) break;
                            if (fabs(This->LifterPosition-HoldingPosition) < 30.0) 
                            {
                                
                                This -> LeftMotor.move_absolute(HoldingPosition,30);
                            }
                            PositionVector[1] = This -> LeftMotor.get_position() ;
                            This -> LifterPosition = PositionVector[1] * LifterRatio;
                            pros::delay(20);
                        }
                        if (!This -> breaktag)
                        {
                            This -> LeftMotor.brake();
                            This->ifReady = true;
                        }
                        break;
                    case WAITING:
                    pros::lcd::print(1,"wait");
                        This -> LeftMotor.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
                        This -> LeftMotor.move_absolute(WaitingPosition,30);
                        while (fabs(This->LifterPosition-WaitingPosition) > 3.0)
                        {
                            if (This -> breaktag) break;    
                            PositionVector[1] = This -> LeftMotor.get_position() ;
                            This -> LifterPosition = PositionVector[1]  * LifterRatio;
                            pros::delay(20);
                        }
                        if (!This -> breaktag)
                        {
                            This -> LeftMotor.brake();

                            This->ifReady = true;
                        }
                        break;
                }
            }
            pros::delay(Deltatime);
            pros::lcd::print(2,"%f,%f",PositionVector[1]);
        }
    }
    void   LifterModule::Hold()    { LifterState = HOLDING; ifReady = false; breaktag = true;}
    void   LifterModule::Hide()    { LifterState =  HIDDEN; ifReady = false; breaktag = true;}
    void   LifterModule::Wait()    { LifterState = WAITING; ifReady = false; breaktag = true;}
    bool   LifterModule::IfReady() { return ifReady; }
    State  LifterModule::GetLifterStatus()   { return    LifterState; }
    double LifterModule::GetLifterPosition() { return LifterPosition; }
}
#endif //_ROPOLiftER_HPP_