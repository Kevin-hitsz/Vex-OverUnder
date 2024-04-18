#include "pros/motors.h"
#include "pros/motors.hpp"
#include "RopoMath/Misc.hpp"
#include "pros/rtos.hpp"
#ifndef _ROPOLIFTER_HPP_
#define _ROPOLIFTER_HPP_

namespace RopoLifter{

    // Params
    const double LifterRatio = 1.0 / 1.0;   // 减速比(电机转三圈，举升杆转一圈)

    const double HoldingPosition = 145.0 * LifterRatio;
    const double TouchingPosition = 100.0 * LifterRatio;
    const double HiddenPosition = 0.0 * LifterRatio;
    
    const int FullSpeedVoltage = 6000;
    const int Deltatime = 20;

    typedef enum{
        HIDDEN = 0,         //when lifter is hidden in the robot
        HOLDING = 1,        //when lifter is holding triball
        TOUCHING = 2         //when lifter is tending to touch the bar
    }State;

    // Class
    class LifterModule{
        private:
            double LifterPosition;
            State LifterState;
            pros::Motor &LeftLiftMotor;
            pros::Motor &RightLiftMotor;
             
            bool ifReady;
            bool breaktag;

        public:
            static void LifterBackGroundFunction(void *Parameter);
            LifterModule(pros::Motor&, pros::Motor&);                 // 构造函数
            void Hold();
            void Hide();
            void Touch();
            bool IfReady();
            State GetLifterStatus();
            double GetLifterPosition();
    };

    LifterModule::LifterModule(pros::Motor& left, pros::Motor& right):LeftLiftMotor(left),RightLiftMotor(right)
     {
        LifterState = HIDDEN;
        LifterPosition = 0;
        ifReady = true;
        breaktag = false;
        new pros::Task(LifterModule::LifterBackGroundFunction,this);
    }

    void LifterModule::LifterBackGroundFunction(void *Parameter) {
    	
        if(Parameter == nullptr) return;
        
        LifterModule *This = static_cast<LifterModule *>(Parameter);
        
        This->LeftLiftMotor.move_velocity(0);
        This->RightLiftMotor.move_velocity(0);
        
        This -> LeftLiftMotor.set_encoder_units(pros::E_MOTOR_ENCODER_DEGREES);
        This -> RightLiftMotor.set_encoder_units(pros::E_MOTOR_ENCODER_DEGREES);
        
        This ->LeftLiftMotor. tare_position();
        This ->RightLiftMotor.tare_position();

        Vector PositionVector(RopoMath::ColumnVector,2) ;

        This -> ifReady = true;
        This -> breaktag = false;
        while(true) {
        	
            PositionVector[1] = This -> LeftLiftMotor.get_position() ;
            PositionVector[2] = This -> RightLiftMotor.get_position() ;
            
            This -> LifterPosition = PositionVector[1] + PositionVector[2] / 2.0;
            
            if (!This->ifReady)
            {
                This -> breaktag = false;
                switch(This -> LifterState){
                    case HIDDEN:
                        //pros::lcd::print(1,"hid");
                        This -> LeftLiftMotor.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
                        This -> RightLiftMotor.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
                        
                        This -> LeftLiftMotor.move_absolute(HiddenPosition,0.5*(HiddenPosition-This->LifterPosition));
                        This -> RightLiftMotor.move_absolute(HiddenPosition,0.5*(HiddenPosition-This->LifterPosition));
                        
                        while (fabs(This->LifterPosition-HiddenPosition) > 2.0)
                        {
                            if (This -> breaktag) break;
                            
                            PositionVector[1] = This -> LeftLiftMotor.get_position() ;
                            PositionVector[2] = This -> RightLiftMotor.get_position() ;
                            This -> LifterPosition = PositionVector[1] + PositionVector[2] / 2.0;
                            pros::delay(20);
                        }
                        if (!This -> breaktag)
                        {
                            
                            This -> LeftLiftMotor.brake();
                            This -> RightLiftMotor.brake();
                            This->ifReady = true;
                        }
                        break;    
                    case HOLDING:
                        //pros::lcd::print(1,"hold");
                        This -> LeftLiftMotor.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
                        This -> RightLiftMotor.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);              
                        This -> LeftLiftMotor.move_absolute(HoldingPosition,0.7*(HoldingPosition-This->LifterPosition));
                        This -> RightLiftMotor.move_absolute(HoldingPosition,0.7*(HoldingPosition-This->LifterPosition));
                        while (fabs(This->LifterPosition-HoldingPosition) > 3.0)
                        {
                            if (This -> breaktag) break;
                            // if (fabs(This->LifterPosition-HoldingPosition) < 30.0) 
                            // {
                                
                            //     This -> LeftLiftMotor.move_absolute(HoldingPosition,30);
                            //     This -> RightLiftMotor.move_absolute(HoldingPosition,30);
                            // }
                            PositionVector[1] = This -> LeftLiftMotor.get_position() ;
                            PositionVector[2] = This -> RightLiftMotor.get_position() ;
                            This -> LifterPosition = PositionVector[1] + PositionVector[2] / 2.0;
                            pros::delay(20);
                        }
                        if (!This -> breaktag)
                        {
                            // This -> LeftLiftMotor.brake();
                            // This -> RightLiftMotor.brake();
                            This->ifReady = true;
                        }
                        break;
                    case TOUCHING:
                        //pros::lcd::print(1,"wait");
                        This -> LeftLiftMotor.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
                        This -> RightLiftMotor.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
                        This -> LeftLiftMotor.move_absolute(TouchingPosition,100);
                        This -> RightLiftMotor.move_absolute(TouchingPosition,100);
                        while (fabs(This->LifterPosition-TouchingPosition) > 3.0)
                        {
                            if (This -> breaktag) break;    
                            PositionVector[1] = This -> LeftLiftMotor.get_position() ;
                            PositionVector[2] = This -> RightLiftMotor.get_position() ;
                            This -> LifterPosition = PositionVector[1] + PositionVector[2] / 2.0;
                            pros::delay(20);
                        }
                        if (!This -> breaktag)
                        {
                            This -> LeftLiftMotor.brake();
                            This -> RightLiftMotor.brake();
                            This->ifReady = true;
                        }
                        break;
                }
            }
            pros::delay(Deltatime);
            //pros::lcd::print(2,"%f,%f",PositionVector[1]);
        }
    }
    void   LifterModule::Hold()    { LifterState = HOLDING; ifReady = false; breaktag = true;}
    void   LifterModule::Hide()    { LifterState =  HIDDEN; ifReady = false; breaktag = true;}
    void   LifterModule::Touch()    { LifterState = TOUCHING; ifReady = false; breaktag = true;}
    bool   LifterModule::IfReady() { return ifReady; }
    State  LifterModule::GetLifterStatus()   { return    LifterState; }
    double LifterModule::GetLifterPosition() { return LifterPosition; }
}
#endif //_ROPOLiftER_HPP_