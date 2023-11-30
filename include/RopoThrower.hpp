#include "pros/motors.h"
#include "pros/motors.hpp"
#include "RopoMath/Vector.hpp"
#ifndef _ROPOTHROWER_HPP_
#define _ROPOTHROWER_HPP_

namespace RopoThrower{

    const double ThrownPosition = 0.0;//120.0
    const double WaitingPosition = 280.0;//210.0
    const double HidingPosition = 0.0;
    const double ThrowerRatio = 1.0;  //  /3.0 // /18.0
    const int FullSpeedVoltage = 6000;
    const int Deltatime = 3;

    typedef enum{
        HIDDEN = 0,      //when thrower is hidden in the robot
        WAITING = 1,    //when thrower have streched and waiting to shoot
        //THROWING = 2      //when thrower haven't been hiden
    }State;

    typedef enum{
        HIDE = 0,
        THROW = 1,
        WAIT = 2
    }AimState;

    class ThrowerModule{
        private:
        double ThrowerPosition;
        State ThrowerState;
        AimState ThrowerAimState;
        pros::MotorGroup *Motors; 
        bool ifReady;
        public:
        static void ThrowerBackGroundFunction(void *Parameter);
        ThrowerModule(pros::MotorGroup *Mtrs);
        void Throw();
        void Hide();
        void Wait();
        bool IfReady();
        State GetThrowerStatus();

    };
    ThrowerModule::ThrowerModule(pros::MotorGroup *Mtrs) {
        ThrowerState = HIDDEN;
        ThrowerAimState = HIDE;
        Motors = Mtrs;
        ThrowerPosition = 0;
        new pros::Task(ThrowerModule::ThrowerBackGroundFunction,this);
    }
    void ThrowerModule::ThrowerBackGroundFunction(void *Parameter) {
    	
        if(Parameter == nullptr)return;
        
        ThrowerModule *This = static_cast<ThrowerModule *>(Parameter);
        // This -> Motors -> move_velocity(-FullSpeedVoltage);
        // pros::delay(1500);
        This -> Motors -> move_velocity(0);
        This -> Motors -> set_encoder_units(pros::E_MOTOR_ENCODER_DEGREES);
        This -> Motors -> tare_position();
        std::vector<double> PositionVector ;
        This -> ifReady = false;
        while(true) {
        	
            PositionVector = This -> Motors -> get_positions() ;
			This -> ThrowerPosition = 0;
            for(double i : PositionVector) This -> ThrowerPosition += i;
            This -> ThrowerPosition = This -> ThrowerPosition/ 2.0 * ThrowerRatio;
            while(This -> ThrowerPosition >=  360)This -> ThrowerPosition -= 360;
            while(This -> ThrowerPosition <= 0)This -> ThrowerPosition += 360;

            switch(This -> ThrowerState){
                case HIDDEN:
	                if(This -> ThrowerAimState == HIDE) {
                        This -> Motors -> move_voltage(0);
                        This -> ifReady = true;
                    }
	                else if(This -> ThrowerAimState == WAIT || This -> ThrowerAimState == THROW) {
	                    // if(This -> ThrowerPosition < WaitingPosition){
	                    //     This -> Motors -> move_voltage(FullSpeedVoltage);
	                    // }
	                    This -> ThrowerState = WAITING;
	                }
					break;
                case WAITING:
	                if(This -> ThrowerAimState == HIDE) {
                        This -> Motors -> move_voltage(-FullSpeedVoltage*1.2);
                        This -> ifReady = false;
                        pros::delay(1000);
                        This -> ThrowerState = HIDDEN;
	                } 
                    else if( This -> ThrowerAimState == WAIT) {
                        if ( WaitingPosition - This -> ThrowerPosition < 20 && WaitingPosition - This -> ThrowerPosition > 0 ) {
                            This -> Motors -> brake();
                            This -> ifReady = true;
                        }
                        else if(This -> ThrowerPosition > WaitingPosition){
	                        This -> Motors -> move_voltage(fmin(fmax( ( -(This->ThrowerPosition - 360) + WaitingPosition) / 160 , 0.4),1.0)*FullSpeedVoltage);
                            This -> ifReady = false;
                        }
                        else if(This -> ThrowerPosition < WaitingPosition){
	                        This -> Motors -> move_voltage(fmin(fmax( ( WaitingPosition - This->ThrowerPosition) / 160 , 0.4),1.0)*FullSpeedVoltage);
                            This -> ifReady = false;
                        }
	                    
	                } 
                    else if(This -> ThrowerAimState == THROW) {
	                    This -> Motors -> move_voltage(FullSpeedVoltage);
                        This -> ifReady = false;
                        pros::delay(400);
                        This -> ifReady = false;
	                }
                	break;
            }
            pros::delay(Deltatime);
        }
    }
    void ThrowerModule::Throw()   { ThrowerAimState = THROW; }
    void ThrowerModule::Hide()    { ThrowerAimState = HIDE; }
    void ThrowerModule::Wait()    { ThrowerAimState = WAIT; }
    bool ThrowerModule::IfReady() { return ifReady;}
    State ThrowerModule::GetThrowerStatus() { return ThrowerState;}
}
#endif //_ROPOTHROWER_HPP_