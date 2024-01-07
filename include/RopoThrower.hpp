#include "pros/motors.h"
#include "pros/motors.hpp"
#include "RopoMath/Vector.hpp"
#ifndef _ROPOTHROWER_HPP_
#define _ROPOTHROWER_HPP_

namespace RopoThrower{

    const double ThrownPosition = 0.0;//120.0
    const double WaitingPosition = 325.0;//210.0
    const double HidingPosition = 0.0;
    const double ThrowerRatio = 1.0 / 3.0 ;// /18.0
    const int FullSpeedVoltage = 12000;
    const int Deltatime = 3;

    typedef enum{
        HIDDEN = 0 ,    //when thrower is hidden in the robot
        WAITING = 1,    //when thrower have streched and waiting to shoot
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
        bool is_disable;
        pros::Task* ThrowerTask;
        public:
        static void ThrowerBackGroundFunction(void *Parameter);
        ThrowerModule(pros::MotorGroup *Mtrs);
        ~ThrowerModule();
        void Throw();
        void Hide();
        void Wait();
        bool IfReady();
        State GetThrowerStatus();
        void set_is_disable(bool _is_disable);
        double get_thrower_position();

    };
    ThrowerModule::ThrowerModule(pros::MotorGroup *Mtrs) {
        ThrowerState = HIDDEN;
        ThrowerAimState = HIDE;
        Motors = Mtrs;
        is_disable = true;
        ThrowerPosition = 0;
        ThrowerTask = new pros::Task(ThrowerModule::ThrowerBackGroundFunction,this);
    }

    ThrowerModule::~ThrowerModule() {
        delete ThrowerTask;
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
        	if (This -> is_disable == false) {
                PositionVector = This -> Motors -> get_positions() ;
                This -> ThrowerPosition = 0;
                for(double i : PositionVector) This -> ThrowerPosition += i;
                This -> ThrowerPosition = This -> ThrowerPosition/ 2.0 * ThrowerRatio;
                while(This -> ThrowerPosition >  360)This -> ThrowerPosition -= 360;
                while(This -> ThrowerPosition <= 0)This -> ThrowerPosition += 360;
                pros::delay(10);
                switch(This -> ThrowerState){
                    case HIDDEN:
                        if(This -> ThrowerAimState == HIDE) {
                            This -> Motors -> move_voltage(-FullSpeedVoltage*0.8);
                            This -> ifReady = true;
                        }
                        else if(This -> ThrowerAimState == WAIT || This -> ThrowerAimState == THROW) {
                            This -> Motors -> move_voltage(FullSpeedVoltage/2.0);
                            pros::delay(500);
                            This -> ThrowerState = WAITING;
                        }
                        break;
                    case WAITING:
                        if(This -> ThrowerAimState == HIDE) {
                            This -> Motors -> move_voltage(-FullSpeedVoltage*0.8);
                            This -> ifReady = false;
                            pros::delay(1100);
                            This -> ThrowerState = HIDDEN;
                        } 
                        else if( This -> ThrowerAimState == WAIT) {
                            if ( WaitingPosition - This -> ThrowerPosition < 18 && WaitingPosition - This -> ThrowerPosition > 0 ) {
                                //This -> Motors -> set_brake_modes(pros::E_MOTOR_BRAKE_HOLD);
                                This -> Motors -> brake();
                                This -> Motors -> move_voltage(0);
                                This -> ifReady = true;
                            }
                            else if(This -> ThrowerPosition > WaitingPosition){
                                This -> Motors -> move_voltage(fmin(fmax( ( -(This->ThrowerPosition - 360) + WaitingPosition) / 60 , 0.75),1.0)*FullSpeedVoltage);
                                This -> ifReady = false;                                                                                //160
                            }
                            else if(This -> ThrowerPosition < WaitingPosition){
                                This -> Motors -> move_voltage(fmin(fmax( ( WaitingPosition - This->ThrowerPosition) / 60 , 0.75),1.0)*FullSpeedVoltage);
                                This -> ifReady = false;
                            }
                            
                        } 
                        else if(This -> ThrowerAimState == THROW) {
                            This -> Motors -> move_voltage(FullSpeedVoltage+2000);
                            This -> ifReady = false;
                            pros::delay(400);
                            This -> ifReady = false;
                        }
                        break;
                }
            } else {
                pros::delay(Deltatime);
            }
            pros::delay(Deltatime);
        }
    }
    void ThrowerModule::Throw()   { 
        ThrowerAimState = THROW;
        is_disable = false;
    }
    void ThrowerModule::Hide()    { 
        ThrowerAimState = HIDE; 
        is_disable = false;
    }
    void ThrowerModule::Wait()    { 
        ThrowerAimState = WAIT; 
        is_disable = false;
    }
    double ThrowerModule::get_thrower_position() { return ThrownPosition;}
    bool ThrowerModule::IfReady() { return ifReady;}
    State ThrowerModule::GetThrowerStatus() { return ThrowerState;}
    void ThrowerModule::set_is_disable(bool _is_disable) { is_disable = _is_disable; }
}
#endif //_ROPOTHROWER_HPP_