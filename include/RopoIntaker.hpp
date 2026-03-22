#include "pros/motors.hpp"
#include "RopoMath/Vector.hpp"

#ifndef _ROPOINTAKER_HPP_
#define _ROPOINTAKER_HPP_
namespace RopoIntaker{

    const int IntakingVoltage = 12000;
    const int HidingVoltage = -10000;

    class IntakerModule{
        private:
            pros::MotorGroup *Motors;
            bool now_status;       //true for intakeing false for hiding
            bool target_status;
            bool is_throwing;     
            bool is_resting;       //true for is_resting(voltage = 0), false for rolling(voltage = 12000)
        public:
            
            static void IntakeBackGroundFunction(void *Parameter){
                if(Parameter == nullptr) return;
                IntakerModule *This = static_cast<IntakerModule *>(Parameter);
                while(true){
                    if(This -> target_status == true && This -> is_resting == false){
                        This -> Motors -> move_voltage(IntakingVoltage);
                        This -> now_status = true;
                    } 
                    else if (This -> target_status == true && This -> is_resting == true){
                        This -> Motors -> move_voltage(0);
                        This -> now_status = true;
                    } 
                    else if (This -> now_status == true && This -> target_status == false){
                        This -> Motors -> move_voltage(HidingVoltage );
                        pros::delay(700);
                        This -> now_status = false;
                    } 
                    else if (This -> now_status == false && This -> target_status == false){
                        This -> Motors -> move_voltage(HidingVoltage /4.0);
                    } 
                    pros::delay(30);
                }
            }

            IntakerModule(pros::MotorGroup *Mtrs){
                Motors = Mtrs;
                now_status = true;
                target_status = true;
                is_resting = true;
                is_throwing = false;
                Motors -> set_brake_modes(pros::E_MOTOR_BRAKE_BRAKE);
                new pros::Task(IntakerModule::IntakeBackGroundFunction, this);
            }

            void ChangeIntakeStatus(){
                target_status = !target_status;
            }
            void SetIntakeStatus(bool _target_status){
                target_status = _target_status;
            }
            void ChangeRestMode(){
                is_resting = !is_resting;
            }
            void SetRestMode(bool _is_resting){
                is_resting = _is_resting;
            }
            void SetStayHidingMode(bool Mode){
                is_throwing = Mode;
            }
            bool GetIntakeMode(){
                return now_status;
            }
            bool GetRestingMode(){
                return is_resting;
            }
            bool GetThrowingMode(){
                return is_throwing;
            }
    };
    
}

#endif 
