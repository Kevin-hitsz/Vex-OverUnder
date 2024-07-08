#include <vector>
#include "RopoApi.hpp"
#include "pros/motors.hpp"
#include "pros/adi.hpp"

namespace RopoIntaker{
    class Intaker{
        private:
            std::vector<pros::Motor*> DriveMotors;
            pros::ADIDigitalOut Trigger;
            bool GrabFlag = false;
            enum IntakerState{
                STOP = 0,
                FOR  = 1,
                BACK = 2,
            }IntakerFlag = FOR;
            void VoltageDriveMotors(float voltage){
                for(auto motor : DriveMotors){
                    motor->move_voltage(voltage);
                }
            };
            void VelocityDriveMotors(float velocity){
                for(auto& motor : DriveMotors){
                    motor->move_velocity(velocity);
                }
            };

        public:
            Intaker(const std::vector<pros::Motor*>& motors,const pros::ADIDigitalOut& trigger) 
            :DriveMotors(motors), Trigger(trigger){}
            void SwitchIntakerFor(){
		        IntakerFlag = FOR;
		        VoltageDriveMotors(-10000);
	        }
            void SwitchIntakerBack(){
		        IntakerFlag = BACK;
		        VoltageDriveMotors(10000);
	        }
            void SwitchIntakerStop(){
                IntakerFlag = STOP;
                VoltageDriveMotors(0);
            }
            void SwitchIntakerSwitch(){
                if(IntakerFlag == FOR || IntakerFlag == BACK){
                    SwitchIntakerStop();
                }else if(IntakerFlag == STOP){
                    SwitchIntakerFor();
                }
            }
            void SwitchIntakerChange(){
                if(IntakerFlag == BACK){
                    SwitchIntakerFor();
                }else if(IntakerFlag == FOR){
                    SwitchIntakerBack();
                }
            }
            void SwitchIntakerGrabFor(){
                Trigger.set_value(1);
            }
            void SwitchIntakerGrabBack(){
                Trigger.set_value(0);
            }
    };
}
