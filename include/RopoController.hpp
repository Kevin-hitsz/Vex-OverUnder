// Code : UTF - 8
#ifndef ROPO_CONTROLLER_HPP
#define ROPO_CONTROLLER_HPP

#include "pros/llemu.hpp"
#include "pros/misc.hpp"
#include "pros/rtos.hpp"
#include <vector>
#include <functional>
#include <cmath>

namespace RopoController{

	typedef enum {
		Linear = 0,
		Exp = 1,
		Ln = 2
	} AxisCastType;

	class AxisValueCast{
		private:
			pros::Controller &MyController;
			pros::controller_analog_e_t Axis;
			AxisCastType CastMode;
		public:
			AxisValueCast(pros::Controller &_MyController, pros::controller_analog_e_t _Axis, AxisCastType _CastMode):
				MyController(_MyController),Axis(_Axis),CastMode(_CastMode){}
			float GetAxisValue(){
				float AxisValue = MyController.get_analog(Axis);
				AxisValue /= 127.0;
				if(CastMode == Linear) // Axis to Axis
					return AxisValue;
				else if (CastMode == Exp) { // Axis to e^(Axis)
					if(AxisValue > 0)
						AxisValue = (exp(AxisValue) - 1) / (exp(1)-1) ;
					else 
						AxisValue = - ((exp(-AxisValue) - 1) / (exp(1)-1) );
					return AxisValue;
				}
				else if(CastMode == Ln){ // Axis to ln(Axis)
					if(AxisValue > 0) AxisValue = log(AxisValue+1) / log(2);
					else AxisValue = - log(-AxisValue+1) / log(2);
					return AxisValue;
				}   
		        return AxisValue;
			}
    };

	typedef enum {
		Rising = 0,
		Falling = 1,
		DoubleEdge = 2,
		Pressing = 3,
		DoubleClick = 4
	} ButtonTriggerType;

	class ButtonTaskCore{
		private:
			pros::controller_digital_e_t MyButton;
			ButtonTriggerType TriggerType;
			void *Callback;
			bool CallbackType;
			void *Parameter;
			int PressTime;
			pros::Task *BackgroundTask;
			bool Flag,DoubleClickFlag;
			int BeginTime;

			void Trigger(){
				if(!CallbackType){
					delete BackgroundTask;
					BackgroundTask = new pros::Task((void (*)(void))Callback);
					// ((void (*)(void))Callback)();
				}
				else {
					delete BackgroundTask;
					BackgroundTask = new pros::Task((void (*)(void *))Callback,Parameter);
					// ((void (*)(void *))Callback)(Parameter);
				}
			}
		public:
			ButtonTaskCore(pros::controller_digital_e_t _Button, ButtonTriggerType _TriggerType, void (*_Callback)(void), int _PressTime):
				MyButton(_Button),TriggerType(_TriggerType),Callback(nullptr),CallbackType(false),Parameter(nullptr),
				PressTime(_PressTime),BackgroundTask(nullptr),Flag(false),DoubleClickFlag(false),BeginTime(-1){
				Callback = (void *)_Callback;
			}
			ButtonTaskCore(pros::controller_digital_e_t _Button, ButtonTriggerType _TriggerType, void (*_Callback)(void *), void *_Parameter, int _PressTime):
				MyButton(_Button),TriggerType(_TriggerType),Callback(nullptr),CallbackType(true),Parameter(_Parameter),
				PressTime(_PressTime),BackgroundTask(nullptr),Flag(false),DoubleClickFlag(false),BeginTime(-1){
				Callback = (void *)_Callback;
			}
			~ButtonTaskCore(){delete BackgroundTask;}
			pros::controller_digital_e_t Button(){
				return MyButton;
			}
			void Update(bool ButtonStatus){
				if( TriggerType == Rising ){
					if(Flag == false && ButtonStatus == true){
						Flag = true;
						Trigger();
					}
					else if(ButtonStatus == false) 
						Flag = false;
				}
				else if( TriggerType == Falling ){
					// pros::lcd::print(5,"%s %d %d","F",Flag?1:0,BeginTime);
					if(Flag == false && ButtonStatus == false){
						Flag = true;
						Trigger();
					}
					else if(ButtonStatus == true) 
						Flag = false;
				}
				else if( TriggerType == DoubleEdge ){
					// pros::lcd::print(5,"%s %d %d","DE",Flag?1:0,BeginTime);
					if(Flag == false && ButtonStatus == true){
						Flag = true;
						Trigger();
					}
					else if(Flag == true && ButtonStatus == false) {
						Flag = false;
						Trigger();
					}
				}
				else if( TriggerType == Pressing ){
					// pros::lcd::print(5,"%s %d %d","P",Flag?1:0,BeginTime);
					if(Flag == false && ButtonStatus == true)
						Flag = true, BeginTime = pros::millis();
					else if(ButtonStatus == false) 
						Flag = false, BeginTime = -1;
					else if(BeginTime != -1 && ButtonStatus == true && pros::millis() > BeginTime + PressTime){
						Trigger();
						BeginTime = -1;
					}
				}
				else if( TriggerType == DoubleClick ){
					// pros::lcd::print(5,"%s %d %d","DC",Flag?1:0,BeginTime);
					if(BeginTime + 200 < pros::millis())
						BeginTime = -1, DoubleClickFlag = false;
					if(Flag == false && DoubleClickFlag == false && ButtonStatus == true)
						BeginTime = pros::millis(),	Flag = true;
					else if(Flag == true && DoubleClickFlag == false && BeginTime != -1 && ButtonStatus == false)
						DoubleClickFlag = true;
					else if(Flag == true && DoubleClickFlag == true && BeginTime != -1 && ButtonStatus == true){
						Trigger();
						BeginTime = -1;
					}
					else if(BeginTime == -1) Flag = false;
				}
			}
	};

	class ButtonTaskLine{
		private:
			pros::Controller &MyController;
			std::vector<ButtonTaskCore> ButtonQueue;
			pros::Task *BackgroundTask;
			static void BackgroundTaskFunction(void *Parameter){
				if(Parameter == nullptr)return;
				ButtonTaskLine *This = static_cast<ButtonTaskLine *>(Parameter);
				while(true){
					for(ButtonTaskCore &ButtonTask : This->ButtonQueue){
						bool ButtonStatus = This->MyController.get_digital(ButtonTask.Button());
						ButtonTask.Update(ButtonStatus);
					}
					pros::delay(20);
				}
			}
		public:
			ButtonTaskLine(pros::Controller &_MyController):MyController(_MyController){
				BackgroundTask = nullptr;
			}
			~ButtonTaskLine(){
				delete BackgroundTask;
				ButtonQueue.clear();
			}
			void Enable(){
				BackgroundTask = new pros::Task(BackgroundTaskFunction,this);
			}
			void AddButtonDetect(pros::controller_digital_e_t Button, ButtonTriggerType TriggerType, void (*Callback)(void), int PressTime = 1000){
				ButtonTaskCore *temp = new ButtonTaskCore(Button,TriggerType,Callback,PressTime);
				ButtonQueue.push_back(*temp);
			}
			void AddButtonDetect(pros::controller_digital_e_t Button, ButtonTriggerType TriggerType, void (*Callback)(void *), void *Parameter, int PressTime = 1000){
				ButtonTaskCore *temp = new ButtonTaskCore(Button,TriggerType,Callback,Parameter,PressTime);
				ButtonQueue.push_back(*temp);
			}
	};
}

#endif //ROPO_CONTROLLER_HPP
