// Code : UTF - 8
#ifndef ROPO_CHASSIS_HPP
#define ROPO_CHASSIS_HPP

#include "RopoApi.hpp"
#include "RopoControl/Chassis.hpp"
#include "RopoControl/Regulator.hpp"
#include "RopoMath/Vector.hpp"
#include "RopoMath/Misc.hpp"
#include "pros/rtos.hpp"
#include "RopoSensor/Debugger.hpp"

namespace RopoChassis{
	// Api

	typedef RopoApi::FloatType FloatType;

	// Code
	class TankChassis{
		private:
			static constexpr float WheelRad = 0.041275;
			static constexpr float ChassisParameter = (0.285+0.2855)/2;//0.2855
			static constexpr float DefaultVelocityLimits = 600;

			RopoControl::TankChassisCore Core;
			void (*MotorMove[2])(FloatType);
			RopoMath::Vector<FloatType> ChassisVelocity;
			RopoMath::Vector<FloatType> MotorVelocity;
			const int SampleTime;

			// For AutoMove Functions
			RopoMath::Vector<FloatType> (*GetCurPosition)();
			RopoMath::Vector<FloatType> AimPosition;
			FloatType DegErrorTolerance;
			bool Arrived;

			pros::Task* BackgroundTask;

			void OpenLoopMove(const RopoMath::Vector<FloatType>& Velocity) {
				const FloatType ChassisRatio = 7.0 / 5.0;
				const FloatType radTorpm = 600 / 62.83;
				static RopoMath::Vector<RopoApi::FloatType> _Velocity(RopoMath::ColumnVector,2);
				_Velocity = Velocity;
				_Velocity[1] = _Velocity[1] * ChassisRatio * radTorpm;
				_Velocity[2] = _Velocity[2] * ChassisRatio * radTorpm;
				MotorVelocity = Core.Move(_Velocity);
				MotorMove[0](MotorVelocity[1]);
				MotorMove[1](MotorVelocity[2]);
			}
		public:
			enum AutoStatus{
				Disable = -1,
				MovePosAbs = 0,
				MoveForward = 1,
				Rotate = 2
			} AutoMoveType;
			static void ChassisMoveBackgroundFunction(void *Parameter){
				if(Parameter == nullptr)return;
				TankChassis *This = static_cast<TankChassis *>(Parameter);
				AutoStatus LastMoveType = This->AutoMoveType;  
				auto AimPosition = This->GetCurPosition();
				while(true){
					if(This->AutoMoveType == Disable){
						This->OpenLoopMove(This->ChassisVelocity);
					}
					pros::delay(This->SampleTime);
				}
			}

			TankChassis(	void (*RightMotorMove)(FloatType),
							void (*LeftMotorMove )(FloatType),
							RopoMath::Vector<FloatType> (*GetPosition_)(),
							int _SampleTime = 5):
				Core( WheelRad, ChassisParameter, DefaultVelocityLimits),
				MotorMove{  RightMotorMove,LeftMotorMove},
				ChassisVelocity(RopoMath::ColumnVector,2),SampleTime(_SampleTime),
				AutoMoveType(Disable),GetCurPosition(GetPosition_),AimPosition(RopoMath::ColumnVector,3),
				DegErrorTolerance(5),Arrived(false),
				BackgroundTask(nullptr){
				BackgroundTask = new pros::Task(ChassisMoveBackgroundFunction,this);
			}

			void SetAutoMoveType(AutoStatus status) {AutoMoveType = status;}
			void SetVelocityLimits(FloatType VelocityLimits) {Core.SetVelocityLimits(VelocityLimits);}
			void SetDegErrorTolerance(FloatType ErrorTolerance) {DegErrorTolerance = ErrorTolerance;}
			RopoMath::Vector<FloatType> GetChassisVelocity(){return ChassisVelocity;}
			RopoMath::Vector<FloatType> GetMotorVelocity(){return MotorVelocity;}
			bool IfArrived(){return Arrived;}
			void MoveVelocity(const RopoMath::Vector<FloatType>& Velocity) {
				ChassisVelocity = Velocity, AutoMoveType = Disable;
			}
			void MoveVelocity(RopoApi::FloatType X,RopoApi::FloatType W){
				ChassisVelocity[1] = X;
				ChassisVelocity[2] = W;
			}
	};
}

#endif //ROPO_CHASSIS_HPP
