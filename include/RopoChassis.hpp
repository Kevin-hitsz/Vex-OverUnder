// Code : UTF - 8
#pragma once

#include "RopoApi.hpp"
#include "RopoControl/Chassis.hpp"
#include "RopoControl/Regulator.hpp"
#include "RopoMath/Vector.hpp"
#include "RopoMath/Misc.hpp"
#include "pros/rtos.hpp"
#include <algorithm>


namespace RopoChassis{
	// Code
	class TankChassis{
		private:
			static constexpr float WheelRad = 0.041275;
			static constexpr float ChassisParameter = (0.295+0.295)/2; // 0.2855
			static constexpr float DefaultVelocityLimits = 600;

			inline static RopoControl::PIDRegulator DistanceRegulator{0.004,0.0004,0.00000,0.0004,-1e7,0.15,0.3};
			inline static RopoControl::PIDRegulator FastDegRegulator{0.000036,0.00001,0.00001,0.0015,-1e7,2,0.3};
			inline static RopoControl::PIDRegulator SlowDegRegulator{0.000036,0.00001,0.00001,0.0015,-1e7,2,0.3};
			//0.00001,0.00012,0.0004
			RopoControl::TankChassisCore Core;
			void (*MotorMove[2])(FloatType);
			Vector ChassisVelocity;
			Vector MotorVelocity;
			const int SampleTime;
			// Attention the Reverse fuction from Core
			// int DetectionX;

			// For AutoMove Functions
			enum AutoStatus{
				Disable = -1,
				MovePosAbs = 0,
				MoveForward = 1,
				Rotate = 2
			} AutoMoveType;
			Vector (*GetCurPosition)();
			Vector AimPosition;
			FloatType DegErrorTolerance;
			bool Arrived, DistantArrived, DegreeArrived;

			pros::Task* BackgroundTask;

			void OpenLoopMove(const Vector& Velocity) {
				const FloatType ChassisRatio = 3.0 / 2.0;
				const FloatType radTorpm = 600 / 62.83;				// 1 degree / 2pi * 60s
				static Vector _Velocity(RopoMath::ColumnVector,2);
				_Velocity = Velocity;
				_Velocity[1] = _Velocity[1] * ChassisRatio * radTorpm;
				_Velocity[2] = _Velocity[2] * ChassisRatio * radTorpm;
				MotorVelocity = Core.Move(_Velocity);
				MotorMove[0](MotorVelocity[1]);
				MotorMove[1](MotorVelocity[2]);
			}

			

		public:

			static inline Matrix RotationMatrix(FloatType Degree){
				static Matrix _RotationMatrix(3, 3);
				_RotationMatrix[1][1] = RopoMath::Cos(Degree), _RotationMatrix[1][2] = -RopoMath::Sin(Degree), _RotationMatrix[1][3] = 0;
				_RotationMatrix[2][1] = RopoMath::Sin(Degree), _RotationMatrix[2][2] =  RopoMath::Cos(Degree), _RotationMatrix[2][3] = 0;
				_RotationMatrix[3][1] = 0, _RotationMatrix[3][2] = 0, _RotationMatrix[3][3] = 1;
				return _RotationMatrix;
			}

			static void ChassisMoveBackgroundFunction(void *Parameter){
				if(Parameter == nullptr)return;
				TankChassis *This = static_cast<TankChassis *>(Parameter);
				AutoStatus LastMoveType = This->AutoMoveType;
				auto AimPosition = This->GetCurPosition();
				while(true){
					if(This->AutoMoveType == Disable){
						This->OpenLoopMove(This->ChassisVelocity);
					}

					// MovPosAbs/Rotate
					else{
						auto CurrentPosition = This->GetCurPosition();
						Vector TempChassisVelocity(RopoMath::ColumnVector,2); // 
						
						// Filter
						AimPosition[1] = RopoMath::LowPassFilter<FloatType>(This->AimPosition[1],AimPosition[1],1,1000.0 / This->SampleTime);
						AimPosition[2] = RopoMath::LowPassFilter<FloatType>(This->AimPosition[2],AimPosition[2],1,1000.0 / This->SampleTime);
						AimPosition[3] = This->AimPosition[3];

						// Solve Delta
						Vector Delta(RopoMath::ColumnVector,3);
						Delta = AimPosition - CurrentPosition;
						while(Delta[3] >= 180.0) Delta[3] -= 360.0;
						while(Delta[3] < -180.0) Delta[3] += 360.0;

						// Solve Temp
						FloatType DeltaDistance = RopoMath::TempDirection(AimPosition[1], AimPosition[2], CurrentPosition[1], CurrentPosition[2]);
						FloatType DeltaRotation = RopoMath::TempDegree(Delta[1],Delta[2]);
						if(This->AutoMoveType == MovePosAbs){
							This->Arrived = false;		
							This->DegreeArrived = SlowDegRegulator.IfArrived();	
							This->DistantArrived = DistanceRegulator.IfArrived();
							if(!This->DegreeArrived){
								TempChassisVelocity[1] = 0;
								TempChassisVelocity[2] = FastDegRegulator.Update(Delta[3]) * 0.8 / ( This->SampleTime / 1000.0 );
							}
							if(!This->DistantArrived && This->DegreeArrived){
								TempChassisVelocity[1] = DistanceRegulator.Update(DeltaDistance) / ( This->SampleTime / 1000.0 );
								// WTF?
								if(DeltaRotation > 90 ) DeltaRotation -= 180;
								else if(DeltaRotation < -90 ) DeltaRotation += 180;
								TempChassisVelocity[2] = DeltaRotation / 45.0 ;
								if(DeltaDistance < 0.2) TempChassisVelocity[2] = 0;
							}
							if(This->DistantArrived && This->DegreeArrived) TempChassisVelocity[1] = TempChassisVelocity[2] = 0, This->Arrived=true;
						}
						if(This->AutoMoveType == Rotate){
							This->Arrived = false;
							This->DegreeArrived = SlowDegRegulator.IfArrived();
							TempChassisVelocity[1] = 0;
							if(!This->DegreeArrived){
								TempChassisVelocity[2] = SlowDegRegulator.Update(Delta[3]) * 0.8 / ( This->SampleTime / 1000.0 );	// *0.8
							}
							else TempChassisVelocity[2] = 0, This->Arrived = true;
						}						
						This->OpenLoopMove(TempChassisVelocity);
					}					
					LastMoveType = This->AutoMoveType;
					pros::delay(This->SampleTime);
				}
			}

			TankChassis(	void (*RightMotorMove)(FloatType),
							void (*LeftMotorMove )(FloatType),
							Vector (*GetPosition_)(),
							int _SampleTime = 5):
				Core( WheelRad, ChassisParameter, DefaultVelocityLimits),
				MotorMove{  RightMotorMove,LeftMotorMove},
				ChassisVelocity(RopoMath::ColumnVector,2),SampleTime(_SampleTime),
				AutoMoveType(Disable),GetCurPosition(GetPosition_),AimPosition(RopoMath::ColumnVector,3),
				DegErrorTolerance(5),Arrived(false),DistantArrived(false),DegreeArrived(false),
				BackgroundTask(nullptr){
				BackgroundTask = new pros::Task(ChassisMoveBackgroundFunction,this);
			}

			void SetVelocityLimits(FloatType VelocityLimits) {Core.SetVelocityLimits(VelocityLimits);}
			void SetDegErrorTolerance(FloatType ErrorTolerance) {DegErrorTolerance = ErrorTolerance;}
			Vector GetChassisVelocity()const{return ChassisVelocity;}
			Vector GetMotorVelocity()const{return MotorVelocity;}
			bool IfArrived()const{return Arrived;}
			bool IfDistantArrived()const{return DistantArrived;}
			bool IfDegreeArrived()const{return DegreeArrived;}

			void MoveVelocity(const Vector& Velocity) {
				ChassisVelocity = Velocity, AutoMoveType = Disable;
			}
			void MoveVelocity(FloatType X,FloatType W){
				ChassisVelocity[1] = X, ChassisVelocity[2] = W, AutoMoveType = Disable;
			}

			void AutoRotateAbs(FloatType AimDegree) {
				AimPosition[3] = AimDegree, AutoMoveType = Rotate, SlowDegRegulator.Reset(), Arrived = false;
				while(!Arrived) pros::delay(20);
			}
			void AutoMovePosAbs(FloatType AimX, FloatType AimY){
				AimPosition[1] = AimX, AimPosition[2] = AimY, AutoMoveType = MovePosAbs, Arrived = false;
				while(!Arrived) pros::delay(20);
			}
	};
}
