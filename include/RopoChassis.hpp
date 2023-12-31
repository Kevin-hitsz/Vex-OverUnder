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
#include "RopoPosition.hpp"
#include <algorithm>


namespace RopoChassis{
	// Api

	typedef RopoApi::FloatType FloatType;
	typedef RopoMath::Vector<FloatType> Vector;
	typedef RopoMath::Matrix<FloatType> Matrix;

	// Code
	class TankChassis{
		private:
			static constexpr float WheelRad = 0.034925;
			static constexpr float ChassisParameter = (0.295+0.295)/2;//0.2855
			static constexpr float DefaultVelocityLimits = 600;

			inline static RopoControl::PIDRegulator DistanceRegulator{0.004,0.001,0.00000,0.0006,-1e7,0.15,0.3};		//
			inline static RopoControl::PIDRegulator SlowDegRegulator{0.000072,0.00019,0.000101,0.0040,-1e7,1.6,0.4};		//
			//0.00001,0.00012,0.0004
			RopoControl::TankChassisCore Core;
			void (*MotorMove[2])(FloatType);
			Vector ChassisVelocity;
			Vector MotorVelocity;
			const int SampleTime;
			int DetectionX ;

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
			bool Arrived, DisArrived, DegArrived;

			pros::Task* BackgroundTask;

			void OpenLoopMove(const Vector& Velocity) {
				const FloatType ChassisRatio = 6.0 / 5.0;
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
					if(LastMoveType != This->AutoMoveType){
						This->Arrived = false;
						DistanceRegulator.Reset();
						SlowDegRegulator.Reset();
						AimPosition = This->GetCurPosition();
					}
					if(This->AutoMoveType == Disable){
						This->OpenLoopMove(This->ChassisVelocity);
					}
					else{
						auto CurrentPosition = This->GetCurPosition();
						Vector TempChassisVelocity(RopoMath::ColumnVector,2);
						
						AimPosition[1] = RopoMath::LowPassFilter<FloatType>(This->AimPosition[1],AimPosition[1],1,1000.0 / This->SampleTime);
						AimPosition[2] = RopoMath::LowPassFilter<FloatType>(This->AimPosition[2],AimPosition[2],1,1000.0 / This->SampleTime);
						// AimPosition[3] = RopoMath::LowPassFilter<FloatType>(This->AimPosition[3],AimPosition[3],1,1000.0 / This->SampleTime);
						AimPosition[3] = This->AimPosition[3];

						Vector Delta(RopoMath::ColumnVector,3);
						
						Delta[1] = AimPosition[1] - CurrentPosition[1];
						Delta[2] = AimPosition[2] - CurrentPosition[2];
						Delta[3] = AimPosition[3] - CurrentPosition[3];
						FloatType DeltaRotation = RopoMath::DeltaTwoPoint(CurrentPosition[1],CurrentPosition[2],AimPosition[1],AimPosition[2]);
						while(Delta[3] >= 180.0) Delta[3] -= 360.0;
						while(Delta[3] < -180.0) Delta[3] += 360.0;

						if(This->AutoMoveType == MoveForward){
							// 此情况默认车头正对
							// FloatType DeltaDis = RopoMath::Distance(Delta[1],Delta[2]);		
							// if(Delta[1] < 0)	DeltaDis *= -1; 

							// This->DisArrived = DistanceRegulator.IfArrived();

							// if(!This->DisArrived){
							// 	FloatType DisRes = DistanceRegulator.Update(DeltaDis);
							// 	TempChassisVelocity[1] = DisRes / ( This->SampleTime / 1000.0 );
							// 	This->Arrived = false;
							// 	//TempChassisVelocity[2] = TempChassisVelocity[1]
							// }
							// else{
							// 	TempChassisVelocity[1] = 0;
							// 	This->Arrived = true;
							// 	TempChassisVelocity[2] = 0;
							// }
							// TempChassisVelocity[2] = 0;

							FloatType DeltaDis = RopoMath::Distance(Delta[1],Delta[2]);		
							if(Delta[1] < 0)	DeltaDis *= -1; 
							DeltaDis *= This -> DetectionX;
							This->DisArrived = DistanceRegulator.IfArrived();

							if(!This->DisArrived){
								FloatType DisRes = DistanceRegulator.Update(DeltaDis);
								TempChassisVelocity[1] = DisRes / ( This->SampleTime / 1000.0 );
								FloatType _R = DeltaDis;//圆心在左正，圆心在右负
								DeltaRotation -= CurrentPosition[3];
								if(_R > 0.5){
									_R = (_R - 0.5)/3.0+_R;
								}
								if(DeltaRotation > 90 ){
									DeltaRotation -= 180;
								}
								if(DeltaRotation < -90 ){
									DeltaRotation += 180;
								}
								// _R = _R/RopoMath::Sin(DeltaRotation/2.0);

								// if(_R > 0){
								// 	_R = std::max(_R,0.5);			
								// }
								// if(_R < 0){
								// 	_R = std::min(_R,-0.5);
								// }
								// if (TempChassisVelocity[1] > 3.0){	
								// 	TempChassisVelocity[1] = 3.0;
								// }
								// TempChassisVelocity[2] = TempChassisVelocity[1]/_R;
								TempChassisVelocity[2] = DeltaRotation / 45.0 ;
								if(DeltaDis < 0.2){
									TempChassisVelocity[2] = 0;
								}

							}
							else{
								TempChassisVelocity[1] = 0;
								TempChassisVelocity[2] = 0;
							}
						}
						else if(This->AutoMoveType == Rotate){
							This->DegArrived = SlowDegRegulator.IfArrived();

							if(!This->DegArrived){
								FloatType DegRes = SlowDegRegulator.Update(Delta[3]);
								TempChassisVelocity[2] = DegRes / ( This->SampleTime / 1000.0 );
								This->Arrived = false;
							}
							else{
								TempChassisVelocity[2] = 0;
								This->Arrived = true;
							}
							TempChassisVelocity[1] = 0;
						}
						else if(This->AutoMoveType == MovePosAbs){


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
				DegErrorTolerance(5),Arrived(false),DisArrived(false),DegArrived(false),
				BackgroundTask(nullptr){
				BackgroundTask = new pros::Task(ChassisMoveBackgroundFunction,this);
			}

			void SetVelocityLimits(FloatType VelocityLimits) {Core.SetVelocityLimits(VelocityLimits);}
			void SetDegErrorTolerance(FloatType ErrorTolerance) {DegErrorTolerance = ErrorTolerance;}
			Vector GetChassisVelocity(){return ChassisVelocity;}
			Vector GetMotorVelocity(){return MotorVelocity;}

			bool IfArrived(){return Arrived;}
			bool IfDisArrived(){return DisArrived;}
			bool IfDegArrived(){return DegArrived;}

			void MoveVelocity(const Vector& Velocity) {
				ChassisVelocity = Velocity, AutoMoveType = Disable;
			}
			void MoveVelocity(RopoApi::FloatType X,RopoApi::FloatType W){
				ChassisVelocity[1] = X;
				ChassisVelocity[2] = W;
				AutoMoveType = Disable;
			}

			void AutoRotateAbs(FloatType AimDegree) {
				AimPosition[3] = AimDegree, AutoMoveType = Rotate, DegArrived = false, SlowDegRegulator.Reset();
			}
			void AutoDirectMove(FloatType AimX, FloatType AimY){
				AimPosition[1] = AimX;
				AimPosition[2] = AimY;
				AimPosition[3] = GetCurPosition()[3];
				AutoMoveType = MoveForward, DisArrived = false, DistanceRegulator.Reset();
			}
			void AutoMovePosAbs(FloatType xPos, FloatType yPos, FloatType theta){
				Arrived = false;
				auto CurPosition = GetCurPosition();
				FloatType DeltaX = xPos - CurPosition[1], DeltaY = yPos -CurPosition[2];
				if (DeltaX < 0) {
					DetectionX = -1;
				}
				else {
					DetectionX = 1;
				}
				FloatType TurnDeg = RopoMath::DeltaTwoPoint(DeltaX, DeltaY);

				AutoRotateAbs(TurnDeg);
				while (!DegArrived)
				{
					pros::delay(20);
				}
				
				AutoDirectMove(xPos,yPos);
				while(!DisArrived){
					pros::delay(20);
				}

				AutoRotateAbs(theta);
				while (!DegArrived)
				{
					pros::delay(20);
				}
				Arrived = true;
			}
			void AutoMovePosAbsBack(FloatType xPos, FloatType yPos, FloatType theta){
				Arrived = false;
				auto CurPosition = GetCurPosition();

				FloatType DeltaX = xPos - CurPosition[1], DeltaY = yPos -CurPosition[2];
				if (DeltaX < 0) {
					DetectionX = 1;
				}
				else {
					DetectionX = -1;
				}
				FloatType TurnDeg = RopoMath::DeltaTwoPoint(DeltaX, DeltaY);
				if (TurnDeg < 0) {
					TurnDeg += 180;
				} else {
					TurnDeg -= 180;
				}
				AutoRotateAbs(TurnDeg);
				while (!DegArrived)
				{
					pros::delay(20);
				}
				
				AutoDirectMove(xPos,yPos);
				double Time = pros::millis();
				while((!DisArrived) && (pros::millis()-Time <3000) ){
					pros::delay(20);
				}

				AutoRotateAbs(theta);
				while (!DegArrived)
				{
					pros::delay(20);
				}
				Arrived = true;
			}
	};
}

#endif //ROPO_CHASSIS_HPP
