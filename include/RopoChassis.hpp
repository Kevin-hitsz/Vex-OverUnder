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
	
	class TankChassis{
		private:
			static constexpr float WheelRad = 0.034925;						//轮子半径
			static constexpr float ChassisParameter = (0.295+0.295)/2; 		//车体宽度
			static constexpr float DefaultVelocityLimits = 600;				//最大速度限制

			//控制器参数为p，i，d，最大值限幅，最小值限幅，误差容限，到达退出时间（秒）
			inline static RopoControl::PIDRegulator DistanceRegulator{0.004,0.0004,0.00000,0.0004,-0.0004,0.01,0.3};
			inline static RopoControl::PIDRegulator SlowDegRegulator{0.000036,0.00001,0.000001,0.0030,-0.0030,1,0.3};
			
			RopoControl::TankChassisCore Core;								
			void (*MotorMove[2])(FloatType);
			Vector ChassisVelocity;
			Vector MotorVelocity;
			const int SampleTime;			//采样间隔
			
			
			
			enum AutoStatus{
				Disable = -1,				//手动状态
				MovePosAbs = 0,				//
				MoveForward = 1,			//直行
				Rotate = 2					//旋转
			} AutoMoveType;
			
			Vector (*GetCurPosition)();
			Vector AimPosition;									//目标位姿（x，y，theta）
			bool DisArrived;									//距离到达标志 
			bool DegArrived;									//角度到达标志

			pros::Task* BackgroundTask;
			bool moveReverse=false;
			bool flag;
			
			void OpenLoopMove(const Vector& Velocity) {
				const FloatType ChassisRatio = 56.0 / 44.0;
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


			static void ChassisMoveBackgroundFunction(void *Parameter){
				if(Parameter == nullptr)return;
				TankChassis *This = static_cast<TankChassis *>(Parameter);
				AutoStatus LastMoveType = This->AutoMoveType;
				auto AimPosition = This->GetCurPosition();
				auto IniPosition = This->GetCurPosition();
				FloatType RunDistance=0;	
				FloatType aimDistance=0;				
				Vector Delta(RopoMath::ColumnVector,3);
				Vector TempChassisVelocity(RopoMath::ColumnVector,2);
				FloatType DisRes=0;
				FloatType DeltaDis=0;
				FloatType DeltaRotation=0;
				auto CurrentPosition=This->GetCurPosition();
				FloatType DegRes=0;
				//指示单个动作是否完成
				
				
				while(true){
					if((LastMoveType != This->AutoMoveType)||This->flag){
						DistanceRegulator.Reset();
						SlowDegRegulator.Reset();
						AimPosition = This->GetCurPosition();
						IniPosition = This->GetCurPosition();
					}

					if(This->AutoMoveType == Disable)
					{
						This->OpenLoopMove(This->ChassisVelocity);
					}
					else if(!This->flag)
					{
						
						CurrentPosition = This->GetCurPosition();
						//输入平滑
						AimPosition[1] = RopoMath::LowPassFilter<FloatType>(This->AimPosition[1],AimPosition[1],10,1000.0 / This->SampleTime);
						AimPosition[2] = RopoMath::LowPassFilter<FloatType>(This->AimPosition[2],AimPosition[2],10,1000.0 / This->SampleTime);
						AimPosition[3] = RopoMath::LowPassFilter<FloatType>(This->AimPosition[3],AimPosition[3],10,1000.0 / This->SampleTime);
						
						
						aimDistance=RopoMath::Distance(AimPosition[1]-IniPosition[1],AimPosition[2]-IniPosition[2]);		
						
						
						DeltaRotation = RopoMath::DeltaTwoPoint(CurrentPosition[1],CurrentPosition[2],AimPosition[1],AimPosition[2]);
						Delta[3]=AimPosition[3]-CurrentPosition[3];
						//防止+180到-180的角度突变
						while(Delta[3] >= 180.0) Delta[3] -= 360.0;
						while(Delta[3] < -180.0) Delta[3] += 360.0;
						pros::lcd::print(2,"%f,%f,%f",AimPosition[3],CurrentPosition[3],Delta[3]);
						//自动直行状态
						
						if(This->AutoMoveType == MoveForward)
						{
							
							//已走距离
							RunDistance = RopoMath::Distance(CurrentPosition[1]-IniPosition[1],CurrentPosition[2]-IniPosition[2]);		
							//距离误差
							DeltaDis=aimDistance-RunDistance;
							//是否到达
							This->DisArrived = DistanceRegulator.IfArrived();

							if(!This->DisArrived)
							{
								
								//pid计算直行控制量
								DisRes = DistanceRegulator.Update(DeltaDis);
								TempChassisVelocity[1] = (This->moveReverse?-1:1)*(DisRes / ( This->SampleTime / 1000.0) );
								
								//前80%路程车体方向锁定指向末位置，后20%路程车体方向锁定为初始方向，防止末位置方向抖动

								if(!This->moveReverse)
								{
									if(DeltaDis/aimDistance>0.1)
										DeltaRotation -= CurrentPosition[3];
									else
										DeltaRotation=IniPosition[3]-CurrentPosition[3];

									
									while(DeltaRotation >= 180.0) DeltaRotation -= 360.0;
									while(DeltaRotation < -180.0) DeltaRotation += 360.0;
								}
								else
								{
									if(DeltaRotation>90)
										DeltaRotation-=180;
									else if(DeltaRotation<-90)
										DeltaRotation+=180;

									if(DeltaDis/aimDistance>0.2)
										DeltaRotation -= CurrentPosition[3];
									else
										DeltaRotation=IniPosition[3]-CurrentPosition[3];

									
									while(DeltaRotation >= 180.0) DeltaRotation -= 360.0;
									while(DeltaRotation < -180.0) DeltaRotation += 360.0;
								}
								
								
								//方向锁定
								TempChassisVelocity[2] = DeltaRotation *0.02 ;

								

								//调试打印
								// pros::lcd::print(1,"%f,%f,%f",CurrentPosition[1],AimPosition[1],TempChassisVelocity[1]);
								
							}
							else
							{
								pros::lcd::print(3,"arrive1");
								This->flag=true;
								TempChassisVelocity[1] = 0;
								TempChassisVelocity[2] = 0;
							}
						}

						//自动旋转状态
						else if(This->AutoMoveType == Rotate)
						{
							//是否到达目标角
							This->DegArrived = SlowDegRegulator.IfArrived();

							if(!This->DegArrived){
								
								DegRes = SlowDegRegulator.Update(Delta[3]);
								TempChassisVelocity[2] = DegRes *1000.0/ This->SampleTime;
								// This->Arrived = false;
							}
							else{
								This->flag=true;
								TempChassisVelocity[2] = 0;

								// This->Arrived = true;
							}
							TempChassisVelocity[1] = 0;
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
				DisArrived(false),DegArrived(false),
				BackgroundTask(nullptr),moveReverse(false),flag(false){
				BackgroundTask = new pros::Task(ChassisMoveBackgroundFunction,this);
			}

			void SetVelocityLimits(FloatType VelocityLimits) {Core.SetVelocityLimits(VelocityLimits);}
			Vector GetChassisVelocity(){return ChassisVelocity;}
			Vector GetMotorVelocity(){return MotorVelocity;}

			bool IfDisArrived(){return DisArrived;}
			bool IfDegArrived(){return DegArrived;}

			//手动赋予速度
			void MoveVelocity(const Vector& Velocity) 
			{
				ChassisVelocity = Velocity;
				AutoMoveType = Disable;
			}

			//手动赋予速度
			void MoveVelocity(FloatType X,FloatType W)
			{
				ChassisVelocity[1] = X;
				ChassisVelocity[2] = W;
				AutoMoveType = Disable;
			}

			//自动旋转至目标角度
			void AutoRotateAbs(FloatType AimDegree)
			{
				flag=false;
				AimPosition[3] = AimDegree;
				AutoMoveType = Rotate;
				DegArrived = false; 
				SlowDegRegulator.Reset();
			}

			//自动控制直行
			void AutoDirectMove(FloatType AimX, FloatType AimY,bool move)
			{
				moveReverse=move;
				flag=false;
				AimPosition[1] = AimX;
				AimPosition[2] = AimY;
				AimPosition[3] = GetCurPosition()[3];
				AutoMoveType = MoveForward, DisArrived = false, DistanceRegulator.Reset();
			}
			//鸡毛
	// 		void AutoMovePosAbs(FloatType xPos, FloatType yPos, FloatType theta){
	// 			Arrived = false;
	// 			auto CurPosition = GetCurPosition();
	// 			FloatType DeltaX = xPos - CurPosition[1], DeltaY = yPos -CurPosition[2];
	// 			if (DeltaX < 0) {
	// 				DetectionX = -1;
	// 			}
	// 			else {
	// 				DetectionX = 1;
	// 			}
	// 			FloatType TurnDeg = RopoMath::DeltaTwoPoint(DeltaX, DeltaY);

	// 			AutoRotateAbs(TurnDeg);
	// 			while (!DegArrived)
	// 			{
	// 				pros::delay(20);
	// 			}
				
	// 			AutoDirectMove(xPos,yPos);
	// 			while(!DisArrived){
	// 				pros::delay(20);
	// 			}

	// 			AutoRotateAbs(theta);
	// 			while (!DegArrived)
	// 			{
	// 				pros::delay(20);
	// 			}
	// 			Arrived = true;
	// 		}

	// 		//
	// 		void AutoMovePosAbsBack(FloatType xPos, FloatType yPos, FloatType theta){
	// 			Arrived = false;
	// 			auto CurPosition = GetCurPosition();

	// 			FloatType DeltaX = xPos - CurPosition[1], DeltaY = yPos -CurPosition[2];
	// 			if (DeltaX < 0) {
	// 				DetectionX = 1;
	// 			}
	// 			else {
	// 				DetectionX = -1;
	// 			}
	// 			FloatType TurnDeg = RopoMath::DeltaTwoPoint(DeltaX, DeltaY);
	// 			if (TurnDeg < 0) {
	// 				TurnDeg += 180;
	// 			} else {
	// 				TurnDeg -= 180;
	// 			}
	// 			AutoRotateAbs(TurnDeg);
	// 			while (!DegArrived)
	// 			{
	// 				pros::delay(20);
	// 			}
				
	// 			AutoDirectMove(xPos,yPos);
	// 			while(!DisArrived){
	// 				pros::delay(20);
	// 			}

	// 			AutoRotateAbs(theta);
	// 			while (!DegArrived)
	// 			{
	// 				pros::delay(20);
	// 			}
	// 			Arrived = true;
	// 		}
	};
}

#endif //ROPO_CHASSIS_HPP
