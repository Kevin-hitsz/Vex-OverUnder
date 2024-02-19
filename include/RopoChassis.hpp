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
			static constexpr float WheelRad = 0.0508;						//轮子半径
			static constexpr float ChassisParameter = (0.262+0.262)/2; 		//车体宽度
			static constexpr float DefaultVelocityLimits = 600;				//最大速度限制

			//控制器参数为p，i，d，最大值限幅，最小值限幅，误差容限，到达退出时间（秒）
			inline static RopoControl::PIDRegulator DistanceRegulator{0.0026 ,0.0001  ,0.00001 ,0.00075,-0.00075,0.035,0.15};
			//0.0024 ,0.0001  ,0.00001 ,0.00075,-0.00075,0.01,0.3
			inline static RopoControl::PIDRegulator SlowDegRegulator {0.00007,0.000005,0.00002,0.0030 ,-0.0030 ,3   ,0.2};
			//0.00007,0.000003,0.000001,0.0030 ,-0.0030 ,1   ,0.3
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
				const FloatType ChassisRatio = 9.0 / 4.0;
				const FloatType radTorpm = 600 / 62.83;				
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

				FloatType RunDistance=0;	
				FloatType aimDistance=0;				
				FloatType DeltaDis=0;
				FloatType DeltaRotation=0;
				FloatType DegRes=0;
				FloatType DisRes=0;
				
				Vector AimPosition = This->GetCurPosition();
				Vector IniPosition = This->GetCurPosition();
				Vector Delta(RopoMath::ColumnVector,3);
				Vector TempChassisVelocity(RopoMath::ColumnVector,2);
				Vector CurrentPosition=This->GetCurPosition();
				
				while(true)
				{
					if((This->AutoMoveType != Disable)&&This->flag)
					{
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
						AimPosition[1] = RopoMath::LowPassFilter<FloatType>(This->AimPosition[1],AimPosition[1],11,1000.0 / This->SampleTime);//10
						AimPosition[2] = RopoMath::LowPassFilter<FloatType>(This->AimPosition[2],AimPosition[2],11,1000.0 / This->SampleTime);//10
						AimPosition[3] = RopoMath::LowPassFilter<FloatType>(This->AimPosition[3],AimPosition[3],10,1000.0 / This->SampleTime);//10
						
						
						aimDistance=RopoMath::Distance(AimPosition[1]-IniPosition[1],AimPosition[2]-IniPosition[2]);		
						
						
						DeltaRotation = RopoMath::DeltaTwoPoint(CurrentPosition[1],CurrentPosition[2],AimPosition[1],AimPosition[2]);
						Delta[3]=AimPosition[3]-CurrentPosition[3];
						//寻找最小转角（注意+180到-180的角度突变）
						while(Delta[3] >= 180.0) Delta[3] -= 360.0;
						while(Delta[3] < -180.0) Delta[3] += 360.0;
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
								
								//前90%路程车体方向锁定指向末位置，后10%路程车体方向锁定为初始方向，防止末位置方向抖动

								if(!This->moveReverse)
								{
									if(DeltaDis/aimDistance>0.2)
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

									if(DeltaDis/aimDistance>0.1)
										DeltaRotation -= CurrentPosition[3];
									else
										DeltaRotation=IniPosition[3]-CurrentPosition[3];

									
									while(DeltaRotation >= 180.0) DeltaRotation -= 360.0;
									while(DeltaRotation < -180.0) DeltaRotation += 360.0;
								}
								
								
								//方向锁定
								TempChassisVelocity[2] = DeltaRotation *0.02 ;

								

							}
							else
							{
								TempChassisVelocity[1] = 0;
								TempChassisVelocity[2] = 0;
								This->flag=true;
							}
						}

						//自动旋转状态
						else if(This->AutoMoveType == Rotate)
						{
							//是否到达目标角
							This->DegArrived = SlowDegRegulator.IfArrived();

							if(!This->DegArrived)
							{
								DegRes = SlowDegRegulator.Update(Delta[3]);
								TempChassisVelocity[2] = DegRes *1000.0/ This->SampleTime;
							}
							else
							{
								TempChassisVelocity[2] = 0;
								This->flag=true;
							}
							TempChassisVelocity[1] = 0;
						}
						
						
						This->OpenLoopMove(TempChassisVelocity);
					}					
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
				BackgroundTask(nullptr),moveReverse(false),flag(true){
				BackgroundTask = new pros::Task(ChassisMoveBackgroundFunction,this);
			}

			void SetVelocityLimits(FloatType VelocityLimits) {Core.SetVelocityLimits(VelocityLimits);}
			Vector GetChassisVelocity(){return ChassisVelocity;}
			Vector GetMotorVelocity(){return MotorVelocity;}

			/// @brief 单次直行或旋转是否完成
			/// @return 单次直行或旋转是否完成
			bool IfArrived()
			{
				return flag;
			}

			/// @brief 直行是否到达
			/// @return 直行是否到达
			bool IfDisArrived(){return DisArrived;}

			/// @brief 旋转是否到达
			/// @return 旋转是否到达
			bool IfDegArrived(){return DegArrived;}

			/// @brief 赋予车速度
			/// @param Velocity 赋予的速度矢量
			void MoveVelocity(const Vector& Velocity) 
			{
				ChassisVelocity = Velocity;
				AutoMoveType = Disable;
			}

			/// @brief 赋予车速度
			/// @param X 速度直行分量 （m/s）
			/// @param W 速度旋转分量（rad/s）
			void MoveVelocity(FloatType X,FloatType W)
			{
				ChassisVelocity[1] = X;
				ChassisVelocity[2] = W;
				AutoMoveType = Disable;
			}

			/// @brief 旋转至目标角
			/// @param AimDegree 目标角
			void AutoRotateAbs(FloatType AimDegree)
			{
				SlowDegRegulator.Reset();
				flag=false;
				AimPosition[3] = AimDegree;
				AutoMoveType = Rotate;
				DegArrived = false; 
			}

			/// @brief 相对旋转一定角度
			/// @param RelativeDegree 相对角
			void AutoRotateRelative(FloatType RelativeDegree)
			{
				SlowDegRegulator.Reset();
				flag=false;
				RelativeDegree += (GetCurPosition())[3];
				if(RelativeDegree > 180 ) RelativeDegree -= 180;
				if(RelativeDegree < -180 ) RelativeDegree += 180;
				AimPosition[3] = RelativeDegree;
				AutoMoveType = Rotate;
				DegArrived = false; 
			}
			
			/// @brief 直行到目标点
			/// @param AimX 目标X
			/// @param AimY 目标Y
			/// @param move 是否倒车，必须与目标坐标匹配
			void AutoDirectMove(FloatType AimX, FloatType AimY,bool move)
			{
				DistanceRegulator.Reset();
				moveReverse=move;
				flag=false;
				AimPosition[1] = AimX;
				AimPosition[2] = AimY;
				AimPosition[3] = GetCurPosition()[3];
				DisArrived = false;
				AutoMoveType = MoveForward;
			}

			/// @brief 直接使车移动到目标点
			/// @param AimX 目标X坐标
			/// @param AimY 目标Y坐标
			void AutoPositionMove(FloatType AimX,FloatType AimY)
			{
				RopoMath:: Vector CurentPosition=GetCurPosition();
				//旋转指向目标点
				AutoRotateAbs(RopoMath::DeltaTwoPoint(AimX-CurentPosition[1],AimY-CurentPosition[2]));
				//等待到达
				while(!flag) pros::delay(20);
				while(!flag) pros::delay(200);
				
				AutoDirectMove(AimX,AimY,false);
				//等待到达
				while(!flag) pros::delay(20);
				while(!flag) pros::delay(200);
			}

			void AutoPositionMoveBack(FloatType AimX,FloatType AimY)
			{
				RopoMath:: Vector CurentPosition=GetCurPosition();
				//旋转指向目标点
				AutoRotateAbs(RopoMath::DeltaTwoPoint(AimX-CurentPosition[1],AimY-CurentPosition[2])+180);
				//等待到达
				while(!flag) pros::delay(20);
				while(!flag) pros::delay(200);
				
				AutoDirectMove(AimX,AimY,true);
				//等待到达
				while(!flag) pros::delay(20);
				while(!flag) pros::delay(200);
			}

			/// @brief 直接使车移动到目标点并旋转至目标角度
			/// @param AimX 目标X坐标
			/// @param AimY 目标Y坐标
			/// @param Theta 目标角度
			void AutoPositionMove(FloatType AimX,FloatType AimY,FloatType Theta)
			{
				
				RopoMath:: Vector CurentPosition=GetCurPosition();
				//旋转指向目标点
				AutoRotateAbs(RopoMath::DeltaTwoPoint(AimX-CurentPosition[1],AimY-CurentPosition[2]));
				//等待到达
				while(!flag) pros::delay(20);
				while(!flag) pros::delay(100);
				
				AutoDirectMove(AimX,AimY,false);
				//等待到达
				while(!flag) pros::delay(20);
				while(!flag) pros::delay(100);
				//旋转至目标角度
				AutoRotateAbs(Theta);
				while(!flag) pros::delay(20);
				while(!flag) pros::delay(100);
			}

			void AutoPositionMoveBack(FloatType AimX,FloatType AimY,FloatType Theta)
			{
				
				RopoMath:: Vector CurentPosition=GetCurPosition();
				//旋转指向目标点
				AutoRotateAbs(RopoMath::DeltaTwoPoint(AimX-CurentPosition[1],AimY-CurentPosition[2])+180);
				//等待到达
				while(!flag) pros::delay(20);
				while(!flag) pros::delay(100);
				
				AutoDirectMove(AimX,AimY,true);
				//等待到达
				while(!flag) pros::delay(20);
				while(!flag) pros::delay(100);
				//旋转至目标角度
				AutoRotateAbs(Theta);
				while(!flag) pros::delay(20);
				while(!flag) pros::delay(100);
			}
	};
}

#endif //ROPO_CHASSIS_HPP
