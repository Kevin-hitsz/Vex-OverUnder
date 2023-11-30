// Code : UTF - 8
#ifndef ROPO_CONTROL_CHASSIS_HPP
#define ROPO_CONTROL_CHASSIS_HPP

#include "Api.hpp"
#include <cmath>

namespace RopoControl{

	class ChassisCore{
		protected:
			bool IfReverse;
			Vector WheelVelocity; // rpm
		public:
			virtual Vector Move(Vector Velocity) = 0;
			ChassisCore():IfReverse{false}{}
			~ChassisCore(){}
			Vector GetWheelVelocity(){return WheelVelocity;}
			void Reverse(){IfReverse ^= 1;}
	};

	class TankChassisCore : public ChassisCore{
		private:
			Matrix ParameterMatrix; // (m)
			FloatType VelocityLimits;
		public:
			TankChassisCore(const FloatType _WheelRad,const FloatType _Parameter,const FloatType _VelocityLimits = 600.0):
				// _Parameter = ( gamma * WheelDistance ) / 2.0 (m)
				// _WheelRad (m)
				ParameterMatrix(2,2),VelocityLimits(_VelocityLimits){
				ParameterMatrix[1][1] = 1, ParameterMatrix[1][2] = _Parameter;
				ParameterMatrix[2][1] = 1, ParameterMatrix[2][2] = -_Parameter;
				ParameterMatrix = ParameterMatrix * (1.0 / (_WheelRad));
			}
			~TankChassisCore(){}

			virtual Vector Move(Vector Velocity){
				// Velocity :Column Vector [Vx,Wc]^T (m/s)
				if(IfReverse)Velocity[1] *= -1;
				WheelVelocity = ParameterMatrix * Velocity;
				FloatType MaxRpm = 0;
				MaxRpm = fmax(MaxRpm,fabs(WheelVelocity[1]));
				MaxRpm = fmax(MaxRpm,fabs(WheelVelocity[2]));
				if(MaxRpm > VelocityLimits)
					WheelVelocity = WheelVelocity * (VelocityLimits / MaxRpm);
				return WheelVelocity;
			}

			void SetVelocityLimits(FloatType _VelocityLimits){
				VelocityLimits = _VelocityLimits;
			}
	};

	class MecanumChassisCore : public ChassisCore{
		private:
			Matrix ParameterMatrix; // (m)
			FloatType VelocityLimits; // (rpm)
		public:
			MecanumChassisCore(const FloatType _WheelRad,const FloatType _Parameter,const FloatType _VelocityLimits = 1000.0):
				// _Parameter = ( ChassisLength + ChassisWidth ) / 2.0 (m)
				// _WheelRad (m)
				ParameterMatrix(4,3),VelocityLimits(_VelocityLimits){
				ParameterMatrix[1][1] = 1 , ParameterMatrix[1][2] = -1 , ParameterMatrix[1][3] = -_Parameter;
				ParameterMatrix[2][1] = 1 , ParameterMatrix[2][2] =  1 , ParameterMatrix[2][3] =  _Parameter;
				ParameterMatrix[3][1] = 1 , ParameterMatrix[3][2] = -1 , ParameterMatrix[3][3] =  _Parameter;
				ParameterMatrix[4][1] = 1 , ParameterMatrix[4][2] =  1 , ParameterMatrix[4][3] = -_Parameter;
				ParameterMatrix = ParameterMatrix * (4.0 / _WheelRad);
			}
			~MecanumChassisCore(){}

			virtual Vector Move(Vector Velocity){
				// Velocity :Column Vector [Vx,Vy,Wc]^T (m/s)
				if(IfReverse)Velocity[1] *= -1, Velocity[2] *= -1;
				WheelVelocity = ParameterMatrix * Velocity;
				FloatType MaxRpm = 0;
				MaxRpm = fmax(MaxRpm,fabs(WheelVelocity[1]));
				MaxRpm = fmax(MaxRpm,fabs(WheelVelocity[2]));
				MaxRpm = fmax(MaxRpm,fabs(WheelVelocity[3]));
				MaxRpm = fmax(MaxRpm,fabs(WheelVelocity[4]));
				if(MaxRpm > VelocityLimits)
					WheelVelocity = WheelVelocity * (VelocityLimits / MaxRpm);
				return WheelVelocity;
			}

			void SetVelocityLimits(FloatType _VelocityLimits){
				VelocityLimits = _VelocityLimits;
			}
	};

}

#endif //ROPO_CONTROL_CHASSIS_HPP
