// Code : UTF - 8
#ifndef ROPO_CONTROL_REGULATOR_HPP
#define ROPO_CONTROL_REGULATOR_HPP

#include "Api.hpp"

namespace RopoControl{

	template<class T>inline T Limit(T Input, T HighLimit, T LowLimit){
		return (Input > HighLimit)?(HighLimit):( (Input < LowLimit)?(LowLimit):(Input) );
	}

	class Regulator{
		protected:
			bool Arrived;
		public:
			Regulator(){}
			~Regulator(){}
			virtual FloatType Update(FloatType Error) = 0;
			virtual void Reset() = 0;
			bool IfArrived(){return Arrived;};
	};

	class PRegulator : public Regulator{
		protected:
			FloatType Kp;
			FloatType OutputLimitHigh;
			FloatType OutputLimitLow;
			FloatType ErrorTol;
		public:
			PRegulator(FloatType _Kp,FloatType _OutputLimitHigh,FloatType _OutputLimitLow,FloatType _ErrorTol):
				Kp(_Kp),OutputLimitHigh(_OutputLimitHigh),OutputLimitLow(_OutputLimitLow),ErrorTol(_ErrorTol){}
			virtual FloatType Update(FloatType Error){
				if(Error < ErrorTol)Arrived = true;
				else Arrived = false;
				return Limit( Error * Kp, OutputLimitHigh, OutputLimitLow );
			}
			virtual void Reset(){Arrived = false;}
	};

	class PIDRegulator : public Regulator{
		protected:
			FloatType Kp,Ki,Kd;
			FloatType OutputLimitHigh;
			FloatType OutputLimitLow;
			FloatType ErrorTol;
			FloatType JumpTime;
			bool First;
		public:
			PIDRegulator(FloatType _Kp,FloatType _Ki,FloatType _Kd,FloatType _OutputLimitHigh,FloatType _OutputLimitLow,FloatType _ErrorTol,FloatType _JumpTime = 0.03):
				Kp(_Kp),Ki(_Ki),Kd(_Kd),OutputLimitHigh(_OutputLimitHigh),OutputLimitLow(_OutputLimitLow),ErrorTol(_ErrorTol),JumpTime(_JumpTime),First(true){}
			virtual FloatType Update(FloatType Error){
				static FloatType PreError;
				static FloatType IntError;
				static FloatType DevError;
				static FloatType Time;
				static FloatType ArrivedTime;
				if(First){
					PreError = Error;
					IntError = 0;
					Time = GetSystemTimeInSec();
					First = false;
					ArrivedTime = -1;
				}
				IntError += Error * ( GetSystemTimeInSec() - Time );
				if(IntError * Error < 0.0)
					IntError = 0;
				Time = GetSystemTimeInSec();
				DevError = Error - PreError;
				PreError = Error;
				if( fabs(Error) < ErrorTol ){
					if(ArrivedTime < 0.0) 
						ArrivedTime = GetSystemTimeInSec();
					else if(GetSystemTimeInSec() - ArrivedTime > JumpTime)
						Arrived = true;
				}
				else ArrivedTime = -1 , Arrived = false;
				return Arrived?0:Limit( Kp * Error + Ki * IntError + Kd * DevError , OutputLimitHigh, OutputLimitLow) ;
			}
			virtual void Reset(){First = true,Arrived = false;}
	};

}

#endif //ROPO_CONTROL_REGULATOR_HPP
