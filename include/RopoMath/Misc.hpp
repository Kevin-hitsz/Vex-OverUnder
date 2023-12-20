// Code : UTF - 8
#ifndef ROPO_MATH_MISC_HPP
#define ROPO_MATH_MISC_HPP

#include "RopoApi.hpp"
#include <algorithm>
#include <cmath>

namespace RopoMath{

	const double Pi = 3.1415926535;
	// const double Esp = 1e-8;

	// Input in rad
	template<class T>inline T acot(T Input){
		return atan(1 / Input);
	}

	// Input in degree
	template<class T>inline T Sin(T Input){
		return sin(Pi / 180.0 * Input);
	}
	template<class T>inline T Cos(T Input){
		return cos(Pi / 180.0 * Input);
	}
	template<class T>inline T Tan(T Input){
		return tan(Pi / 180.0 * Input);
	}
	template<class T>inline T Cot(T Input){
		return cot(Pi / 180.0 * Input);
	}
	template<class T>inline T Acos(T Input){
		return 180.0 / Pi * acos(Input);
	}
	template<class T>inline T Asin(T Input){
		return 180.0 / Pi * asin(Input);
	}
	template<class T>inline T Atan(T Input){
		return 180.0 / Pi * atan(Input);
	}
	template<class T>inline T Acot(T Input){
		return 180.0 / Pi * acot(Input);
	}

	// Misc function
	template<class T>inline T Sign(T Input){
		if(Input > 0.0) return 1;
        if(Input < 0.0) return -1;
		return 0;
	}
	template<class T>inline T Sat(T Input){
        // Sat(x) = x / (|x| + delta), in which delta is a really small value
        static const double Delta = 1E-3;
        return Input / (fabs(Input) + Delta);
    }
	template<class T>inline T LowPassFilter(T Intput, T Last_Input, T CutOff, T Sample_Rate){
        // Cutoff Frequency unit: Hz
        // Sample_Rate unit: Hz
        double K = 2 * Pi * CutOff / Sample_Rate;
        return K * Intput + (1 - K) * Last_Input;
    }
	template<class T>inline T Limit(T Input, T limit){
		// Limit the x in (-limit, limit)
		if(Input > limit) Input = limit;
		else if (Input < -limit)
			Input = -limit;
		return Input;
	}
	template<class T>inline T Distance(T X, T Y){
		// Get Distance from (x,y) to (0,0)
		return sqrt(X*X + Y*Y);  
	}

	template<class T>inline T DeltaDirection(T AimX,T AimY,T CurX,T CurY){
		// Get Delta Distance from (CurX,CurY) to (AimX,AimY) with Direction
		T DeltaX = AimX - CurX;
		T DeltaY = AimY - CurY;
		return Sign(Distance(AimX,AimY) - Distance(CurX, CurY)) * sqrt(DeltaX * DeltaX + DeltaY * DeltaY);
	}

	template<class T>inline T DeltaDegree(T DeltaX,T DeltaY) {
		// Get Delta Vector Degree from X
		FloatType Deg = atan2(DeltaY,DeltaX);
		 
		return 180 / Pi * Deg;
	}
}

#endif //ROPO_MATH_MISC_HPP
