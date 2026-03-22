// Code : UTF - 8
#ifndef ROPO_MATH_MISC_HPP
#define ROPO_MATH_MISC_HPP

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
        else if(Input < 0.0) return -1;
		else return 0;
	}
	template<class T>inline T Sat(T Input){
        // Sat(x) = x / (|x| + delta), in which delta is a really small value
        static const double Delta = 1E-3;
        return Input / (fabs(Input) + Delta);
    }

	/**
	 * 一阶IIR低通滤波器
	 * 
	 * @param Input 信号输入
	 * @param Last_Output 上一个时刻的滤波器输出
	 * @param CutOff	截止频率
	 * @param Sample_Rate	采样率
	 * @return 滤波输出
	*/
	template<class T>inline T LowPassFilter(T Intput, T Last_Output, T CutOff, T Sample_Rate){
        // Cutoff Frequency unit: Hz
        // Sample_Rate unit: Hz
        double K = 2 * Pi * CutOff / Sample_Rate;
        return K * Intput + (1 - K) * Last_Output;
    }

	//将输入限制在（-limit,limit）
	template<class T>inline T Limit(T Input, T limit){
		
		if(Input > limit) Input = limit;
		else if (Input < -limit)
			Input = -limit;
		return Input;
	}

	//平方平均
	template<class T>inline T Distance(T X,T Y){
		return sqrt(X * X + Y * Y);
	}

	//返回点1指向点2矢量与X轴夹角
	template<class T>inline T DeltaTwoPoint(T X1,T Y1,T X2,T Y2){
		T DeltaRotation;
		if(X2-X1 == 0){
			if(Y2-Y1 > 0){
				DeltaRotation = 90.0;
			}
			else if(Y2-Y1 < 0){
				DeltaRotation = -90.0;
			}
			else {
				DeltaRotation = 0;
			}
			return DeltaRotation;
		}
		else{
			DeltaRotation = Atan<T>((Y2-Y1)/(X2-X1));
			
			if(X2-X1 < 0 && Y2-Y1 >= 0){
				DeltaRotation += 180;
			}
			if(X2-X1 < 0 && Y2-Y1 < 0){
				DeltaRotation -= 180;
			}
			return DeltaRotation;
		}
	}
	
	
	//返回点1指向点2矢量与X轴夹角
	template<class T>inline T DeltaTwoPoint(T DeltaX,T DeltaY){
		T DeltaRotation;
		if(DeltaX == 0){
			if(DeltaY > 0){
				DeltaRotation = 90.0;
			}
			else if(DeltaY < 0){
				DeltaRotation = -90.0;
			}
			else {
				DeltaRotation = 0;
			}
			return DeltaRotation;
		}
		else{
			DeltaRotation = Atan<T>( (DeltaY) / (DeltaX) );
			
			if(DeltaX < 0 && DeltaY >= 0){
				DeltaRotation += 180;
			}
			if(DeltaX < 0 && DeltaY < 0){
				DeltaRotation -= 180;
			}
			return DeltaRotation;
		}
	}

}

#endif //ROPO_MATH_MISC_HPP
