// Code : UTF - 8
#ifndef ROPO_CONTROL_API_HPP
#define ROPO_CONTROL_API_HPP

#include "../RopoApi.hpp"
#include "../RopoMath/Matrix.hpp"
#include "../RopoMath/Vector.hpp"
#include "../pros/rtos.hpp"

namespace RopoControl{

	#ifdef _PROS_RTOS_HPP_
		/// @brief 获取设备初始化到现在的时间（秒）
		/// @return 设备初始化到现在的时间（秒）
		FloatType GetSystemTimeInSec(){return pros::millis() / 1000.0;}
	#else
		FloatType GetSystemTimeInSec(){return 0;}
	#endif

}

#endif //ROPO_CONTROL_API_HPP
