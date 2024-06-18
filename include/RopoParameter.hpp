#ifndef ROPO_PARAMETER_HPP
#define ROPO_PARAMETER_HPP

#include "pros/motors.hpp"
#include "RopoMath/Misc.hpp"

namespace RopoParameter {
	//gps 
	//    对于GPS，则将gps对准方向作为gps的前方，第一个参数为gps端口号
	//    第2、3个参数是车体中心在整个场地坐标系中的x、y位置，y轴正方向为0°，x轴正方向是90°，角度遵循顺时针
	//    第 4 个参数是gps传感器初始化时朝向场地的角度方向，y轴正方向为0°，角度遵循顺时针
	//    第 5、6个参数gps位于车体的坐标，若gps所视方向为车体0°，车体y轴正方向为车体0°，则车体x轴正方向为车体90°，角度遵循顺时针


	// 自动一
	static constexpr double FIELD_HEADING_INITIAL = 180.0;//比赛场地相对于战队场地，逆时针为+		(校内场地红方出发为0)
	static constexpr double ROPO_HEADING_INITIAL  = 180.0 - FIELD_HEADING_INITIAL;//对于战队场地X轴
	
	static constexpr int GPS_PORT = 4;
	static constexpr double GPSX_INITIAL_0 = -1.43;
	static constexpr double GPSY_INITIAL_0 = -0.91;
	static double GPSX_INITIAL =  GPSX_INITIAL_0 * RopoMath::Cos(FIELD_HEADING_INITIAL) + GPSY_INITIAL_0 * RopoMath::Sin(FIELD_HEADING_INITIAL);
	static double GPSY_INITIAL = -GPSX_INITIAL_0 * RopoMath::Sin(FIELD_HEADING_INITIAL) + GPSY_INITIAL_0 * RopoMath::Cos(FIELD_HEADING_INITIAL);
	
	static constexpr double GPS_HEADING_INITIAL_0 = 0;
	static constexpr double GPS_HEADING_INITIAL = (GPS_HEADING_INITIAL_0 + FIELD_HEADING_INITIAL) >= 360 ? (GPS_HEADING_INITIAL_0 + FIELD_HEADING_INITIAL -  360) : (GPS_HEADING_INITIAL_0 + FIELD_HEADING_INITIAL);
	
	static constexpr double GPSX_OFFSET = -0.03761;
	static constexpr double GPSY_OFFSET = -0.11660;

	//Chassis
	static constexpr double CHASSIS_PARAMETER = 0.224; 				//车体宽度
	static constexpr double WHEEL_RAD = 0.06985;					//轮子半径
	static constexpr double CHASSIS_RATIO = 1.95;	      		//减速比
	static constexpr double CHASSIS_SPPED_MAX= 600;					//底盘电机最高转速
	static constexpr double CHASSIS_SPPED_MAX_VOLTAGE= 12000;			//底盘电机最高电压
	static constexpr double RAD_TO_RPM= CHASSIS_SPPED_MAX / 62.83;		//角速度转化为rpm

	//Intaker
	static constexpr double INTAKER_SPEED_MAX = 200;					//吸球电机最高转速
	static constexpr double INTAKER_SPEED_MAX_VOLTAGE = 12000;		//吸球电机最高转速

};

#endif // ROPO_PARAMETER_HPP