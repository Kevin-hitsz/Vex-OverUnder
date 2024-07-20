#ifndef ROPO_PARAMETER_HPP
#define ROPO_PARAMETER_HPP

#include "pros/motors.hpp"
#include "RopoMath/Misc.hpp"

namespace RopoParameter {
	//gps 
	//    对于GPS，则将gps对准方向作为gps的前方，第一个参数为gps端口号
	//    第2、3个参数是车体中心在整个场地坐标系中的x、y位置，y轴正方向为0°，x轴正方向是90°，角度遵循顺时针		// 车相对地垫绝对坐标
	//    第 4 个参数是gps传感器初始化时朝向场地的角度方向，y轴正方向为0°，角度遵循顺时针						// GPS朝向相对地垫绝对角度
	//    第 5、6个参数gps位于车体的坐标，若gps所视方向为车体0°，车体y轴正方向为车体0°，则车体x轴正方向为车体90°，角度遵循顺时针


	// // 自动一
	// static constexpr double FIELD_HEADING_INITIAL = 0;//比赛场地相对于战队场地，逆时针为+		(校内场地红方出发为0)
	// static constexpr double ROPO_HEADING_INITIAL  = 90.0 - FIELD_HEADING_INITIAL;// 车头方向对于战队场地X轴（逆时针为正）
	
	// static constexpr int GPS_PORT = 11;
	// static constexpr double GPSX_INITIAL_0 = -0.855;	
	// static constexpr double GPSY_INITIAL_0 = -1.394;
	// static double GPSX_INITIAL =  GPSX_INITIAL_0 * RopoMath::Cos(FIELD_HEADING_INITIAL) + GPSY_INITIAL_0 * RopoMath::Sin(FIELD_HEADING_INITIAL);
	// static double GPSY_INITIAL = -GPSX_INITIAL_0 * RopoMath::Sin(FIELD_HEADING_INITIAL) + GPSY_INITIAL_0 * RopoMath::Cos(FIELD_HEADING_INITIAL);
	
	// static constexpr double GPS_HEADING_INITIAL_0 = 270;
	// static constexpr double GPS_HEADING_INITIAL = (GPS_HEADING_INITIAL_0 + FIELD_HEADING_INITIAL) >= 360 ? (GPS_HEADING_INITIAL_0 + FIELD_HEADING_INITIAL -  360) : (GPS_HEADING_INITIAL_0 + FIELD_HEADING_INITIAL);
	
	// static constexpr double GPSX_OFFSET = -0.14600;
	// static constexpr double GPSY_OFFSET =  0.17500;

	// 自动二
	static constexpr double FIELD_HEADING_INITIAL = 180;//比赛场地相对于战队场地，逆时针为+		(校内场地红方出发为0)、
	// 蓝方90 红方0
	static constexpr double ROPO_HEADING_INITIAL  = -45.0 - FIELD_HEADING_INITIAL;// 车头方向对于战队场地X轴（逆时针为正）
	
	static constexpr int GPS_PORT = 11;
	static constexpr double GPSX_INITIAL_0 = -1.380;	
	static constexpr double GPSY_INITIAL_0 = -1.325;	
	static double GPSX_INITIAL =  GPSX_INITIAL_0 * RopoMath::Cos(FIELD_HEADING_INITIAL) + GPSY_INITIAL_0 * RopoMath::Sin(FIELD_HEADING_INITIAL);
	static double GPSY_INITIAL = -GPSX_INITIAL_0 * RopoMath::Sin(FIELD_HEADING_INITIAL) + GPSY_INITIAL_0 * RopoMath::Cos(FIELD_HEADING_INITIAL);
	
	static constexpr double GPS_HEADING_INITIAL_0 = 50.0;
	static constexpr double GPS_HEADING_INITIAL = (GPS_HEADING_INITIAL_0 + FIELD_HEADING_INITIAL) >= 360 ? (GPS_HEADING_INITIAL_0 + FIELD_HEADING_INITIAL -  360) : (GPS_HEADING_INITIAL_0 + FIELD_HEADING_INITIAL);
	
	static constexpr double GPSX_OFFSET = -0.08500;
	static constexpr double GPSY_OFFSET =  0.17500;

	//Chassis
	static constexpr double CHASSIS_PARAMETER = 0.224; 					//车体宽度
	static constexpr double WHEEL_RAD = 0.06985 / 2;					//轮子半径	// 0.06985
	static constexpr double CHASSIS_RATIO = 48.0 / 38.0;	      		//减速比	// 1.95		// 1.25
	static constexpr double CHASSIS_SPPED_MAX= 600;						//底盘电机最高转速
	static constexpr double CHASSIS_SPPED_MAX_VOLTAGE= 12000;			//底盘电机最高电压
	static constexpr double RAD_TO_RPM= CHASSIS_SPPED_MAX / 62.83;		//角速度转化为rpm

	//Intaker
	static constexpr double INTAKER_SPEED_MAX = 200;					//吸球电机最高转速
	static constexpr double INTAKER_SPEED_MAX_VOLTAGE = 12000;			//吸球电机最高转速

};

#endif // ROPO_PARAMETER_HPP