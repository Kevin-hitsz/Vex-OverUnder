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


	// 资格赛gps参数
	// static constexpr double FIELD_HEADING_INITIAL = 180;//比赛场地相对于战队场地，逆时针为+		(校内场地红方出发为0)  校内比赛：红方0，蓝方180
	// static constexpr double ROPO_HEADING_INITIAL  = 180 - FIELD_HEADING_INITIAL;
	// static constexpr int GPS_PORT = 6;
	// static constexpr double GPSX_INITIAL_0 = -0.6 - 0.150;
	// static constexpr double GPSY_INITIAL_0 = -1.2 - 0.111 - 0.1736;
	// static double GPSX_INITIAL =  GPSX_INITIAL_0 * RopoMath::Cos(FIELD_HEADING_INITIAL) + GPSY_INITIAL_0 * RopoMath::Sin(FIELD_HEADING_INITIAL);
	// static double GPSY_INITIAL = -GPSX_INITIAL_0 * RopoMath::Sin(FIELD_HEADING_INITIAL) + GPSY_INITIAL_0 * RopoMath::Cos(FIELD_HEADING_INITIAL);
	// static constexpr double GPS_HEADING_INITIAL_0 = 90;
	// static constexpr double GPS_HEADING_INITIAL = (GPS_HEADING_INITIAL_0 + FIELD_HEADING_INITIAL) >= 360 ? (GPS_HEADING_INITIAL_0 + FIELD_HEADING_INITIAL -  360) : (GPS_HEADING_INITIAL_0 + FIELD_HEADING_INITIAL);
	// static constexpr double GPSX_OFFSET = 0.0655;  // 138.8 65.5
	// static constexpr double GPSY_OFFSET = 0.1388;

	// 决赛淘汰赛gps参数
	static constexpr double FIELD_HEADING_INITIAL = 180;//比赛场地相对于战队场地，逆时针为+		(校内场地红方出发为0)
	static constexpr double ROPO_HEADING_INITIAL  = 250 - FIELD_HEADING_INITIAL; //对于战队场地X轴
	static constexpr int GPS_PORT = 6;
	static constexpr double GPSX_INITIAL_0 = -1.2 - 0.03765; // 需要校正旋转中心初始位置  dX=0.0062 dy=0.04382 
	static constexpr double GPSY_INITIAL_0 = -1.2 - 0.05090;
	static double GPSX_INITIAL =  GPSX_INITIAL_0 * RopoMath::Cos(FIELD_HEADING_INITIAL) + GPSY_INITIAL_0 * RopoMath::Sin(FIELD_HEADING_INITIAL);
	static double GPSY_INITIAL = -GPSX_INITIAL_0 * RopoMath::Sin(FIELD_HEADING_INITIAL) + GPSY_INITIAL_0 * RopoMath::Cos(FIELD_HEADING_INITIAL);
	static constexpr double GPS_HEADING_INITIAL_0 = 17.590;
	static constexpr double GPS_HEADING_INITIAL = (GPS_HEADING_INITIAL_0 + FIELD_HEADING_INITIAL) >= 360 ? (GPS_HEADING_INITIAL_0 + FIELD_HEADING_INITIAL -  360) : (GPS_HEADING_INITIAL_0 + FIELD_HEADING_INITIAL);
	static constexpr double GPSX_OFFSET = 0.0655;  // 138.8 65.5
	static constexpr double GPSY_OFFSET = 0.1388;

	//skill gps
	// static constexpr double FIELD_HEADING_INITIAL = 0;//比赛场地相对于战队场地，逆时针为+		(校内场地红方出发为0)
	// static constexpr double ROPO_HEADING_INITIAL  = 90 - FIELD_HEADING_INITIAL; //对于战队场地X轴
	// static constexpr int GPS_PORT = 6;
	// static constexpr double GPSX_INITIAL_0 = -0.6 - 0.08511 - 0.18 - 0.6; // 需要校正旋转中心初始位置
	// static constexpr double GPSY_INITIAL_0 = -1.2 - 0.13791 + 2.4 - 0.14;
	// static double GPSX_INITIAL =  GPSX_INITIAL_0 * RopoMath::Cos(FIELD_HEADING_INITIAL) + GPSY_INITIAL_0 * RopoMath::Sin(FIELD_HEADING_INITIAL);
	// static double GPSY_INITIAL = -GPSX_INITIAL_0 * RopoMath::Sin(FIELD_HEADING_INITIAL) + GPSY_INITIAL_0 * RopoMath::Cos(FIELD_HEADING_INITIAL);
	// static constexpr double GPS_HEADING_INITIAL_0 = 180;
	// static constexpr double GPS_HEADING_INITIAL = (GPS_HEADING_INITIAL_0 + FIELD_HEADING_INITIAL) >= 360 ? (GPS_HEADING_INITIAL_0 + FIELD_HEADING_INITIAL -  360) : (GPS_HEADING_INITIAL_0 + FIELD_HEADING_INITIAL);
	// static constexpr double GPSX_OFFSET = 0.0655;  // 138.8 65.5
	// static constexpr double GPSY_OFFSET = 0.1388;

	// 预赛淘汰赛gps参数
	// static constexpr double FIELD_HEADING_INITIAL = 180;//比赛场地相对于战队场地，逆时针为+		(校内场地红方出发为0)
	// static constexpr double ROPO_HEADING_INITIAL  = 90 - FIELD_HEADING_INITIAL; //对于战队场地X轴
	// static constexpr int GPS_PORT = 6;
	// static constexpr double GPSX_INITIAL_0 = -0.6 - 0.08511 - 0.18; // 需要校正旋转中心初始位置
	// static constexpr double GPSY_INITIAL_0 = -1.2 - 0.13791;
	// static double GPSX_INITIAL =  GPSX_INITIAL_0 * RopoMath::Cos(FIELD_HEADING_INITIAL) + GPSY_INITIAL_0 * RopoMath::Sin(FIELD_HEADING_INITIAL);
	// static double GPSY_INITIAL = -GPSX_INITIAL_0 * RopoMath::Sin(FIELD_HEADING_INITIAL) + GPSY_INITIAL_0 * RopoMath::Cos(FIELD_HEADING_INITIAL);
	// static constexpr double GPS_HEADING_INITIAL_0 = 180;
	// static constexpr double GPS_HEADING_INITIAL = (GPS_HEADING_INITIAL_0 + FIELD_HEADING_INITIAL) >= 360 ? (GPS_HEADING_INITIAL_0 + FIELD_HEADING_INITIAL -  360) : (GPS_HEADING_INITIAL_0 + FIELD_HEADING_INITIAL);
	// static constexpr double GPSX_OFFSET = 0.0655;  // 138.8 65.5
	// static constexpr double GPSY_OFFSET = 0.1388;

	//Chassis
	static constexpr double CHASSIS_PARAMETER = 0.314 ; 				//车体宽度
	static constexpr double WHEEL_RAD = 0.03394;						//轮子半径
	static constexpr double CHASSIS_RATIO = 47.0 / 43.0;	      		//减速比
	static constexpr double CHASSIS_SPPED_MAX= 600;					//底盘电机最高转速
	static constexpr double RAD_TO_RPM = CHASSIS_SPPED_MAX / 62.83;		//角速度转化为rpm

};

#endif // ROPO_PARAMETER_HPP