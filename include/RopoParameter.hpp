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

	static constexpr double FIELD_HEADING_INITIAL = 90;//比赛场地相对于战队场地，逆时针为+		(红270，蓝90)
	static constexpr double ROPO_HEADING_INITIAL  = 90 - FIELD_HEADING_INITIAL;//对于战队场地X轴
	
	static constexpr int GPS_PORT = 14;
	static constexpr double GPSX_INITIAL_0 = -0.91;
	static constexpr double GPSY_INITIAL_0 = -1.56;
	static double GPSX_INITIAL =  GPSX_INITIAL_0 * RopoMath::Cos(FIELD_HEADING_INITIAL)+GPSY_INITIAL_0 * RopoMath::Sin(FIELD_HEADING_INITIAL);
	static double GPSY_INITIAL =  -GPSX_INITIAL_0 * RopoMath::Sin(FIELD_HEADING_INITIAL)+GPSY_INITIAL_0 * RopoMath::Cos(FIELD_HEADING_INITIAL);
	
	static constexpr double GPS_HEADING_INITIAL_0 = 90;
	static constexpr double GPS_HEADING_INITIAL = (GPS_HEADING_INITIAL_0 + FIELD_HEADING_INITIAL) >= 360 ? (GPS_HEADING_INITIAL_0 + FIELD_HEADING_INITIAL -  360) : (GPS_HEADING_INITIAL_0 + FIELD_HEADING_INITIAL);
	
	static constexpr double GPSX_OFFSET =0.15;
	static constexpr double GPSY_OFFSET =0.12;////大车Y_OFFSET在原有基础上减少7cm

	//Chassis参数
	static constexpr double CHASSIS_PARAMETER = 0.336 ; 				//车体宽度
	static constexpr double WHEEL_RAD = 0.034925;						//轮子半径
	static constexpr double CHASSIS_RATIO = 54/44;	      		//减速比
	static constexpr double CHASSIS_SPPED_MAX= 600;					//底盘电机最高转速
	static constexpr double RAD_TO_RPM = CHASSIS_SPPED_MAX / 62.83;		//角速度转化为rpm

	//IMU
	static constexpr double IMU_K=1.0075;
	const int InertialPort = 12;

	//气动
	const char climb_pneumatic_port = 'B';
	const char left_wing_pneumatic_port  = 'D';
	const char right_wing_pneumatic_port  = 'C';
	const char lead_ball_pneumatic_port = 'E';


		const pros::motor_gearset_e_t ChassisGearset = pros::E_MOTOR_GEAR_BLUE;
		const int LeftChassisMotor1Port  	= 2;
		const int LeftChassisMotor2Port  	= 3;
		const int LeftChassisMotor3Port  	= 6;
        const int LeftChassisMotor4Port  	= 9;
		const int LeftChassisMotor5Port  	= 10;
		
		const int RightChassisMotor1Port	= 15;
		const int RightChassisMotor2Port	= 16;
		const int RightChassisMotor3Port	= 17;
		const int RightChassisMotor4Port	= 18;
		const int RightChassisMotor5Port	= 20;

		//Intaker
		const pros::motor_gearset_e_t IntakeGearset = pros::E_MOTOR_GEAR_GREEN;
		const int IntakeMotorPort		= 8;
		//自动直行方向锁定参数
		const double direct_lock_rotate=0.01;

		//kp,ki,pd,最大值限幅，最小值限幅，误差容限到达退出时间
		#define distance_pid 0.004,0.00001,0.000001,0.006,-0.006,0.02,0.25
		#define rotate_pid 0.00006 ,0.0000005  ,0.00000001 ,0.006,-0.006,2,0.25
};

#endif // ROPO_PARAMETER_HPP