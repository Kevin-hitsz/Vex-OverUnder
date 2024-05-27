#include "main.h"
#include "RopoApi.hpp"
#include "RopoChassis.hpp"
#include "RopoController.hpp"
#include "RopoDevice.hpp"
#include "RopoMath/Misc.hpp"
#include "pros/llemu.hpp"
#include "pros/misc.h"
#include "pros/rtos.hpp"
#include <cmath>

namespace ControllerModule{
	bool ClimberFlag = false;
	bool ExternLeftFlag = false;
	bool ExternRightFlag = false;
	bool ShooterPneumaticFlag = false;
}

void initialize() {
	RopoDevice::DeviceIni();
	pros::lcd::initialize();
	pros::delay(10);
}

namespace RopoFunction{
	
	int InitialTime = 0;
	bool TimeFlag = true;

	void ShooterPneumatic(){
		if(ControllerModule::ShooterPneumaticFlag == false){
			RopoDevice::ThreeWire::ShooterPneumatic.set_value(true);
			ControllerModule::ShooterPneumaticFlag = true;
		}
		else if(ControllerModule::ShooterPneumaticFlag == true){
			RopoDevice::ThreeWire::ShooterPneumatic.set_value(false);
			ControllerModule::ShooterPneumaticFlag = false;
		}
	}


	void Intake(){
		RopoDevice::Motors::IntakeMotor.move_velocity(-600);
	}
	void Outtake(){
		RopoDevice::Motors::IntakeMotor.move_velocity(600);
	}
	void StopIn(){
		RopoDevice::Motors::IntakeMotor.move_velocity(0);
	}

	void ExternLeft(){
		if(ControllerModule::ExternLeftFlag == false){
			RopoDevice::ThreeWire::LExternPneumatic.set_value(true);
			ControllerModule::ExternLeftFlag = true;
		}
		else if(ControllerModule::ExternLeftFlag == true){
			RopoDevice::ThreeWire::LExternPneumatic.set_value(false);
			ControllerModule::ExternLeftFlag = false;
		}
	}

	void ExternRight(){
		if(ControllerModule::ExternRightFlag == false){
			RopoDevice::ThreeWire::RExternPneumatic.set_value(true);
			ControllerModule::ExternRightFlag = true;
		}
		else if(ControllerModule::ExternRightFlag == true){
			RopoDevice::ThreeWire::RExternPneumatic.set_value(false);
			ControllerModule::ExternRightFlag = false;
		}
	}

	
	void IntakerUp(){
		RopoDevice::ThreeWire::IntakerPneumatic.set_value(true);
	}
	void IntakerDown(){
		RopoDevice::ThreeWire::IntakerPneumatic.set_value(false);
	}


	void ShooterInit(){
		RopoDevice::Motors::LShooterMotor.move_velocity(-30);
		RopoDevice::Motors::RShooterMotor.move_velocity(30);
	}
	void ShooterStopInit(){
		RopoDevice::Motors::LShooterMotor.move_velocity(0);
		RopoDevice::Motors::RShooterMotor.move_velocity(0);
	}
	bool DetectLoader(){
		if(RopoDevice::Motors::LShooterMotor.get_torque() < 0.2 && RopoDevice::Motors::RShooterMotor.get_torque() < 0.2) return false;
		else return true;
	}
	void ReLoad(){
		RopoDevice::Motors::LShooterMotor.move_voltage(-11000);
		RopoDevice::Motors::RShooterMotor.move_voltage(11000);
		pros::delay(500);
		while (RopoDevice::Motors::RShooterMotor.get_actual_velocity() > 1) pros::delay(5);
		ShooterStopInit();
	}
	void Shoot(){
		RopoDevice::Motors::LShooterMotor.move_voltage(10000);
		RopoDevice::Motors::RShooterMotor.move_voltage(-10000);
		pros::delay(500);
		while (RopoDevice::Motors::LShooterMotor.get_actual_velocity() > 1) pros::delay(5);
		ShooterStopInit();
	}
	void Climber(){
		if(ControllerModule::ClimberFlag == false){
			ReLoad();
			RopoDevice::ThreeWire::ClimberPneumatic.set_value(true);
			ControllerModule::ClimberFlag = true;
		}
		else if(ControllerModule::ClimberFlag == true){
			RopoDevice::ThreeWire::ClimberPneumatic.set_value(false);
			ControllerModule::ClimberFlag = false;
			ShooterPneumatic();
			Shoot();
		}
	}


	void shootandsweep(int n){

		float x,y,theta;
		x = RopoDevice::Chassis.GetX();
		y = RopoDevice::Chassis.GetY();
		theta = RopoDevice::Chassis.GetTheta();
		ReLoad();
		ShooterPneumatic();
		ExternLeft();

		for(int i = 1; i<=n; i++)
		{
			pros::delay(1000);
			Shoot();
			pros::delay(300);
			RopoDevice::Chassis.AutoSetAimStatus(0, 0, 8);
			pros::delay(800);
			RopoDevice::Chassis.AutoSetAimStatus(0, 0, 0);
			pros::delay(300);
			RopoDevice::Chassis.AutoSetPosition(x, y, theta, 2000);
			while (RopoDevice::Chassis.IfPosition_OK() == false) {pros::delay(20);}
			ReLoad();
		}
		
		ShooterPneumatic();
		Shoot();
		ExternLeft();


	}

	void closemove(float x, float y, float theta,int max_time)
	{
		RopoDevice::Chassis.AutoSetPosition(x, y, theta, max_time);
		while(RopoDevice::Chassis.IfPosition_OK() == false)
		{
			pros::delay(20);
		}
	}
	void openmove(float Vx, float Vy, float Vw, int move_time){
		RopoDevice::Chassis.AutoSetAimStatus(Vx, Vy, Vw);
		pros::delay(move_time);
	}

	void TurnAround1(){
		openmove(0, 0, 8, 600);
		openmove(0, 0, 0, 400);
	}
	void TurnAround2(){
		openmove(0, 0,-8, 600);
		openmove(0, 0, 0, 400);
	}
	void MoveToZero(){
		RopoDevice::Chassis.AutoSetPosition(0.0, 0.0, 0.0, 4000);
	}


	/*void Hit(){
		RopoDevice::Motors::HitterMotor.move_voltage(-12000);
		pros::delay(200);
		while (std::abs(RopoDevice::Motors::HitterMotor.get_actual_velocity()) > 5) pros::delay(5);
		RopoDevice::Motors::HitterMotor.move_voltage(0);
	}
	void HitterReset(){
		RopoDevice::Motors::HitterMotor.move_voltage(12000);
		pros::delay(200);
		while (RopoDevice::Motors::HitterMotor.get_actual_velocity() > 5) pros::delay(5);
		RopoDevice::Motors::HitterMotor.move_voltage(0);
	}*/

	void Test(){
		RopoDevice::Chassis.AutoSetPosition(0,0,0,2000);
	}
	//bool ControlMode = false;
	
	void ChangeControlMode(){
		//ControlMode = !ControlMode;
		RopoDevice::Chassis.ChangeControlMode();
	}
	void Import(){

	}


	void autonomous_1(){
		double x,y,angle = 0;
		RopoDevice::Chassis.SetInitialAngle(-RopoMath::Pi / 4);

		/*	step_1 拨出联队球推入球网	*/
		ExternRight();												   // 展开侧翼							
		pros::delay(200);

		openmove(-1,0,-5,600);					 //拨出联队球
		openmove(0,0,0,500);

		ExternRight();												   // 收起侧翼 防止被卡
		pros::delay(200);

		closemove(-0.38, 0.09, -45, 800); 		 // 对准推球位置 车尾对准球门
				
		openmove(-1.57,0,0,800);					 // 向后冲推入联队球
		openmove(0,0,0,50);
		/*	step_1 end	*/ 


		/*  step_2 到中间撞球并返回导球点  */
		openmove(1,0,5,500);						// 稍微离开球门并将车头朝外 车头朝向大致与三角区横杆呈30度角	
		openmove(0,0,0,50);

		closemove(0.05,0.65,-108.9, 800);        // 准备前往场地中央 p2.1
		closemove( -0.17, 0.80, -125.7, 300);				// 中间过渡点 大概率无法走到 主要目的是使车尾在移动过程中逐渐对准对方球门方向
		closemove( -0.33, 0.92, -127.8, 300);			    // 中间过渡点 同上
		closemove( -0.47, 1.02, -132.5, 1000);				// 准备推球点 p2.2
		openmove(0,0,0,50);

		ExternRight();																		// 展开翅膀
		pros::delay(200);
			
		closemove( 0.43, 1.91, -132.5, 1200);	// 推球动作 目标点设定在横杆后

		openmove(0,0,0,50);
		ExternRight();												// 收起翅膀

		closemove(-0.04,1.06,86.179, 800);		// 准备前往抛球 p2.3
		closemove(0.03, 0.48, 85.65, 800);		// 路径中间点	p2.4	
		closemove(0.12, 0.00, 72.12, 1000);		// 抛球位置
		openmove(0,0,0,50);
		/*	step_2 end	*/ 


		/*  step_3 抛球x6 导球x5  */
		ReLoad();													  // 放下抛投框
		pros::delay(200);

		ShooterPneumatic();											  // 弹出限位
		pros::delay(200);

		for(int i = 1; i <= 5; i++){
			Shoot();												  // 抛球
			if(i != 5) pros::delay(800);				  //  抛起第5球后会快速放下抛投框
			if(i == 5) pros::delay(200);
			ReLoad();												 // 放框
		} 

		ShooterPneumatic();											// 收回限位
		pros::delay(100);
		Shoot();													// 收起抛投

		closemove(0.10, 0.04, 19.52, 1000);		// 导球位置

		ExternRight();												// 展开翅膀
		pros::delay(200);

		for (int i = 1; i <= 6; i++) {

			if(i == 4)												// 在导第四球前校准一次位置
			{
				closemove(0.10, 0.04, 19.52, 1000);
				openmove(0,0,0,200);
			}

			openmove(0, 0, 8, 300);
			
			if(i != 6){												// 在击出最后一球后不会回拉 准备前往通道推球
				openmove(0,0,0,300);
				openmove(0, 0, -8, 300);
				openmove(0,0,0,600);
			}	
		}
		openmove(0,0,6,200);					// 快速旋转对准通道
		/*	step_3 end	*/ 


		/*  step_4 将导出的球推入球网  */


		//closemove(0.10, 0.04, 19.52, 1000);						//  仅step_4单步测试时使用
		//ExternRight();											//  仅step_4单步测试时使用

		closemove(0.23, 0.17, -140.32, 500);// 准备进入通道	p4.1
		closemove(0.51, 0.19, -135, 800);	// 通道起始位置 p4.2
		ExternRight();											// 收起侧翼  

		openmove(-1.57, 0,0,200);
		openmove(-0.5, 0,0,400);
				
		
		openmove(-0.5, 0,0,400);			  // 缓慢通过通道前半段

		closemove(1.07, 0.76, -135, 600);	// 矫正位置 p4.3
		ExternRight();											  // 打开侧翼

		openmove(-1.57, 0,0,200);
		openmove(-0.5, 0,0,400);
		/*x = RopoDevice::Chassis.GetX();
		y = RopoDevice::Chassis.GetY();
		closemove(x, y, -135, 500);*/
		openmove(-0.5, 0,0,400);				//行使至通道出口


		closemove(1.64, 1.33, -135, 1500);	// 通道出口位置矫正 车头对齐地垫边缘 p4.4
		//openmove(-0.5, 0,0.8,1000);							  // 行使至三角区横杆边

		openmove(-0.5, 0,0.4,600);			                  // 大致与三角区横杆平行
		closemove(1.89, 2.07, -92.52, 800);	                      // 行使至三角区边缘 接触边界 p4.5

		closemove(1.67, 1.99, -92.00, 800);	// 调整点 使车侧方移动，防止被与墙壁紧贴的棕球卡死 p4.6

		closemove(1.79, 2.20, -45, 1500);		// 预备推球位置

		ExternRight();
		pros::delay(100);						// 收起翅膀 防止入网被卡

		openmove(-1.57, 0,0,1000);				// 第一次推球
		openmove(0,0,0,100);

		openmove(1, 0,0,500);
		openmove(0,0,0,100);
		closemove(1.79, 2.20, -45, 800);		// 预备推球位置

		openmove(-1.57, 0,0,1000);			// 第二次推球
		openmove(0,0,0,100);

		openmove(1, 0,0,300);				// 驶离球门区域，车头大致对准碰杆方向
		openmove(0, 0,-8,400);
		openmove(0,0,0,50);
		Intake();							// 打开吃球装置
		/*	step_4 end	*/ 
		

		/*  step_5 碰杆  */
		closemove(1.70, 1.47, -111.10, 1000);	// 中间点 p5.1
		closemove(1.12, 0.74, -135, 1500);		// 碰杆点
		ReLoad();								// 碰杆
		StopIn();								// 停止吃球
		/*	step_5 end	*/

		RopoDevice::Chassis.Operator();
		RopoDevice::Chassis.ChangeControlMode();
	}
}




/*void autonomous_A1(){
	float x,y,theta;
	RopoFunction::ReLoad();
	//RopoFunction::HitterReset();

	pros::delay(1000);


	RopoFunction::Intake();
 	RopoDevice::Chassis.AutoSetPosition(0,5,0,100);
	RopoFunction::StopIn();
	RopoFunction::ExternRight();
	RopoDevice::Chassis.AutoSetPosition(0,-40,0,100);
	RopoDevice::Chassis.AutoSetPosition(0,-40,45,100);
	RopoFunction::ExternLeft();
	RopoDevice::Chassis.AutoSetPosition(5,-45,45,100);
	RopoDevice::Chassis.AutoSetPosition(5,-45,90,100);
	RopoDevice::Chassis.AutoSetAimStatus(0, -20, 0);
	RopoDevice::Chassis.AutoSetAimStatus(0, 10, 0);
	RopoDevice::Chassis.AutoSetAimStatus(0, -20, 0);
	RopoDevice::Chassis.AutoSetAimStatus(0, 10, 0);

	x = RopoDevice::Chassis.GetX();
	y = RopoDevice::Chassis.GetY();
	RopoDevice::Chassis.AutoSetPosition(x,y,-90,100);
	
	RopoDevice::Chassis.AutoSetAimStatus(0, 20, 0);
}*/



void autonomous_Wisco(){
	RopoDevice::Chassis.SetInitialAngle(RopoMath::Pi / 2);
	using namespace RopoFunction;
	Intake();
	openmove(1.57, 0.0, 0.0, 500);             //出黑杆范围
	closemove(0.78, -0.04, -36.92, 650);       //到吃第一个球点位
	openmove(0.47, 0.02, 0.0, 500);            //往前慢点走，吃第一个球


	/*直接推球方案（容易被翅膀卡球）**************/
	/* closemove(1.03, 0.01, 90.0, 2600);        //到推球点位
	ExternRight();                                                  //开翅膀
	openmove(-1.57, 0.0, 0.0, 1200);           //往前推
	StopIn();
	openmove( 0.80, 0.0, 0.0, 500);            //回来一点，准备再推一次
	openmove( -1.57, 0.0, 0.0, 1500);          //再推一次
	openmove( 0.0, 0.0, 0.0, 300);            
	ExternRight();            */                                       //收翅膀
	
	
	/*扫球推球方案（相对稳妥效率高）**************/
	closemove(1.00, 0.01, 141.5, 1300);        //到扫球点位
	ExternRight();                                                   //开翅膀
	closemove(1.14, -0.2, 53.4, 700);          //扫球
	ExternRight();                                                  //关翅膀
	closemove(1.13, -0.2, 0.0, 1000);           //到推球点位
	closemove(1.13, -1.5, 0.0, 800);            //推球
	pros::delay(2800);
	/****************************************/


	pros::delay(1500);                         
	closemove(0.47, 0.00, 169.58, 4000);       //到中间过渡一下，准备导入
	closemove(0.01, 0.29, -132.52, 2800);      //进入导入点位
	int load_number = 9;
	ExternRight();                                                      //开
	pros::delay(1000);
	for (int i = 1; i <= load_number; i++) {                          //导入n次球
		if(i % 3 == 0)
		{                                                             //导三次校准一次
		closemove(0.01, 0.29, -132.52, 500);
		}
		openmove(0.0, 0.0,  8.0, 300);           //开导
		openmove(0.0, 0.0,  0.0, 150);
		//ExternRight();
		if(i == load_number){break;}
		openmove(0.0, 0.0, -8.0, 280);            //回
		openmove(0.0, 0.0,  0.0, 600);
		//ExternRight();
		}        
	ExternRight();                                                     //关
	pros::delay(1500);
	closemove(-0.18, -0.09, 62.68, 1000);        //刚要进入通道
	closemove(-0.18  , -0.55, 90.0, 800);        //进入通道
	openmove( -1.50, 0.0, 0.0, 400);             //通过通道
	ExternRight();                                                   //开
	closemove(-0.18, -1.27, 90.0, 600);         //即将出通道，屁股与黑杆齐平
	closemove(-0.19, -1.75, 90.0, 700);        //屁股快到导入杆的地垫了，还差8厘米左右
	closemove(-0.11, -1.97, 130.6, 700);       //进入导入杆所在地垫，车体开始略微倾斜
	openmove(-0.5, 0.0, 0.0, 600);             //直线往前推一点
	closemove(0.22, -2.31, 135.0, 500);        //屁股与导入杆末端齐平，准备转身
	openmove(-0.4, -0.4, 0.0, 300);            //往前调整姿态，准备转身
	closemove(0.45, -2.42, 174.43, 800);        //转身，到达推球点
	openmove(0.3, 0.0, 0.0, 300);                //往前挪一点，获得更大推球速度
	openmove(-1.57, 0.0, 0.0, 1000);             //推球
	openmove( 1.20, 0.0, 0.0, 500);             //往前退一点，准备吐球
	TurnAround1();                                                    //转身
	Outtake();                                                        //吐球  
	pros::delay(700);    
	TurnAround2();                                                    //转回来
	ExternRight();                                                    //关
	closemove(0.45, -2.42, 174.43, 700);        //第二次到达推球点
	openmove(0.3, 0.0, 0.0, 300);                //往前挪一点，获得更大推球速度
	openmove( -1.57, 0.0, 0.0, 1000);             //第二次撞球门 
	Intake();
	closemove(0.20, -2.28, 132.09, 750);       //屁股与导入杆末端齐平，右后轮刚跨入球门前一个地垫
	closemove(-0.15, -1.90, 99.0, 680);        //准备进入通道
	closemove(-0.16, -1.27, 90.0, 680);        //进入通道
	closemove(-0.15, -0.09, 62.68, 750);       //即将回到战略点
	closemove(0.0, 0.0, -90.0, 1000);           //到达战略点
	StopIn();
	RopoDevice::Chassis.ChangeControlMode();
	RopoDevice::Chassis.Operator();
	
}



void disabled() {}

void competition_initialize() {}

void skill() {    // 世锦赛
	using namespace RopoFunction;
	InitialTime = pros::millis(); 
	TimeFlag = false;
	RopoDevice::Chassis.SetInitialAngle(0.0);
	openmove(0.0, 1.5, 0.0, 700);
	closemove(0.29, 0.39, -29.984, 1800);
	const int throw_number = 2;
	ReLoad();
	ShooterPneumatic();
	for(int i = 1; i <= throw_number - 1; i++){
		Shoot();
		if(i != throw_number - 1) pros::delay(800);
		ReLoad();
	}
	ShooterPneumatic();
	pros::delay(400);
	Shoot();
	closemove(0.26, 0.26, -94.52, 1800);
	int load_number = 5;
	ExternRight();                                                    //开翅膀
	for (int i = 1; i <= load_number; i++) {                          //导入n次球
		if(i % 3 == 0)
		{                                                             //导三次校准一次
		closemove(0.26, 0.26, -94.52, 500);
		}
		openmove(0.0, 0.0,  -8.0, 300);           //开导
		openmove(0.0, 0.0,  0.0, 150);
		if(i == load_number){break;}
		openmove(0.0, 0.0, 8.0, 280);           //回
		openmove(0.0, 0.0,  0.0, 600);
	}
	ExternRight();                                                    //收翅膀
	closemove(0.54, 0.45, -147.51, 600);         //准备进通道
	closemove(0.82, 0.48, -179.9, 1500);          //进通道
	openmove(-1.75, 0.0, 0.0, 800);             //往后猛退
	closemove(2.16, 0.51, 180.0, 500);          
	closemove(2.51, 0.53, 180.0, 1300);        //出通道
	closemove(2.51, 0.53, 140.67, 800);        //屁股进入导入杆所在区域，准备开翅膀
	ExternRight();                                                    //开翅膀
	openmove(-0.4, 0.0, 0.0, 500);               //慢慢往前走一点
	ExternRight();                                                    //收翅膀
	closemove(2.92, 0.20, 135.0, 600);           //屁股与导入杆后端齐平
	openmove(0.4, 0.4, 0.0, 250);                //往前调整位置，准备转身
	closemove(3.08, -0.08, 87.0, 900);          //转身准备推球
	openmove(0.3, 0.0, 0.0, 300);                //往前蓄力，准备推球 
	openmove(-1.57, 0.0, 0.0, 1000);               //推球
	openmove(0.8, 0.0, 0.0, 400);                 //蓄力准备再推
	openmove(-1.57, 0.0, 0.0, 900);               //再推
	openmove(0.8, 0.0, 0.0, 700);                 //往前走一点，避免旋转时卡住
	closemove(2.41, -0.12, 45.3117, 1200);        //到达圆弧推点
	ExternRight();                                                     //开翅膀
	openmove(-0.7, 0.0, 1.2,3500);                //圆弧形推进网
	closemove(2.07, -0.46, 83.0, 1200);
	ExternRight();                                                    //收翅膀 
	closemove(2.51, 0.16, 155.0, 1000);           //爬杆过渡点1
	closemove(2.34, 0.50, 179.0, 1000);           //爬杆过度点2
	closemove(2.00, 0.50, 180.0, 600);           //爬杆点
	Climber();
	openmove(1.57, 0.0, 0.0, 1500);
	Climber();
	TimeFlag = true;
	RopoDevice::Chassis.Operator();
}

void skill_new(){   // 机创赛版本

	using namespace RopoFunction;

	/* step_1 导入22球（导球点即为起始点） */
		/* 重复一次抛投初始化动作 */  
	ReLoad();
	ShooterPneumatic();
	pros::delay(800);
	//Shoot();
		// 测试：是否需要在此处加入delay
	//ReLoad();
		/* 正式导球部分 */
	for(int i = 1; i <= 24; i++){
		if(i % 8 == 0)closemove(0,0,0,500); // 车在抛球的时候会慢慢往前移动，不校正位置可能会导致抛球机构打到三角区横杆
		Shoot();
		ReLoad();
		pros::delay(800);
	} 	// 测试:连续二十五次抛投动作是否引起过热？
	  	// 使用单向阀导球更适合还是将球放在抛投框上更适合？
	ShooterPneumatic();
	pros::delay(200);
	Shoot();
	/* step_1 end */

	/* step_2 将接触的联队球与放置在球门的联队球推入球门 */
	/* step_2 end*/

	/* step_3 前往另一个导球点并完成导球 */
	/* step_3 end */

	/* step_4 爬杆 */
	/* step_4 end */

}


void autonomous(){
	//autonomous_Wisco();
	RopoFunction::autonomous_1();
	//skill();
}

void opcontrol() {
	pros::Controller MasterController(pros::E_CONTROLLER_MASTER);
	RopoController::ButtonTaskLine ButtonDetectLine(MasterController);

	FloatType VelocityMax = 1.57;
	FloatType RopoWcLimit = 8;
	
	RopoController::AxisValueCast XVelocityInput(MasterController,pros::E_CONTROLLER_ANALOG_LEFT_Y,RopoController::Ln);
	RopoController::AxisValueCast YVelocityInput(MasterController,pros::E_CONTROLLER_ANALOG_LEFT_X,RopoController::Ln);
	RopoController::AxisValueCast WVelocityInput(MasterController,pros::E_CONTROLLER_ANALOG_RIGHT_X,RopoController::Linear);
	Vector Velocity(RopoMath::ColumnVector,2),ResVelocity;
	MasterController.clear();


	ButtonDetectLine.AddButtonDetect(pros::E_CONTROLLER_DIGITAL_R1, RopoController::Rising, RopoFunction::Intake);
	ButtonDetectLine.AddButtonDetect(pros::E_CONTROLLER_DIGITAL_R1, RopoController::Falling, RopoFunction::StopIn);

	ButtonDetectLine.AddButtonDetect(pros::E_CONTROLLER_DIGITAL_R2, RopoController::Rising, RopoFunction::Outtake);
	ButtonDetectLine.AddButtonDetect(pros::E_CONTROLLER_DIGITAL_R2, RopoController::Falling, RopoFunction::StopIn);
	

	ButtonDetectLine.AddButtonDetect(pros::E_CONTROLLER_DIGITAL_L2, RopoController::Rising, RopoFunction::ExternLeft);
	ButtonDetectLine.AddButtonDetect(pros::E_CONTROLLER_DIGITAL_L1, RopoController::Rising, RopoFunction::ExternRight);

	ButtonDetectLine.AddButtonDetect(pros::E_CONTROLLER_DIGITAL_UP, RopoController::Rising, RopoFunction::Climber);

	//ButtonDetectLine.AddButtonDetect(pros::E_CONTROLLER_DIGITAL_RIGHT, RopoController::Rising, Test);
	ButtonDetectLine.AddButtonDetect(pros::E_CONTROLLER_DIGITAL_B, RopoController::Rising, RopoFunction::ReLoad);
	ButtonDetectLine.AddButtonDetect(pros::E_CONTROLLER_DIGITAL_X, RopoController::Rising, RopoFunction::Shoot);

	ButtonDetectLine.AddButtonDetect(pros::E_CONTROLLER_DIGITAL_DOWN, RopoController::Rising, RopoFunction::ShooterPneumatic);
	ButtonDetectLine.AddButtonDetect(pros::E_CONTROLLER_DIGITAL_Y, RopoController::Rising, skill_new);

	ButtonDetectLine.AddButtonDetect(pros::E_CONTROLLER_DIGITAL_LEFT, RopoController::Rising, RopoFunction::ChangeControlMode);
	ButtonDetectLine.Enable();
	RopoDevice::Chassis.Operator();
	float Second = 0.0;
	while (true) {
		FloatType XInput =  VelocityMax * XVelocityInput.GetAxisValue();
		FloatType YInput =  VelocityMax * YVelocityInput.GetAxisValue();
		FloatType WInput = -RopoWcLimit * WVelocityInput.GetAxisValue();
		if(!RopoFunction::TimeFlag) Second = (pros::millis() - RopoFunction::InitialTime) / 1000;
		if(RopoDevice::Chassis.IsOpcontrol()) RopoDevice::Chassis.OpSetAimStatus(XInput, YInput, WInput);
		MasterController.print(0,0,"%.2f, %.2f, %.2f", RopoDevice::Sensors::Encoder.GetPosX()/1000, RopoDevice::Sensors::Encoder.GetPosY()/1000, RopoDevice::Chassis.GetTheta() * 180 / RopoMath::Pi);
		MasterController.print(1, 0, "%.2f", Second);
		pros::delay(5);
	}
}