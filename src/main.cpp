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
		RopoDevice::Motors::LShooterMotor.move_voltage(-12000);
		RopoDevice::Motors::RShooterMotor.move_voltage(12000);
		pros::delay(500);
		while (RopoDevice::Motors::RShooterMotor.get_actual_velocity() > 1) pros::delay(5);
		ShooterStopInit();
	}
	void Shoot(){
		RopoDevice::Motors::LShooterMotor.move_voltage(12000);
		RopoDevice::Motors::RShooterMotor.move_voltage(-12000);
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

	void StraightMove_x(double vx,int max_time){
		int counter = 0;
		double Angle = RopoDevice::Chassis.GetTheta();
		int error = 3;
		while (counter < max_time) {
			if(fabs(RopoDevice::Chassis.GetTheta() - Angle) >error){
				if (RopoDevice::Chassis.GetTheta() - Angle < -error || RopoDevice::Chassis.GetTheta() - Angle > 180) RopoDevice::Chassis.AutoSetAimStatus(vx, 0, 0.5);
				if (RopoDevice::Chassis.GetTheta() - Angle >  error || RopoDevice::Chassis.GetTheta() - Angle <-180) RopoDevice::Chassis.AutoSetAimStatus(vx, 0, -0.5);
			}
			else RopoDevice::Chassis.AutoSetAimStatus(vx, 0, 0);
			pros::delay(50);
			counter = counter + 50;
		}
	}

	void autonomous_1(){
	RopoDevice::Chassis.SetInitialAngle(-RopoMath::Pi / 4);

	/*	step_1 拨出联队球推入球网	*/
	RopoFunction::ExternRight();							
	pros::delay(700);

	RopoDevice::Chassis.AutoSetAimStatus(-1, 0, -6);		 //拨出联队球
	pros::delay(600);
	RopoDevice::Chassis.AutoSetAimStatus(0, 0, 0);
	pros::delay(500);
	RopoFunction::ExternRight();

	RopoFunction::closemove(-0.38, 0.09, -45, 800); 		 // 对准推球位置

	RopoDevice::Chassis.AutoSetAimStatus(-1.57, 0, 0);		   		// 推入联队球
	pros::delay(800);
	RopoDevice::Chassis.AutoSetAimStatus(0, 0, 0);
	/*	step_1 end	*/ 


	/*  step_2 到中间撞球并返回导球点  */
	pros::delay(50);											   // 前往场地中央推球位置
	RopoDevice::Chassis.AutoSetAimStatus(1, 0, 5);
	pros::delay(500);
	RopoDevice::Chassis.AutoSetAimStatus(0, 0, 0);
	pros::delay(50);
	RopoFunction::closemove(0.05,0.65,-108.9, 800);
	RopoFunction::closemove( -0.17, 0.80, -125.7, 300);
	RopoFunction::closemove( -0.33, 0.92, -127.8, 300);
	RopoFunction::closemove( -0.47, 1.02, -132.5, 1000);
	RopoDevice::Chassis.AutoSetAimStatus(0, 0, 0);

	RopoFunction::ExternRight();
	pros::delay(200);
		
	RopoFunction::closemove( 0.43, 1.91, -132.5, 1200);	// 推球动作

	RopoDevice::Chassis.AutoSetAimStatus(0, 0, 0);
	pros::delay(50);
	RopoFunction::ExternRight();

	RopoFunction::closemove(-0.04,1.06,86.179, 800);		// 前往导球位置
	RopoFunction::closemove(0.03, 0.48, 85.65, 800);
	RopoFunction::closemove(0.12, 0.00, 72.12, 1000);		// 导球位置
	RopoDevice::Chassis.AutoSetAimStatus(0, 0, 0);
	pros::delay(50);
	/*	step_2 end	*/ 


	/*  step_3 抛球x6 导球x5  */
	RopoFunction::ReLoad();
	pros::delay(200);
	RopoFunction::ShooterPneumatic();
	pros::delay(200);

	for(int i = 1; i <= 5; i++){
		RopoFunction::Shoot();
		if(i != 5) pros::delay(800);
		RopoFunction::ReLoad();
	} 

	RopoFunction::ShooterPneumatic();
	pros::delay(100);
	RopoFunction::Shoot();

	RopoFunction::closemove(0.10, 0.04, 19.52, 1000);
	RopoFunction::ExternRight();
	pros::delay(200);
	for (int i = 1; i <= 6; i++) {

		if(i%4 == 0)
		{
			RopoFunction::closemove(0.10, 0.04, 19.52, 1000);
			RopoDevice::Chassis.AutoSetAimStatus(0, 0, 0);
			pros::delay(200);
		}

		RopoDevice::Chassis.AutoSetAimStatus(0, 0, 8);
		pros::delay(400);
		
		if(i != 6){
			RopoDevice::Chassis.AutoSetAimStatus(0, 0, 0);
			pros::delay(300);
			RopoDevice::Chassis.AutoSetAimStatus(0, 0, -8);
			pros::delay(400);
			RopoDevice::Chassis.AutoSetAimStatus(0, 0, 0);
			pros::delay(600);
		}	
	}
	RopoDevice::Chassis.AutoSetAimStatus(0, 0, 0);
	pros::delay(100);
	/*	step_3 end	*/ 


	/*  step_4 将导出的球推入球网  */

	//closemove(0.10, 0.04, 19.52, 1000);	//  仅step4单步测试时使用
	//ExternRight();	//  仅step4单步测试时使用

	closemove(0.23, 0.17, -160.32, 800);// 准备进入通道
	closemove(0.51, 0.19, -135, 1500);	// 通道起始位置

	openmove(-1.57, 0,0,200);
	openmove(-0.5, 0,0,400);				
	ExternRight();
	openmove(-0.5, 0,0,400);

	closemove(1.07, 0.76, -135, 600);	// 矫正位置并打开侧翼
	ExternRight();

	openmove(-1.57, 0,0,200);
	openmove(-0.5, 0,0,800);				//行使至通道出口


	closemove(1.64, 1.33, -135, 800);	// 通道出口位置矫正 车头对齐地垫边缘
	openmove(-0.5, 0,0.8,1000);			// 行使至三角区横杆边

	openmove(-0.5, 0,0.4,600);			// 大致与三角区横杆平行
	closemove(1.89, 2.07, -92.52, 800);	// 行使至三角区边缘 接触边界

	closemove(1.67, 1.99, -92.00, 800);
	closemove(1.79, 2.20, -45, 800);		// 准备推球位置
	ExternRight();
	pros::delay(100);							   // 防止入网被卡

	openmove(-1.57, 0,0,1200);			// 第一次推球
	RopoDevice::Chassis.AutoSetAimStatus(0, 0, 0);
	pros::delay(100);

	openmove(1, 0,0,500);
	RopoDevice::Chassis.AutoSetAimStatus(0, 0, 0);
	pros::delay(100);
	closemove(1.79, 2.20, -45, 800);		// 准备推球位置

	openmove(-1.57, 0,0,1200);			// 第二次推球
	RopoDevice::Chassis.AutoSetAimStatus(0, 0, 0);
	pros::delay(100);

	openmove(1, 0,0,300);				// 驶离球门区域，车头大致对准碰杆方向
	openmove(0, 0,-8,400);
	RopoDevice::Chassis.AutoSetAimStatus(0, 0, 0);
	Intake();
	/*	step_4 end	*/ 
	

	/*  step_5 碰杆  */
	RopoFunction::closemove(1.70, 1.47, -111.10, 1000);
	RopoFunction::closemove(1.12, 0.74, -135, 1500);
	RopoFunction::ReLoad();
	StopIn();
	/*	step_5 end	*/

	RopoDevice::Chassis.Operator();
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
	closemove(1.03, 0.01, 90.0, 2600);        //到推球点位
	ExternRight();                                                  //开翅膀
	openmove(-1.57, 0.0, 0.0, 1200);           //往前推
	StopIn();
	openmove( 0.80, 0.0, 0.0, 500);            //回来一点，准备再推一次
	openmove( -1.57, 0.0, 0.0, 1500);          //再推一次
	openmove( 0.0, 0.0, 0.0, 300);            
	ExternRight();                                                  //收翅膀
	pros::delay(500);                         
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
	pros::delay(2500);
	closemove(-0.18, -0.09, 62.68, 1000);        //刚要进入通道
	closemove(-0.18  , -0.55, 90.0, 800);        //进入通道
	openmove( -1.50, 0.0, 0.0, 500);             //通过通道
	closemove(-0.18, -1.27, 90.0, 600);         //即将出通道，屁股与黑杆齐平
	ExternRight();                                                   //开
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
	//第一次参数
	/* openmove(-0.6, 0.0, 0.0, 700);                //刚出通道，准备将球推进网
	closemove(0.18, -2.30, 142.09, 1000);         //屁股与导入杆末端齐平，右后轮刚跨入球门前一个地垫
	closemove(0.3, -2.30, -180.00, 1000);         //闭环撞入球门
	openmove( -1.57, 0.0, 0.0, 1000);             //往球门猛猛撞
	ExternRight();
	openmove( 1.20, 0.0, 0.0, 500);              //往前退一点，准备再撞一次
	TurnAround1();                                                    //转身
	Outtake();                                                        //吐球      
	TurnAround2();                                                    //转回来
	ExternRight();
	closemove(0.3, -2.30, -180.00, 1000);         //闭环准备第二次撞入球门
	
	openmove( -1.50, 0.0, 0.0, 1000);             //第二次撞球门
	ExternRight(); */
	Intake();
	closemove(0.20, -2.28, 132.09, 800);       //屁股与导入杆末端齐平，右后轮刚跨入球门前一个地垫
	closemove(-0.15, -1.90, 99.0, 800);        //准备进入通道
	closemove(-0.16, -1.27, 90.0, 800);        //进入通道
	closemove(-0.15, -0.09, 62.68, 1000);       //即将回到战略点
	closemove(0.0, 0.0, -90.0, 1000);           //到达战略点
	StopIn();
	RopoDevice::Chassis.ChangeControlMode();
	RopoDevice::Chassis.Operator();
	
}



void disabled() {}

void competition_initialize() {}

void skill() {
  RopoDevice::Chassis.SetInitialAngle(0.0);
  using namespace RopoFunction;
  openmove(0.0, 1.5, 0.0, 700);
  closemove(0.29, 0.39, -29.984, 1800);
  const int throw_number = 13;
  ReLoad();
  ShooterPneumatic();
  for(int i = 1; i <= throw_number - 1; i++){
    Shoot();
    ReLoad();
    if(i != throw_number - 1) pros::delay(800);
  }
  ShooterPneumatic();
  pros::delay(400);
  Shoot();
  closemove(0.37, 0.40, -94.75, 1500);
  int load_number = 10;
  ExternRight();                                                    //开翅膀
  for (int i = 1; i <= load_number; i++) {                          //导入n次球
    if(i % 3 == 0)
    {                                                             //导三次校准一次
      closemove(0.01, 0.29, -94.52, 500);
    }
    openmove(0.0, 0.0,  -8.0, 300);           //开导
    openmove(0.0, 0.0,  0.0, 150);
    if(i == load_number){break;}
    openmove(0.0, 0.0, 8.0, 280);           //回
    openmove(0.0, 0.0,  0.0, 600);
  }
  ExternRight();                                                    //收翅膀
  closemove(0.69, 0.53, -179.0, 2000);          //准备进通道
  openmove(-1.75, 0.0, 0.0, 750);             //往后猛退
  closemove(2.51, 0.53, -179.0, 2000);
  closemove(2.51, 0.47, 140.67, 800);
  ExternRight();                                                    //开翅膀
  openmove(-0.4, 0.0, 0.0, 500);               //慢慢往前走一点
  ExternRight();                                                    //收翅膀
  closemove(2.92, 0.20, 135.0, 600);           //屁股与导入杆后端齐平
  openmove(0.4, 0.4, 0.0, 300);                //往前调整位置，准备转身
  closemove(3.08, -0.08, 87.0, 900);          //转身准备推球
  openmove(0.3, 0.0, 0.0, 300);                //往前蓄力，准备推球 
  openmove(-1.57, 0.0, 0.0, 1000);               //推球
  openmove(0.8, 0.0, 0.0, 600);                 //蓄力准备再推
  openmove(-1.57, 0.0, 0.0, 900);               //再推
  openmove(0.8, 0.0, 0.0, 700);
  closemove(2.43, 0.04, 27.3117, 1200);
  ExternRight();                                                     //开翅膀
  openmove(-0.7, 0.0, 1.5,3000);
  ExternRight();                                                    //收翅膀 
  RopoDevice::Chassis.Operator();
}


void autonomous(){
	autonomous_Wisco();
	//RopoFunction::autonomous_1();
	//skill();
}
/* void Test(){
	RopoFunction::closemove(0,0,0,4000);
} */

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

	ButtonDetectLine.AddButtonDetect(pros::E_CONTROLLER_DIGITAL_RIGHT, RopoController::Rising, RopoFunction::ShooterInit);
	ButtonDetectLine.AddButtonDetect(pros::E_CONTROLLER_DIGITAL_RIGHT, RopoController::Falling, RopoFunction::ShooterStopInit);
	ButtonDetectLine.AddButtonDetect(pros::E_CONTROLLER_DIGITAL_B, RopoController::Rising, RopoFunction::ReLoad);
	ButtonDetectLine.AddButtonDetect(pros::E_CONTROLLER_DIGITAL_X, RopoController::Rising, RopoFunction::Shoot);

	ButtonDetectLine.AddButtonDetect(pros::E_CONTROLLER_DIGITAL_DOWN, RopoController::Rising, RopoFunction::ShooterPneumatic);
	ButtonDetectLine.AddButtonDetect(pros::E_CONTROLLER_DIGITAL_A, RopoController::Rising, skill);
	ButtonDetectLine.AddButtonDetect(pros::E_CONTROLLER_DIGITAL_Y, RopoController::Rising, RopoFunction::autonomous_1);

	ButtonDetectLine.AddButtonDetect(pros::E_CONTROLLER_DIGITAL_LEFT, RopoController::Rising, RopoFunction::ChangeControlMode);
	ButtonDetectLine.Enable();
	RopoDevice::Chassis.Operator();
	while (true) {
		FloatType XInput =  VelocityMax * XVelocityInput.GetAxisValue();
		FloatType YInput =  VelocityMax * YVelocityInput.GetAxisValue();
		FloatType WInput = -RopoWcLimit * WVelocityInput.GetAxisValue();

		if(RopoDevice::Chassis.IsOpcontrol()) RopoDevice::Chassis.OpSetAimStatus(XInput, YInput, WInput);
		MasterController.print(0,0,"%.2f, %.2f, %.2f", RopoDevice::Sensors::Encoder.GetPosX()/1000, RopoDevice::Sensors::Encoder.GetPosY()/1000, RopoDevice::Chassis.GetTheta() * 180 / RopoMath::Pi);
		pros::delay(5);
	}
}