#include "main.h"
#include "RopoController.hpp"
#include "RopoDevice.hpp"
#include "RopoPosition.hpp"
#include "pros/misc.h"
#include "pros/rtos.hpp"
void autonomous_1();
void autonomous_2();
void test();
void skill();
namespace ControllerModule {

	void BoolSwitch(void * Parameter){
		bool *p = static_cast<bool *>(Parameter);
		(*p) ^= 1;
	}

	bool externFlag = false;
	void ExternSwitch(){
		externFlag ^= 1; 
		RopoDevice::ThreeWire::ExternPneumatic.set_value(externFlag);
	}

	bool locktag = false;
	void ChangeCatch(){
		locktag ^= 1;
		RopoDevice::ThreeWire::CatchPneumatic.set_value(locktag);
	}

	void RumbleMe(){
		pros::Controller MasterController1(pros::E_CONTROLLER_MASTER);
		MasterController1.rumble("-.-.-");
		FloatType aa = pros::millis();
		while(pros::millis()-aa<45000){
			pros::delay(100);
		}
		MasterController1.rumble("-.-");
		while(pros::millis()-aa<55000){
			pros::delay(100);
		}
		RopoDevice::ChassisBrake();
		MasterController1.rumble("-.-.-");
		while(pros::millis()-aa<65000){
			pros::delay(100);
		}
		MasterController1.rumble("-.-.-.-");
	}
	int catch_1 = 0;
	void Hold(){
		RopoDevice::LiftMotors.Hold();
		catch_1 = 1;
	}

	void Lift(){
		RopoDevice::LiftMotors.Wait();
		catch_1 = 2;
	}

	void Hide(){
		RopoDevice::LiftMotors.Hide();
		catch_1 = 0;
	}

	void ChangeLift(){
		if (catch_1 == 1) {
			Hide();
		} else {
			Hold();
		}
	}

	bool intaker_forward = false;
	bool intaker_backward = false;
	void RollIntaker(){
		if (intaker_forward && !intaker_backward) {
			RopoDevice::Motors::IntakeMotor.move_velocity(-500);
		}
		else if (!intaker_forward && intaker_backward) {
			RopoDevice::Motors::IntakeMotor.move_velocity(500);
		}
		else {
			RopoDevice::Motors::IntakeMotor.move_velocity(0);
		}
	}

	void SwitchIntakerFor(){
		intaker_forward ^= 1;
		RollIntaker();
	}

	void SwitchIntakerBack(){
		intaker_backward ^= 1;
		RollIntaker();
	}

	void TurnAround(){
		RopoDevice::Chassis.AutoRotateRelative(180);
	}

	void GpsUpdate(){
		RopoDevice::gpsAddPosition.GpsUpdate();
	}

	void AutoLift(){
		RopoDevice::Chassis.MoveVelocity(-0.6,0);
		pros::delay(400);
		RopoDevice::Chassis.MoveVelocity(-0.3,0);
		pros::delay(500);
		RopoDevice::Chassis.MoveVelocity(0,0);
		ChangeCatch();
		RopoDevice::Chassis.MoveVelocity(-0.6,0);
		pros::delay(1000);
		RopoDevice::Chassis.MoveVelocity(-1.2,0);
		pros::delay(1000);
		RopoDevice::Chassis.MoveVelocity(-2.0,0);
		pros::delay(1000);
		RopoDevice::Chassis.MoveVelocity(-1.3,0);
		pros::delay(1000);
		RopoDevice::Chassis.MoveVelocity(-0.6,0);
		pros::delay(1000);
		RopoDevice::Chassis.MoveVelocity(-0.3,0);
		pros::delay(500);
		RopoDevice::Chassis.MoveVelocity(0,0);

	}

	void ControllerPrint(){
		while(true) {
			pros::Controller MasterController(pros::E_CONTROLLER_MASTER);
			MasterController.print(0,1,"degree: %.1lf",RopoDevice::GetPosition()[3]);
			pros::delay(50); 
			MasterController.print(2,1,"X: %.2lf Y:%.2lf",(RopoDevice::GetTransformedPosition())[1],(RopoDevice::GetTransformedPosition())[2]);
			pros::delay(50); 
			MasterController.print(1,1,"degree: %.1lf",-RopoDevice::Sensors::Inertial.get_rotation()*1.017);
			pros::delay(50); 
			// MasterController.print(2,1,"%.2lf  %d",RopoDevice::LiftMotors.GetLifterPosition(), RopoDevice::LiftMotors.GetLifterStatus());
			// pros::delay(50);
		}
	}
}

void initialize() {
	pros::lcd::initialize();
	pros::delay(50);
	RopoDevice::DeviceInit();
	RopoDevice::MotorsInit();
	RopoDevice::Position_Motor::MyPosition.initial();
}

void disabled() {}

void competition_initialize() {}

void autonomous(){
	autonomous_2();
}

void opcontrol()
{
	//pros::Task *RumbleTask = new pros::Task(ControllerModule::RumbleMe);
	pros::Task *PrintTask = new pros::Task(ControllerModule::ControllerPrint);
	pros::Controller MasterController(pros::E_CONTROLLER_MASTER);
	RopoController::ButtonTaskLine ButtonDetectLine(MasterController);
	FloatType VelocityMax = 1.6;//1.4
	FloatType RopoWcLimit = 6;
	bool ChassisMove = false;
	
	RopoController::AxisValueCast XVelocityInput(MasterController,pros::E_CONTROLLER_ANALOG_LEFT_Y,RopoController::Linear);
	RopoController::AxisValueCast WVelocityInput(MasterController,pros::E_CONTROLLER_ANALOG_RIGHT_X,RopoController::Linear);

	Vector Velocity(RopoMath::ColumnVector,2),ResVelocity;

	MasterController.clear();
	ButtonDetectLine.AddButtonDetect(pros::E_CONTROLLER_DIGITAL_R1   , RopoController::Rising, ControllerModule::ExternSwitch);
	ButtonDetectLine.AddButtonDetect(pros::E_CONTROLLER_DIGITAL_R2   , RopoController::Rising,ControllerModule::ChangeLift);
	ButtonDetectLine.AddButtonDetect(pros::E_CONTROLLER_DIGITAL_L1   , RopoController::DoubleEdge, ControllerModule::SwitchIntakerFor);
	ButtonDetectLine.AddButtonDetect(pros::E_CONTROLLER_DIGITAL_L2   , RopoController::DoubleEdge, ControllerModule::SwitchIntakerBack);
	ButtonDetectLine.AddButtonDetect(pros::E_CONTROLLER_DIGITAL_X    , RopoController::Rising, ControllerModule::TurnAround);
	ButtonDetectLine.AddButtonDetect(pros::E_CONTROLLER_DIGITAL_A    , RopoController::Rising, ControllerModule::ChangeCatch);
	ButtonDetectLine.AddButtonDetect(pros::E_CONTROLLER_DIGITAL_A    , RopoController::Rising, RopoDevice::ChassisBrake);
	//ButtonDetectLine.AddButtonDetect(pros::E_CONTROLLER_DIGITAL_Y  , RopoController::Rising,  autonomous_2);
	// ButtonDetectLine.AddButtonDetect(pros::E_CONTROLLER_DIGITAL_UP, RopoController::Rising,  autonomous);
	ButtonDetectLine.AddButtonDetect(pros::E_CONTROLLER_DIGITAL_LEFT , RopoController::Rising,  test);
	ButtonDetectLine.AddButtonDetect(pros::E_CONTROLLER_DIGITAL_DOWN , RopoController::Rising,  ControllerModule::GpsUpdate);

	ButtonDetectLine.Enable();
	RopoDevice::ChassisCoast();
	ControllerModule::intaker_backward = false;
	ControllerModule::intaker_forward = false;
	RopoDevice::Motors::IntakeMotor.move_velocity(0);

	while (true) {

		FloatType XInput =  XVelocityInput.GetAxisValue();
		FloatType WInput = -WVelocityInput.GetAxisValue();
		FloatType RopoWc = RopoWcLimit-fabs(XInput) * 0.0;			
		FloatType RopoVx = VelocityMax-fabs(WInput) * 0.7;	

		if (fabs(XInput) <= 0.06 && fabs(WInput) <= 0.03) {
			if(ChassisMove == true){
				RopoDevice::Motors::MoveOpControll(0.0, 0.0);
				ChassisMove = false;
			}
		} 
		else {
			RopoDevice::Chassis.StartChassisOpControll();//底盘MoveType设置为OpMove
			RopoDevice::Motors::MoveOpControll(XInput * RopoVx, WInput * RopoWc);
			ChassisMove = true;
		}
		pros::delay(4);
	}
}

void autonomous_1(){
	RopoDevice::Chassis.StartChassisAutoControll();//底盘MoveType设置为AutoMove
	// --------- begin ------------
	RopoDevice::Chassis.MoveVelocity(0.8,0);
	pros::delay(500);
	RopoDevice::Chassis.AutoRotateAbs(-60);
	pros::delay(1000);
	ControllerModule::SwitchIntakerBack();//1+

	pros::delay(500);
	ControllerModule::SwitchIntakerBack();//1-			吐

	ControllerModule::SwitchIntakerFor();//1+
	RopoDevice::Chassis.AutoPositionMove(0.55,0.35);
	RopoDevice::Chassis.MoveVelocity(0.2,0);
	pros::delay(200);
	RopoDevice::Chassis.MoveVelocity(0.0,0);
	pros::delay(100);

	RopoDevice::Chassis.AutoRotateAbs(-90);
	pros::delay(800);
	RopoDevice::Chassis.AutoPositionMove(0.55,-0.07);
	ControllerModule::SwitchIntakerFor();//1-			吃
	pros::delay(50);

	//RopoDevice::gpsAddPosition.GpsUpdate();
	ControllerModule::SwitchIntakerBack();//1+
	pros::delay(700);
	ControllerModule::SwitchIntakerBack();//1-			吐

	RopoDevice::Chassis.AutoPositionMove(0.62,0.39);	// 0.401
	RopoDevice::Chassis.AutoRotateAbs(3);
	pros::delay(500);

	ControllerModule::ExternSwitch();//+
	pros::delay(500);
	ControllerModule::ExternSwitch();//-			打走

	ControllerModule::SwitchIntakerFor();//1+
	RopoDevice::Chassis.AutoPositionMove(0.845,0.39);	// 0.401
	pros::delay(300);
	ControllerModule::SwitchIntakerFor();//1-

	RopoDevice::Chassis.MoveVelocity(-0.20,0);
	pros::delay(300);						//		先吃再吐
	ControllerModule::SwitchIntakerBack();//1+
	RopoDevice::Chassis.MoveVelocity(0,0);
	pros::delay(600);

	RopoDevice::Chassis.MoveVelocity(0.65,0.2);
	pros::delay(1700);
	ControllerModule::SwitchIntakerBack();//1-

	// --------- stage 2 --------
	RopoDevice::Chassis.AutoRotateAbs(0);
	pros::delay(300);
	RopoDevice::Chassis.MoveVelocity(0,0);
	pros::delay(300);
	RopoDevice::gpsAddPosition.GpsUpdate();
	pros::delay(150);

	ControllerModule::SwitchIntakerFor();//1+
	RopoDevice::Chassis.AutoPositionMove(1.72, 0.40);
	pros::delay(300);	

	RopoDevice::Chassis.MoveVelocity(-0.2,0);
	pros::delay(300);
		
	RopoDevice::Chassis.AutoPositionMove(1.75, 0.9);
	pros::delay(300);	
	ControllerModule::SwitchIntakerFor();//1-
	RopoDevice::Chassis.AutoRotateAbs(0);
	pros::delay(600);
	ControllerModule::ExternSwitch();//+
	pros::delay(100);
	
	RopoDevice::Chassis.MoveVelocity(0.75,-0.3);
	pros::delay(1200);

	// --------- stage 3 ----------

	RopoDevice::Chassis.MoveVelocity(-0.5,0);
	pros::delay(500);
	ControllerModule::ExternSwitch();//-
	RopoDevice::Chassis.AutoPositionMove(2.28, 0.35);
	RopoDevice::Chassis.AutoRotateAbs(2);
	pros::delay(600);
	RopoDevice::Chassis.MoveVelocity(0,0);
	pros::delay(300);
	RopoDevice::gpsAddPosition.GpsUpdate();
	pros::delay(50);

	RopoDevice::Chassis.AutoRotateAbs(135);
	pros::delay(1000);
	RopoDevice::Chassis.MoveVelocity(-0.65,0);
	pros::delay(900);
	RopoDevice::Chassis.MoveVelocity(-0.3,0);
	pros::delay(300);

	RopoDevice::Chassis.MoveVelocity(0,0);
	ControllerModule::ChangeLift();	// 放
	pros::delay(700);
	RopoDevice::Chassis.MoveVelocity(0.4,0);
	pros::delay(800);
	RopoDevice::Chassis.AutoPositionMove(2.00, 0.6);
	RopoDevice::Chassis.AutoRotateAbs(-175);
	pros::delay(800);
	RopoDevice::Chassis.AutoDirectMove(2.34, 0.55, true);
	ControllerModule::ChangeLift();	// 收
	RopoDevice::Chassis.MoveVelocity(-0.8,0);
	pros::delay(700);

	// --------- stage 4 --------------
	RopoDevice::Chassis.MoveVelocity(0,0);
	pros::delay(300);
	RopoDevice::gpsAddPosition.GpsUpdate();
	pros::delay(50);
	RopoDevice::Chassis.MoveVelocity(0.5,0);
	pros::delay(300);
	RopoDevice::Chassis.AutoPositionMove(1.77, 0);
	RopoDevice::Chassis.AutoRotateAbs(60);
	pros::delay(1000);
	ControllerModule::ChangeLift(); 	//放
	pros::delay(1000);

	// --------- end --------------
	pros::delay(300);
	RopoDevice::Chassis.MoveVelocity(0,0);
}

void autonomous_2(){
	RopoDevice::Chassis.StartChassisAutoControll();//底盘MoveType设置为AutoMove
	// --------- begin ------------
	RopoDevice::Chassis.MoveVelocity(0.8,0);
	pros::delay(500);
	RopoDevice::Chassis.AutoRotateAbs(-60);
	pros::delay(1000);
	ControllerModule::SwitchIntakerBack();//1+

	pros::delay(500);
	ControllerModule::SwitchIntakerBack();//1-			吐

	ControllerModule::SwitchIntakerFor();//1+
	RopoDevice::Chassis.AutoPositionMove(0.55,0.35);
	RopoDevice::Chassis.MoveVelocity(0.2,0);
	pros::delay(200);
	RopoDevice::Chassis.MoveVelocity(0.0,0);
	pros::delay(100);

	RopoDevice::Chassis.AutoRotateAbs(-90);
	pros::delay(800);
	RopoDevice::Chassis.AutoPositionMove(0.55,-0.07);
	ControllerModule::SwitchIntakerFor();//1-			吃
	pros::delay(50);

	//RopoDevice::gpsAddPosition.GpsUpdate();
	ControllerModule::SwitchIntakerBack();//1+
	pros::delay(700);
	ControllerModule::SwitchIntakerBack();//1-			吐

	RopoDevice::Chassis.AutoPositionMove(0.62,0.39);	// 0.401
	RopoDevice::Chassis.AutoRotateAbs(3);
	pros::delay(500);

	ControllerModule::ExternSwitch();//+
	pros::delay(500);
	ControllerModule::ExternSwitch();//-			打走

	ControllerModule::SwitchIntakerFor();//1+
	RopoDevice::Chassis.AutoPositionMove(0.85,0.39);	// 0.401
	pros::delay(300);
	ControllerModule::SwitchIntakerFor();//1-

	RopoDevice::Chassis.MoveVelocity(-0.20,0);
	pros::delay(250);						//		先吃再吐
	ControllerModule::SwitchIntakerBack();//1+
	RopoDevice::Chassis.MoveVelocity(0,0);
	pros::delay(500);

	RopoDevice::Chassis.MoveVelocity(0.6,0.2);
	pros::delay(1600);

	// --------- stage 2 --------
	RopoDevice::Chassis.AutoRotateAbs(0);
	pros::delay(400);
	RopoDevice::Chassis.MoveVelocity(0.6,0.2);
	pros::delay(800);
	ControllerModule::SwitchIntakerBack();//1-
	RopoDevice::Chassis.MoveVelocity(0,0);
	pros::delay(300);
	RopoDevice::gpsAddPosition.GpsUpdate();
	RopoDevice::Chassis.MoveVelocity(-0.3,0);
	pros::delay(300);
	ControllerModule::SwitchIntakerFor();//1+
	RopoDevice::Chassis.AutoPositionMove(1.72, 0.40);	// !!!!!!
	pros::delay(300);	
	
	RopoDevice::Chassis.MoveVelocity(-0.2,0);
	pros::delay(300);
		
	RopoDevice::Chassis.AutoPositionMove(1.79, 0.85);
	pros::delay(300);	
	ControllerModule::SwitchIntakerFor();//1-
	RopoDevice::Chassis.AutoRotateAbs(0);
	pros::delay(400);
	ControllerModule::ExternSwitch();//+
	pros::delay(300);
	
	RopoDevice::Chassis.MoveVelocity(0.75,-0.3);
	pros::delay(1000);

	// --------- stage 3 ----------
	RopoDevice::Chassis.MoveVelocity(0,0);
	pros::delay(300);
	RopoDevice::gpsAddPosition.GpsUpdate();
	
	RopoDevice::Chassis.MoveVelocity(-0.5,0);
	pros::delay(250);
	ControllerModule::ExternSwitch();//-
	ControllerModule::SwitchIntakerFor();//1+
	RopoDevice::Chassis.AutoPositionMove(1.75, 0.94);	//// !!!
	ControllerModule::ExternSwitch();//+
	RopoDevice::Chassis.MoveVelocity(-0.6,0.1);
	pros::delay(2000);

	RopoDevice::Chassis.MoveVelocity(0.5,0);
	pros::delay(600);
	ControllerModule::ExternSwitch();//-
	RopoDevice::Chassis.AutoRotateAbs(3);
	pros::delay(1500);
	ControllerModule::SwitchIntakerFor();//1-
	ControllerModule::SwitchIntakerBack();//1+
	RopoDevice::Chassis.MoveVelocity(0.5,0);
	pros::delay(500);
	
	ControllerModule::SwitchIntakerBack();//1-

	// --------- stage 4 ----------
	RopoDevice::Chassis.MoveVelocity(0,0);
	pros::delay(300);
	RopoDevice::gpsAddPosition.GpsUpdate();

	RopoDevice::Chassis.MoveVelocity(-0.5,0);
	pros::delay(500);
	RopoDevice::Chassis.AutoPositionMove(1.9, 0.23);
	RopoDevice::Chassis.AutoRotateAbs(75);
	pros::delay(1500);
	ControllerModule::ExternSwitch();//+
	RopoDevice::Chassis.MoveVelocity(0.8, -1);
	pros::delay(1500);

	RopoDevice::Chassis.MoveVelocity(-0.5,0);
	pros::delay(600);
	RopoDevice::Chassis.MoveVelocity(0.6, 0);
	pros::delay(600);

	RopoDevice::Chassis.MoveVelocity(-0.6,0.6);
	pros::delay(800);

	// --------- end --------------
	pros::delay(300);
	RopoDevice::Chassis.MoveVelocity(0,0);
}

void test(){
	RopoDevice::Chassis.StartChassisAutoControll();//底盘MoveType设置为AutoMove
	RopoDevice::Chassis.AutoPositionMove(0.2,-0.07,45);
	RopoDevice::Chassis.AutoPositionMoveBack(0.55,-0.60,-180);
	RopoDevice::Chassis.AutoPositionMoveBack(0.55,-0.07,0);
	RopoDevice::Chassis.AutoPositionMove(0,0,0);
}