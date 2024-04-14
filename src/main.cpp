#include "main.h"
#include "RopoController.hpp"
#include "RopoDevice.hpp"
#include "RopoPosition.hpp"
#include "pros/misc.h"
#include "pros/rtos.hpp"
void autonomous_1();
void autonomous_2();
void test();
namespace ControllerModule {

	void BoolSwitch(void * Parameter){
		bool *p = static_cast<bool *>(Parameter);
		(*p) ^= 1;
	}

	bool wideExternFlag = false;
	void WideExternSwitch(){
		wideExternFlag ^= 1; 
		RopoDevice::ThreeWire::WideExternPneumatic.set_value(wideExternFlag);
	}

	bool leftExternFlag = false;
	void LeftExternSwitch(){
		leftExternFlag ^= 1; 
		RopoDevice::ThreeWire::LeftExternPneumatic.set_value(leftExternFlag);
	}

	bool rightExternFlag = false;
	void RightExternSwitch(){
		rightExternFlag ^= 1; 
		RopoDevice::ThreeWire::RightExternPneumatic.set_value(rightExternFlag);
	}

	void BothExternSwitch(){
		leftExternFlag ^= 1;
		rightExternFlag ^= 1;
		RopoDevice::ThreeWire::LeftExternPneumatic.set_value(leftExternFlag);
		RopoDevice::ThreeWire::RightExternPneumatic.set_value(rightExternFlag);
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
		intaker_backward = false;
		RollIntaker();
	}

	void SwitchIntakerBack(){
		intaker_backward ^= 1;
		RollIntaker();
	}

	void SwitchIntakerForToBack(){
		intaker_forward ^= 1;
		intaker_backward ^= 1;
		pros::delay(50);
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
	ButtonDetectLine.AddButtonDetect(pros::E_CONTROLLER_DIGITAL_R1   , RopoController::Rising, ControllerModule::BothExternSwitch);
	ButtonDetectLine.AddButtonDetect(pros::E_CONTROLLER_DIGITAL_R2   , RopoController::Rising,ControllerModule::ChangeLift);
	ButtonDetectLine.AddButtonDetect(pros::E_CONTROLLER_DIGITAL_L1   , RopoController::DoubleEdge, ControllerModule::SwitchIntakerForToBack);
	ButtonDetectLine.AddButtonDetect(pros::E_CONTROLLER_DIGITAL_L2   , RopoController::Rising, ControllerModule::SwitchIntakerFor);
	ButtonDetectLine.AddButtonDetect(pros::E_CONTROLLER_DIGITAL_LEFT , RopoController::Rising, ControllerModule::LeftExternSwitch);
	ButtonDetectLine.AddButtonDetect(pros::E_CONTROLLER_DIGITAL_RIGHT , RopoController::Rising, ControllerModule::RightExternSwitch);
	ButtonDetectLine.AddButtonDetect(pros::E_CONTROLLER_DIGITAL_X    , RopoController::Rising, ControllerModule::TurnAround);
	ButtonDetectLine.AddButtonDetect(pros::E_CONTROLLER_DIGITAL_B    , RopoController::Rising, ControllerModule::ChangeCatch);
	ButtonDetectLine.AddButtonDetect(pros::E_CONTROLLER_DIGITAL_A    , RopoController::Rising, RopoDevice::ChassisBrake);
	ButtonDetectLine.AddButtonDetect(pros::E_CONTROLLER_DIGITAL_DOWN , RopoController::Rising,  ControllerModule::WideExternSwitch);
	ButtonDetectLine.AddButtonDetect(pros::E_CONTROLLER_DIGITAL_UP   , RopoController::Rising,  ControllerModule::GpsUpdate);
	ButtonDetectLine.AddButtonDetect(pros::E_CONTROLLER_DIGITAL_Y    , RopoController::Pressing,  autonomous_1);
	ButtonDetectLine.AddButtonDetect(pros::E_CONTROLLER_DIGITAL_Y , RopoController::DoubleClick,  test);

	ButtonDetectLine.Enable();
	RopoDevice::ChassisCoast();
	ControllerModule::intaker_backward = false;
	ControllerModule::intaker_forward = true;
	ControllerModule::RollIntaker();

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
	RopoDevice::ChassisBrake();
	// --------- begin ------------
	//RopoDevice::Motors::IntakeMotor.move_velocity(500);
	RopoDevice::Chassis.MoveVelocity(0.8, 0);
	pros::delay(500);
	RopoDevice::Chassis.AutoPositionMove(1.12, -0.287);
	RopoDevice::Chassis.AutoRotateAbs(90);
	pros::delay(800);
	RopoDevice::Chassis.MoveVelocity(0.6, 0);
	pros::delay(500);
	ControllerModule::WideExternSwitch();		// 开翅膀
	//RopoDevice::Chassis.AutoPositionMove(1.12, -0.63);
	RopoDevice::Chassis.MoveVelocity(-1.0, 0);
	pros::delay(1000);   		// 推球



	// --------- stage 2 --------


	// --------- stage 3 ----------



	// --------- stage 4 --------------

	// --------- end --------------
	pros::delay(300);
	RopoDevice::Chassis.MoveVelocity(0,0);
}

void autonomous_2(){
	RopoDevice::Chassis.StartChassisAutoControll();//底盘MoveType设置为AutoMove
	// --------- begin ------------
	
	// --------- stage 2 --------
	
	// --------- stage 3 ----------
	

	// --------- stage 4 ----------
	
	// --------- end --------------
	pros::delay(300);
	RopoDevice::Chassis.MoveVelocity(0,0);
}

void test(){
	RopoDevice::Chassis.StartChassisAutoControll();//底盘MoveType设置为AutoMove
	RopoDevice::ChassisBrake();
	RopoDevice::Chassis.AutoPositionMove(1.12, -0.287);

	// --------- end --------------
	pros::delay(300);
	RopoDevice::Chassis.MoveVelocity(0,0);
}