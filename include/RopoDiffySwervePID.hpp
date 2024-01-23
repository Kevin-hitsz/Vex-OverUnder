#pragma once
#include "pros/motors.hpp"
#include "RopoMath/Header.hpp"
#include "RopoApi.hpp"

namespace RopoDiffySwervePID{
    FloatType V1;
    FloatType V2;
    class DIffySwervePID{
        private:
            static constexpr FloatType SpinRatio = 1.0 / 3.0;               // 轮速转动比
            static constexpr FloatType AngleRatio = 1.0 / 6.0;              // 轮偏角转动比
            static constexpr FloatType WheelRadius = 2.75 * 0.0254 / 2.0;   // 轮半径
            static constexpr float Control_Time = 10;          // ms
            static constexpr FloatType k = 182.63;            // u = k*v 
            bool Control_Status = false;
            double AimSpeed , CurrentSpeed;    // rad/s
            double AimAngle , CurrentAngle;    // rad
            Motor& motor1;
            Motor& motor2;


            struct PID{
                double err;              
                double err_last; 				// k-1         
                double err_beforelast;     		// k-2   
                double Kp = 1.2 ,Ki = 0.008 ,Kd = 0.01;  
                double output = 0;
                double deltaError = 1.0;
                bool ifArrive = false;
            }pid;

            static void Swerve_Control(void* param){
                if(param == nullptr) return;
                DIffySwervePID *This = static_cast<DIffySwervePID *>(param);

                float Angle_Error = 0;

                while(1){
                    if(This->Control_Status){
                        This->GetStatus(&(This->CurrentSpeed), &(This->CurrentAngle));

                        Angle_Error = This->AimAngle - This->CurrentAngle;  // 轮偏角误差

                        // 轮偏角大于180°，则轮偏角减360°
                        if(fabsf(Angle_Error) > RopoMath::Pi){
                            This->AimAngle -= RopoMath::Sign(Angle_Error) * 2.0 * RopoMath::Pi;
                        }
                        Angle_Error = This->AimAngle - This->CurrentAngle;  // 更改后再次更新

                        // 轮偏角大于90°，则反方向转
                        if(fabsf(Angle_Error) > RopoMath::Pi / 2.0){
                            This->AimAngle -= RopoMath::Sign(Angle_Error) * RopoMath::Pi;
                        }
                        Angle_Error = This->AimAngle - This->CurrentAngle;  // 更改后再次更新

                        // 放缩法 当|Angle_Error|趋近于90°，拉低轮速
                        This->AimSpeed *= cosf(2 * Angle_Error) / 2.0 + 0.5;

                        // PID
                        int delta_voltage = (int)  3 * This->AimSpeed * k;
                        This->pid.err = Angle_Error;
                        double increment = This->pid.Kp*(This->pid.err-This->pid.err_last)+This->pid.Ki*This->pid.err+This->pid.Kd*(This->pid.err-2*This->pid.err_last+This->pid.err_beforelast);	// 增量式
                        This->pid.err_beforelast = This->pid.err_last;      // k-1 -> k-2
                        This->pid.err_last = This->pid.err;                 //   k -> k-1
                        This->pid.output += increment;
                        
                        This->motor1.move_voltage(delta_voltage/2 + RopoMath::Sign(Angle_Error) * This->pid.output * 1000 );
                        This->motor2.move_voltage(-delta_voltage/2 + RopoMath::Sign(Angle_Error) * This->pid.output * 1000);
                        V1 = delta_voltage/2 + RopoMath::Sign(Angle_Error) * This->pid.output * 1000 ;
                        V2 = -delta_voltage/2 + RopoMath::Sign(Angle_Error) * This->pid.output * 1000;
                      
                    }
                    else{
                        This->motor1.brake();
                        This->motor2.brake();
                    }

                    pros::delay(This->Control_Time);
                }
            }
        public:
            DIffySwervePID(Motor& _motor1, Motor& _motor2): motor1(_motor1), motor2(_motor2){
                new Task(Swerve_Control, this);
            }
            ~DIffySwervePID(){}
            inline void Control_On(){ Control_Status = true; }
            inline void Control_Off(){ Control_Status = false; }
            inline void Initialize(){
                motor1.tare_position();
                motor2.tare_position();

                motor1.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
                motor2.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);

                pid.err = 0, pid.err_last = 0, pid.err_beforelast = 0;
                Control_On();
            }

            double get_velocity(){
                static float M1_vel_last  = 0, M2_vel_last  = 0, M1_vel_now = 0, M2_vel_now = 0;
                M1_vel_now = motor1.get_actual_velocity();
                M2_vel_now = motor2.get_actual_velocity();

                // ERR Solution
                if(M1_vel_now == PROS_ERR_F) M1_vel_now = M1_vel_last;
                if(M2_vel_now == PROS_ERR_F) M2_vel_now = M2_vel_last;

                M1_vel_last = M1_vel_now;
                M2_vel_last = M2_vel_now;

                double v1 = M1_vel_now * RopoMath::Pi / 30;
                double v2 = M2_vel_now * RopoMath::Pi / 30;
                return (v1 - v2) * SpinRatio;
            }
            double get_direction(){
                static float M1_pos_basis = 0, M2_pos_basis = 0;
                static float M1_pos_last  = 0, M2_pos_last  = 0, M1_pos_now = 0, M2_pos_now = 0;
                M1_pos_now = motor1.get_direction();
                M2_pos_now = motor2.get_direction();

                // ERR Solution
                if(M1_pos_now == PROS_ERR_F) M1_pos_now = M1_pos_basis = M1_pos_last;
                else M1_pos_now += M1_pos_basis;
                if(M2_pos_now == PROS_ERR_F) M2_pos_now = M2_pos_basis = M2_pos_last;
                else M2_pos_now += M2_pos_basis;

                M1_pos_last = M1_pos_now;
                M2_pos_last = M2_pos_now;

                double theta1 = M1_pos_now * RopoMath::Pi /180;   
                double theta2 = M2_pos_now * RopoMath::Pi /180; 
                return (theta1 + theta2) * AngleRatio; 
            }

            inline void SetAimStatus(float _AimSpeed, float _AimAngle){
                AimAngle = _AimAngle;
                AimSpeed = _AimSpeed;
            }

            // 更新状态变量
            void GetStatus(double* _currentspeed, double* _currentangle){
                *_currentangle = get_direction();
                *_currentspeed = get_velocity();
            }
    };
}