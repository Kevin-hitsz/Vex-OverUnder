#pragma once
#include "RopoApi.hpp"
#include "RopoMath/Matrix.hpp"
#include "RopoMath/Misc.hpp"
#include "pros/motors.hpp"

namespace RopoDiffySwerve{
    class DiffySwerve{
        private:
            static constexpr FloatType SpinRatio = 2.0 / 3.0;// 轮速传动比
            static constexpr FloatType AngleRatio = 1.0 / 3.0; // 轮偏角传动比
            static constexpr FloatType WheelRadius = 2.75 * 0.0254 / 2.0;// 轮半径
            bool Control_Status = false;
            pros::Motor& Motor_1;
            pros::Motor& Motor_2;

            Matrix Status = Matrix(3,1);
            static constexpr float Control_Time = 1; // ms
            Matrix AimStatus = Matrix(3,1); // [alpha, alpha_dot, V]    
            Matrix Voltage = Matrix(2, 1);
            Matrix K = Matrix(2, 3); // Gain Matrix
            Matrix M1 = Matrix(3, 3); // Assitant Matrix
            Matrix M2 = Matrix(2, 1); // Assitant Matrix

            static void Swerve_Control(void* param){
                if(param == nullptr) return;
                DiffySwerve *This = static_cast<DiffySwerve *>(param);

                float Angle_Error = 0;
                
                while(1){
                    if(This -> Control_Status){
                        This -> GetStatus(This -> Status);
                        
                        Angle_Error = This -> AimStatus[1][1] - This -> Status[1][1]; // 轮偏角误差

                        // 轮偏角大于180°，则轮偏角减360°
                        if(fabsf(Angle_Error) > RopoMath::Pi){
                            This -> AimStatus[1][1] -= RopoMath::Sign(Angle_Error) * 2.0 * RopoMath::Pi;
                        }
                        Angle_Error = This -> AimStatus[1][1] - This -> Status[1][1]; // 更改后再次更新
                        
                        // 轮偏角大于90°，则反方向转，且轮转速也对应相反数
                        if(fabsf(Angle_Error) > RopoMath::Pi / 2.0){
                            This -> AimStatus[1][1] -= RopoMath::Sign(Angle_Error) * RopoMath::Pi;
                            // at targetPosition, we'll need to reverse our spin direction
                            This -> AimStatus[3][1] *= -1.0;
                        }
                        Angle_Error = This -> AimStatus[1][1] - This -> Status[1][1]; // 更改后再次更新

                        //放缩法 当|Angle_Error|趋近于90°，拉低轮速
                        This -> AimStatus[3][1] *= cosf(2 * Angle_Error) / 2.0 + 0.5;

		                // Voltage = 1000.0 * (M2 * 轮速 - K * (M1 * Status + AimStatus))，单位 mV
                        This -> Voltage = 1000.0 * (This -> M2 * This -> AimStatus[3][1] - This -> K * (This -> M1 * This -> Status + This -> AimStatus));
                        // 电压限幅输出
                        This -> Voltage[1][1] = RopoMath::Limit<float>(This -> Voltage[1][1], 12000.0);
                        This -> Voltage[2][1] = RopoMath::Limit<float>(This -> Voltage[2][1], 12000.0);
                        This -> Motor_1.move_voltage((int)This -> Voltage[1][1]);
                        This -> Motor_2.move_voltage((int)This -> Voltage[2][1]);
                    }
                    else {
                        This -> Motor_1.brake();
                        This -> Motor_2.brake();
                    }

                    pros::delay(This -> Control_Time);
                }
            }

        public:
            DiffySwerve(pros::Motor& _Motor_1, pros::Motor& _Motor_2)
				:Motor_1(_Motor_1), Motor_2(_Motor_2){
				new pros::Task(Swerve_Control, this);
			}
            ~DiffySwerve(){}
            inline void Control_On(){ Control_Status = true; }                                                               
            inline void Control_Off(){ Control_Status = false; }
            inline void Initialize(){
				Motor_1.tare_position();
				Motor_2.tare_position();

                Motor_1.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
                Motor_2.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);

                M1[1][1] = -1;
                M1[2][2] = 1;
                M1[3][3] = -1;

                M2[1][1] = 1;
                M2[2][1] = -1;
                M2 = (8.0 / 33.0) * M2;

                K[1][1] = -22.3607; K[1][2] = 0.4877; K[1][3] = -0.5164;
                K[2][1] = -22.3607; K[2][2] = 0.4877; K[2][3] =  0.5164;
                K = 0.8 * K;

				Control_On();
			}                                                              
            inline void SetAimStatus(float _AimSpeed, float _AimAngle){
				AimStatus[1][1] = _AimAngle;
                AimStatus[2][1] = 0.0;
        		AimStatus[3][1] = _AimSpeed / WheelRadius;
			}
            
            // 更新状态变量
            void GetStatus(Matrix &Status) {
                // 轮偏角(rad)
                Status[1][1] = (Motor_1.get_position() + Motor_2.get_position()) / 2.0 * RopoMath::Pi / 180.0 * AngleRatio;
                // 偏角速度(rad/s)
                Status[2][1] = (Motor_1.get_actual_velocity() + Motor_2.get_actual_velocity()) / 2.0 * RopoMath::Pi / 30.0 * AngleRatio;
                // 轮速(rad/s)
                Status[3][1] = (Motor_1.get_actual_velocity() - Motor_2.get_actual_velocity()) / 2.0 * RopoMath::Pi / 30.0 * SpinRatio;
            }
    };
}