#pragma once
#include "RopoApi.hpp"
#include "RopoMath/Matrix.hpp"
#include "RopoMath/Misc.hpp"
#include "pros/motors.hpp"

namespace RopoDiffySwerve{
    // FloatType V1;
    // FloatType V2;
    // FloatType A;
    // FloatType A_;
    // FloatType V;
    // FloatType angle_error;
    // FloatType v_error;

    class DiffySwerve{
        private:
            static constexpr FloatType SpinRatio = 2.0 / 3.0;// 轮速传动比
            static constexpr FloatType AngleRatio = 1.0 / 3.0; // 轮偏角传动比
            static constexpr FloatType WheelRadius = 2.75 * 0.0254 / 2.0;// 轮半径
            static constexpr float Control_Time = 10; // ms

            Motor& Motor_1;
            Motor& Motor_2;

            Matrix Status;
            Matrix AimStatus;   // [alpha, alpha_dot, V]    
            Matrix Voltage;
            Matrix K;           // Gain Matrix
            Matrix M1;          // Assitant Matrix
            Matrix M2;          // Assitant Matrix

            pros::Task *BackgroundTaskPtr;

            static void Swerve_Control(void* param){
                if(param == nullptr) return;
                DiffySwerve *This = static_cast<DiffySwerve *>(param);

                float Angle_Error = 0;

                while(1){
                    This -> GetStatus();
                    
                    Angle_Error = This -> AimStatus[1][1] - This -> Status[1][1]; // 轮偏角误差

                    // 轮偏角大于180°，则轮偏角减360°
                    while (Angle_Error < -RopoMath::Pi || RopoMath::Pi <= Angle_Error) {
                        This -> AimStatus[1][1] -= RopoMath::Sign(Angle_Error) * 2.0 * RopoMath::Pi;
                        Angle_Error = This -> AimStatus[1][1] - This -> Status[1][1]; // 更改后再次更新
                        // Angle_Error -= RopoMath::Sign(Angle_Error) * 2.0 * RopoMath::Pi;
                    }
                    
                    // 轮偏角大于90°，则反方向转，且轮转速也对应相反数
                    if (Angle_Error < -RopoMath::Pi / 2.0 || RopoMath::Pi / 2.0 <= Angle_Error) {
                        This -> AimStatus[1][1] -= RopoMath::Sign(Angle_Error) * RopoMath::Pi;
                        Angle_Error = This -> AimStatus[1][1] - This -> Status[1][1]; // 更改后再次更新
                        // at targetPosition, we'll need to reverse our spin direction
                        This -> AimStatus[3][1] *= -1.0;
                    }

                    //放缩法 当|Angle_Error|趋近于90°，拉低轮速
                    This -> AimStatus[3][1] *= cosf(2 * Angle_Error) / 2.0 + 0.5;

                    // Voltage = 1000.0 * (M2 * Vd - K * (M1 * X + Xd))
                    This -> Voltage = 1000.0 * (This -> M2 * This -> AimStatus[3][1] - This -> K * (This -> M1 * This -> Status + This -> AimStatus));
                    // Limit Voltage
                    This -> Voltage[1][1] = RopoMath::Limit<float>(This -> Voltage[1][1], 12000.0);
                    This -> Voltage[2][1] = RopoMath::Limit<float>(This -> Voltage[2][1], 12000.0);
                    This -> Motor_1.move_voltage((int)This -> Voltage[1][1]);
                    This -> Motor_2.move_voltage((int)This -> Voltage[2][1]);
                    // V1 = This->Voltage[1][1];
                    // V2 = This->Voltage[2][1];
                    // angle_error = Angle_Error;
                    // v_error = This ->AimStatus[3][1] - This -> Status[3][1];
                    pros::delay(This -> Control_Time);
                }
            }

        public:
            DiffySwerve(pros::Motor& _Motor_1, pros::Motor& _Motor_2)
				:Motor_1(_Motor_1), Motor_2(_Motor_2), Status(3, 1),
                AimStatus(3, 1), Voltage(2, 1),
                K(2, 3), M1(3, 3), M2(2, 1),
                BackgroundTaskPtr(nullptr) { }
            ~DiffySwerve(){
                delete BackgroundTaskPtr;
            }
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
                M2 = (45.0 / 177.0) * M2;

                K[1][1] = -22.3607; K[1][2] = 0.7230; K[1][3] =  -0.4972;
                K[2][1] = -22.3607; K[2][2] = 0.7230; K[2][3] =   0.4972;
                // K = 0.8 * K;
			}                                                              
            inline void SetAimStatus(float _AimSpeed, float _AimAngle) {
				AimStatus[1][1] = _AimAngle;
                AimStatus[2][1] = 0.0;
        		AimStatus[3][1] = _AimSpeed / WheelRadius;
			}
            void Start() {
                BackgroundTaskPtr = new Task(Swerve_Control, this);
            }

            // 更新状态变量
            void GetStatus() {
                // 轮偏角(rad)
                Status[1][1] = (Motor_1.get_position() + Motor_2.get_position()) / 2.0 * RopoMath::Pi / 180.0 * AngleRatio;
                // 偏角速度(rad/s)
                Status[2][1] = (Motor_1.get_actual_velocity() + Motor_2.get_actual_velocity()) / 2.0 * RopoMath::Pi / 30.0 * AngleRatio;
                // 轮速(rad/s)
                Status[3][1] = (Motor_1.get_actual_velocity() - Motor_2.get_actual_velocity()) / 2.0 * RopoMath::Pi / 30.0 * SpinRatio;

                // A = Status[1][1];
                // A_= Status[2][1];
                // V = Status[3][1];
            }
    };
}
