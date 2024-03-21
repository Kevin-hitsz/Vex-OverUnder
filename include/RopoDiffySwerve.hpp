#pragma once
#include "RopoApi.hpp"
#include "RopoMath/Matrix.hpp"
#include "RopoMath/Misc.hpp"
#include "pros/error.h"
#include "pros/motors.hpp"

namespace RopoDiffySwerve{

    class DiffySwerve{
        private:
            // System
            static constexpr float SpinRatio = 143.0 / 200.0; // Wheel Rpm / Motor Rpm
            static constexpr float AngleRatio = 34.0 / 75.0; // Swerve Angle Rpm / Motor Rpm
            static constexpr float WheelRadius = 2.75 * 0.0254 / 2.0; // unit: m
            bool Control_Status = false; // start to control or not
            pros::Motor& Motor_1; // Motor No.1
            pros::Motor& Motor_2; // Motor No.2

            // Status parameters
            Matrix Status = Matrix(3,1); // include Speed and Angle

            // Control parameters
            static constexpr float Control_Time = 15; // unit: ms
            Matrix AimStatus = Matrix(3,1); // include AimSpeed and AimAngle    
            Matrix Voltage = Matrix(2, 1); // Drive voltage
            Matrix K = Matrix(2, 3); // Gain Matrix
            Matrix M1 = Matrix(3, 3); // Assitant Matrix
            Matrix M2 = Matrix(2, 1); // Assitant Matrix

            static void Swerve_Control(void* param){
                if(param == nullptr) return;
                DiffySwerve *This = static_cast<DiffySwerve *>(param);

                float Angle_Error = 0;
                
                // L
                while(1){
                    // Only start to control the swerve when the Control_Status is True
                    if(This -> Control_Status){
                        // Get the status
                        This -> UpdateStatus(This -> Status);
                        
                        Angle_Error = This -> AimStatus[1][1] - This -> Status[1][1];

                        // if the current values are more than half a rotation apart, move target Angle
                        // one rotation closer to currentPosition so it is within half a rotation apart
                        while(fabsf(Angle_Error) > RopoMath::Pi){
                            This -> AimStatus[1][1] -= RopoMath::Sign(Angle_Error) * 2.0 * RopoMath::Pi;
                            Angle_Error = This -> AimStatus[1][1] - This -> Status[1][1];
                        }

                        // move targetPosition a half rotation closer to currentPosition if necessary
                        if(fabsf(Angle_Error) > RopoMath::Pi / 2.0){
                            This -> AimStatus[1][1] -= RopoMath::Sign(Angle_Error) * RopoMath::Pi;
                            // at targetPosition, we'll need to reverse our spin direction
                            This -> AimStatus[3][1] *= -1.0;
                        }
                        Angle_Error = This -> AimStatus[1][1] - This -> Status[1][1];
                        // Anti-Spin While Big Angle Error
                        This -> AimStatus[3][1] *= (cosf(2 * Angle_Error) / 2.0 + 0.5)*(cosf(2 * Angle_Error) / 2.0 + 0.5);

                        // Get Control Value, while Volt to mV
		                This -> Voltage = 1000.0 * (This -> M2 * This -> AimStatus[3][1] - This -> K * (This -> M1 * This -> Status + This -> AimStatus));
                        // Votage Limitation
                        This -> Voltage[1][1] = RopoMath::Limit<float>(This -> Voltage[1][1], 12000.0);
                        This -> Voltage[2][1] = RopoMath::Limit<float>(This -> Voltage[2][1], 12000.0);

                        // Drive the motors
                        This -> Motor_1.move_voltage((int)This -> Voltage[1][1]);
                        This -> Motor_2.move_voltage((int)This -> Voltage[2][1]);
                    }
                    else {
                        This -> Motor_1 . brake();
                        This -> Motor_2 . brake();
                    }

                    pros::delay(This -> Control_Time);
                }
            }

        public:
            DiffySwerve(pros::Motor &_Motor_1, pros::Motor &_Motor_2)
				:Motor_1(_Motor_1), Motor_2(_Motor_2){
				new pros::Task(Swerve_Control, this);
			}
            ~DiffySwerve(){}
            inline void Control_On(){ Control_Status = true; }                                                                 // Start to control
            inline void Control_Off(){ Control_Status = false; }                                                                // Stop controlling
            inline void Initialize(){ // Intialization
				Motor_1 . tare_position();
				Motor_2 . tare_position();

                Motor_1 . set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
                Motor_2 . set_brake_mode(pros::E_MOTOR_BRAKE_COAST);

                M1[1][1] = -1;
                M1[2][2] = 1;
                M1[3][3] = -1;

                M2[1][1] = 1;
                M2[2][1] = -1;
                M2 = (2000.0 / 8437) * M2;

                K[1][1] = -14.1412; K[1][2] = 0.4139 ; K[1][3] = 0.0508;
                K[2][1] = K[1][1] ; K[2][2] = K[1][2]; K[2][3] = -K[1][3];
                

				// start control
				Control_On();
			}                                                              
            inline void SetAimStatus(float _AimSpeed, float _AimAngle){
				AimStatus[1][1] = _AimAngle;
                AimStatus[2][1] = 0.0;
        		AimStatus[3][1] = _AimSpeed / WheelRadius;
			}                                 // Set AimStatus
            void UpdateStatus(Matrix &Status) {
                // 轮偏角,且 Degree to Rad
                Status[1][1] = (Motor_1.get_position() + Motor_2.get_position()) / 2.0 * RopoMath::Pi / 180.0 * AngleRatio;
                x = Status[1][1];
                // 偏角速度,且 RPM to Rad/s
                Status[2][1] = (Motor_1.get_actual_velocity() + Motor_2.get_actual_velocity()) / 2.0 * RopoMath::Pi / 180.0 * AngleRatio;
                // 轮速,且 RPM to Rad/s
                Status[3][1] = (Motor_1.get_actual_velocity() - Motor_2.get_actual_velocity()) / 2.0 * RopoMath::Pi / 180.0 * SpinRatio;
            } 
            float x;                                                  // Upate Status
    };
}
