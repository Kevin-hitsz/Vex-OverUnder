// Code : UTF - 8
#pragma once

#include "RopoApi.hpp"
#include "RopoDiffySwerve.hpp"
#include "pros/llemu.hpp"
#include <cmath>

namespace RopoChassis{
    class Chassis{
        private:
            const FloatType Length = 0.3672;
            const FloatType Width  = 0.3672;
            enum ChassisStatus{
                opcontrol = 0,
                autonomous= 1
            }Status;

            RopoDiffySwerve::DiffySwerve &LF, &LB, &RF, &RB;

            Matrix AimStatus; // (Vx,Vy,W)
            Matrix SwerveAimStatus; // (V,Theta)
            Matrix AimStatus_X_Y; // (Vx,Vy)
            Matrix Transfer_M; // Transfer Matix

            bool Position_OK = false;
            bool Time_Out = false;
            static constexpr float ControlTime = 20; // ms
            FloatType XYMinError = 0.01;
            FloatType ThetaMinError = 0.01;
            int counter_for_error = 0;
            const int max_counter = 50;
            int counter_for_time = 0;
            int max_time = 1000; // ms
            Matrix AimPosition = Matrix(3,1);
            Matrix Velocity = Matrix(3,1); 
            Matrix ActualPosition = Matrix(3,1);
            Matrix PositionError = Matrix(3,1);
            Matrix Kp = Matrix(3,3);
            Matrix Ki = Matrix(3,3);
            Matrix Integrator = Matrix(3,1);
            Matrix Parameter = Matrix(3,3);

            pros::Task *BackgroundTaskPtr;

            Matrix (* UpdatePosition)();

            static void ChassisControl(void* param){
                if(param == nullptr) return;
                Chassis *This = static_cast<Chassis *>(param);
                while(true){
                    This -> MovingCalculate();
                    This -> SwerveMove();
                    pros::delay(15);
                }
            }

            void SwerveMove(){
                LF.SetAimStatus(SwerveAimStatus[1][1], SwerveAimStatus[2][1]);
                RF.SetAimStatus(SwerveAimStatus[3][1], SwerveAimStatus[4][1]);
                LB.SetAimStatus(SwerveAimStatus[5][1], SwerveAimStatus[6][1]);
                RB.SetAimStatus(SwerveAimStatus[7][1], SwerveAimStatus[8][1]);
            }

            inline void MovingCalculate(){
                AimStatus_X_Y = Transfer_M * AimStatus;
                for(int i = 1; i <= 7; i += 2){
                    SwerveAimStatus[i][1] = sqrtf(pow(AimStatus_X_Y[i][1], 2) + pow(AimStatus_X_Y[i+1][1],2));
                    if(SwerveAimStatus[i][1] != 0.0){
                        SwerveAimStatus[i+1][1] = atan2(AimStatus_X_Y[i+1][1],AimStatus_X_Y[i][1]);
                    }
                }
            }

            void PositionControl(){
                Kp[1][1] = 1; Kp[2][2] = 1; Kp[3][3] = 5;
                Ki[1][1] = 1; Ki[2][2] = 1; Ki[3][3] = 100;
                while( Status == autonomous ){
                ActualPosition = UpdatePosition();
                PositionError = AimPosition - ActualPosition;
                // 限定作用域 
                if(fabsf(PositionError[3][1]) > RopoMath::Pi) PositionError[3][1] -= 2 * RopoMath::Pi * RopoMath::Sign(PositionError[3][1]);
                // 减少震荡
                if(fabsf(PositionError[1][1]) < XYMinError && fabsf(PositionError[2][1]) < XYMinError && fabsf(PositionError[3][1]) < ThetaMinError){
                    counter_for_error++;
                    if (counter_for_error > max_counter){
                        Position_OK = true;
                        counter_for_error = max_counter;
                    }
                }else{
                    counter_for_error = 0;
                    Position_OK = false;
                }

                if(fabsf(PositionError[3][1]) < 0.4) Integrator[3][1] += PositionError[3][1] * (ControlTime / 1000.0);
                Velocity = Kp * PositionError + Integrator;
                Velocity[1][1] = fabsf(Velocity[1][1]) > 1.2 ? 1.2 * RopoMath::Sign(Velocity[1][1]) : Velocity[1][1];
                Velocity[2][1] = fabsf(Velocity[2][1]) > 1.2 ? 1.2 * RopoMath::Sign(Velocity[2][1]) : Velocity[2][1];
                Velocity[3][1] = fabsf(Velocity[3][1]) > (1.5 * RopoMath::Pi) ? (1.5 * RopoMath::Pi * RopoMath::Sign(Velocity[3][1])) : Velocity[3][1];
                // Rotation Matrix         
                Parameter[1][1] = cosf(ActualPosition[3][1]) , Parameter[1][2] = sinf(ActualPosition[3][1]) , Parameter[1][3] = 0;
                Parameter[2][1] =-sinf(ActualPosition[3][1]) , Parameter[2][2] = cosf(ActualPosition[3][1]) , Parameter[2][3] = 0;
                Parameter[3][1] = 0 , Parameter[3][2] = 0 , Parameter[3][3] = 1;
                Velocity = Parameter * Velocity;
                SetAimStatus(Velocity);

                if(counter_for_time * ControlTime > max_time) Time_Out = true;
                counter_for_time++;
                // MasterController.print(0,0,"%.1f %.1f %.1f", AimPosition[1][1], 
                // AimPosition[2][1], AimPosition[3][1]);
                // sprintf(RopoDevice::Sensors::debugger.SendBuffer.Message,"%f, %f, 
                // %f, %f, %f, %f, %f, %f, %f\n", RopoDevice::AimPosition[1][1], ActualPosition[1][1], Velocity[1][1], RopoDevice::AimPosition[2][1], ActualPosition[2][1], 
                // Velocity[2][1], RopoDevice::AimPosition[3][1], ActualPosition[3][1], 
                // Velocity[3][1]);
                pros::delay(ControlTime);
                }
            }

        public:
            Chassis(RopoDiffySwerve::DiffySwerve& LF_,RopoDiffySwerve::DiffySwerve& LB_,RopoDiffySwerve::DiffySwerve& RF_,RopoDiffySwerve::DiffySwerve& RB_,Matrix(* p_getposition)())
            :Status(opcontrol), LF(LF_), LB(LB_), RF(RF_), RB(RB_), UpdatePosition(p_getposition),
            AimStatus(3, 1), SwerveAimStatus(8, 1), AimStatus_X_Y(8, 1),
            Transfer_M(8, 3), BackgroundTaskPtr(nullptr) {
                LF.Initialize(); RF.Initialize(); LB.Initialize(); RB.Initialize();
                LF.Start(); RF.Start(); LB.Start(); RB.Start();
                Transfer_M[1][1] = 1,Transfer_M[1][2] = 0,Transfer_M[1][3] = - Width  * 0.5;
                Transfer_M[2][1] = 0,Transfer_M[2][2] = 1,Transfer_M[2][3] = - Length * 0.5;
                Transfer_M[3][1] = 1,Transfer_M[3][2] = 0,Transfer_M[3][3] =   Width  * 0.5;
                Transfer_M[4][1] = 0,Transfer_M[4][2] = 1,Transfer_M[4][3] = - Length * 0.5;
                Transfer_M[5][1] = -1,Transfer_M[5][2]= 0,Transfer_M[5][3] =   Width  * 0.5;
                Transfer_M[6][1] = 0,Transfer_M[6][2] =-1,Transfer_M[6][3] = - Length * 0.5;
                Transfer_M[7][1] = -1,Transfer_M[7][2]= 0,Transfer_M[7][3] = - Width  * 0.5;
                Transfer_M[8][1] = 0,Transfer_M[8][2] =-1,Transfer_M[8][3] = - Length * 0.5;
                BackgroundTaskPtr = new Task(ChassisControl,this);
            }

            void SetAimStatus(FloatType const Vx, FloatType const Vy, FloatType const W){
                AimStatus[1][1] = Vx;
                AimStatus[2][1] = Vy;
                AimStatus[3][1] = W;
            }

            void SetAimStatus(Matrix const AimStatus_){
                AimStatus = AimStatus_;
            }

            void SetPosition(FloatType x, FloatType y, FloatType theta, int _max_time){
                counter_for_time = 0;
                counter_for_error = 0;
                Integrator[1][1] = Integrator[2][1] = Integrator[3][1] = 0;
                max_time = _max_time;
                Position_OK = false;
                Time_Out = false;
                AimPosition[1][1] = x;
                AimPosition[2][1] = y;
                AimPosition[3][1] = theta;
                while (!Position_OK && !Time_Out) pros::delay(20);
            } 
    };
}
typedef RopoChassis::Chassis Chassis;