// Code : UTF - 8
#pragma once

#include "RopoApi.hpp"
#include "RopoDiffySwerve.hpp"
#include <cmath>
#include "RopoSensor/EncodingDisk.hpp"
#include "pros/imu.hpp"
#include "RopoSensor/EncodingDisk.hpp"
#include "pros/imu.hpp"

namespace RopoChassis{
    enum ChassisMoveMode{
                OpenLoop,
                CloseLoop,
                Opcontrol
            };
    class Chassis{
        private:
            const FloatType Length = 0.295;
            const FloatType Width  = 0.295;
            

            RopoDiffySwerve::DiffySwerve &LF, &LB, &RF, &RB;
            RopoSensor::EncodingDisk &EncodingDisk;
            pros::IMU &InertialSensor;

            Matrix AimStatus = Matrix(3,1); // (Vx,Vy,W)
            Matrix SwerveAimStatus = Matrix(8,1); // (V,Theta)
            Matrix SwerveAimStatus_X_Y = Matrix(8,1); // (Vx,Vy)
            Matrix Transfer_M = Matrix(8,3); // Transfer Matix


            pros::Task *BackgroundTaskPtr;

            Matrix AimPosition = Matrix(3,1);
            Matrix ActualPosition = Matrix(3,1);
            Matrix PositionError = Matrix(3,1);
            Matrix Kp = Matrix(3,3);
            Matrix Ki = Matrix(3,3);
            Matrix Integrator = Matrix(3,1);
            Matrix Parameter = Matrix(3,3);

            bool Position_OK = false;
            static constexpr float ControlTime = 20; // ms
            FloatType XYMinError = 0.01;
            FloatType ThetaMinError = 0.01;
            int counter_for_error = 0;
            const int max_counter = 50;
            int max_time = 1000; // ms
            int DelayTime;
            RopoChassis::ChassisMoveMode MoveMode = Opcontrol;


            static void ChassisControl(void* param){
                if(param == nullptr) return;
                Chassis *This = static_cast<Chassis *>(param);
                while(true){
                    This -> UpdatePosition();
                    if(This -> MoveMode == OpenLoop){
                    }
                    else if (This -> MoveMode == CloseLoop){
                        int counter_for_time = 0;
                        while(1){
                            This -> UpdatePosition();
                            This -> PositionControl();
                            This -> MovingCalculate();
                            This -> SwerveMove();
                            counter_for_time++;
                            if(counter_for_time * 5 > This -> max_time || This -> Position_OK){
                                This -> MoveMode = Opcontrol;
                                break;
                            }
                            pros::delay(5);
                        }
                    }
                    else if (This -> MoveMode == Opcontrol){
                    }
                    This -> MovingCalculate();
                    This -> SwerveMove();
                    pros::delay(This -> DelayTime);
                    //This -> MoveMode = Opcontrol;
                }
            }

            void SwerveMove(){
                LF.SetAimStatus(SwerveAimStatus[1][1], SwerveAimStatus[2][1]);
                RF.SetAimStatus(SwerveAimStatus[3][1], SwerveAimStatus[4][1]);
                LB.SetAimStatus(SwerveAimStatus[5][1], SwerveAimStatus[6][1]);
                RB.SetAimStatus(SwerveAimStatus[7][1], SwerveAimStatus[8][1]);
            }

            inline void MovingCalculate(){
                SwerveAimStatus_X_Y = Transfer_M * AimStatus;
                for(int i = 1; i <= 7; i += 2){
                    SwerveAimStatus[i][1] = sqrtf(pow(SwerveAimStatus_X_Y[i][1], 2) + pow(SwerveAimStatus_X_Y[i+1][1],2));
                    if(SwerveAimStatus[i][1] != 0.0){
                        SwerveAimStatus[i+1][1] = atan2(SwerveAimStatus_X_Y[i+1][1],SwerveAimStatus_X_Y[i][1]);
                    }
                }
            }

        public:
            Chassis(RopoDiffySwerve::DiffySwerve& LF_, RopoDiffySwerve::DiffySwerve& LB_, RopoDiffySwerve::DiffySwerve& RF_, RopoDiffySwerve::DiffySwerve& RB_, pros::IMU& Imu, RopoSensor::EncodingDisk& Encoding_Disk)
            :LF(LF_), LB(LB_), RF(RF_), RB(RB_), InertialSensor(Imu), EncodingDisk(Encoding_Disk),
            AimStatus(3, 1), SwerveAimStatus(8, 1), SwerveAimStatus_X_Y(8, 1),
            Transfer_M(8, 3), BackgroundTaskPtr(nullptr) {
                LF.Initialize(); RF.Initialize(); LB.Initialize(); RB.Initialize();


                Transfer_M[1][1] = 1,Transfer_M[1][2] = 0,Transfer_M[1][3] = - Width  * 0.5;
                Transfer_M[2][1] = 0,Transfer_M[2][2] = 1,Transfer_M[2][3] = - Length * 0.5;
                Transfer_M[3][1] = 1,Transfer_M[3][2] = 0,Transfer_M[3][3] =   Width  * 0.5;
                Transfer_M[4][1] = 0,Transfer_M[4][2] = 1,Transfer_M[4][3] = - Length * 0.5;
                Transfer_M[5][1] = -1,Transfer_M[5][2]= 0,Transfer_M[5][3] =   Width  * 0.5;
                Transfer_M[6][1] = 0,Transfer_M[6][2] =-1,Transfer_M[6][3] = - Length * 0.5;
                Transfer_M[7][1] = -1,Transfer_M[7][2]= 0,Transfer_M[7][3] = - Width  * 0.5;
                Transfer_M[8][1] = 0,Transfer_M[8][2] =-1,Transfer_M[8][3] = - Length * 0.5;

                Kp[1][1] = 3.5; Kp[2][2] = 3.5; Kp[3][3] = 5;

                BackgroundTaskPtr = new Task(ChassisControl,this);
            }
            inline void UpdatePosition(){
                ActualPosition[1][1] = EncodingDisk.GetPosX() / 1000;
                ActualPosition[2][1] = EncodingDisk.GetPosY() / 1000;
                ActualPosition[3][1] = InertialSensor.get_yaw() / 180.0 * RopoMath::Pi;
            }
            inline void AutoSetAimStatus(FloatType const Vx, FloatType const Vy, FloatType const W, int Time = 5){
                MoveMode = OpenLoop;
                AimStatus[1][1] = Vx;
                AimStatus[2][1] = Vy;
                AimStatus[3][1] = W;
                DelayTime = Time;
            }
            inline void OpSetAimStatus(FloatType const Vx, FloatType const Vy, FloatType const W, int Time = 5){
                AimStatus[1][1] = Vx;
                AimStatus[2][1] = Vy;
                AimStatus[3][1] = W;
                DelayTime = Time;
            }
            void PositionControl(){
    
                PositionError = AimPosition - ActualPosition;
                if(fabsf(PositionError[3][1]) > RopoMath::Pi) PositionError[3][1] -= 2 * RopoMath::Pi * RopoMath::Sign(PositionError[3][1]);
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
                if(Position_OK) AimStatus[1][1] = AimStatus[2][1] = AimStatus[3][1] = 0;
                else{
                    AimStatus = Kp * PositionError;
                    AimStatus[1][1] = fabsf(AimStatus[1][1]) > 1.2 ? 1.2 * RopoMath::Sign(AimStatus[1][1]) : AimStatus[1][1];
                    AimStatus[2][1] = fabsf(AimStatus[2][1]) > 1.2 ? 1.2 * RopoMath::Sign(AimStatus[2][1]) : AimStatus[2][1];
                    AimStatus[3][1] = fabsf(AimStatus[3][1]) > (1.5 * RopoMath::Pi) ? (1.5 * RopoMath::Pi * RopoMath::Sign(AimStatus[3][1])) : AimStatus[3][1];
                    // Rotaion Matrix
                    Parameter[1][1] = cosf(ActualPosition[3][1]) , Parameter[1][2] = sinf(ActualPosition[3][1]) , Parameter[1][3] = 0;
                    Parameter[2][1] =-sinf(ActualPosition[3][1]) , Parameter[2][2] = cosf(ActualPosition[3][1]) , Parameter[2][3] = 0;
                    Parameter[3][1] = 0                          , Parameter[3][2] = 0                          , Parameter[3][3] = 1;
                    AimStatus = Parameter * AimStatus;
                }
                AimStatus[2][1] = -AimStatus[2][1];             // ???
            }

            void AutoSetPosition(FloatType x, FloatType y, FloatType theta, int _max_time){
                MoveMode = CloseLoop;
                counter_for_error = 0;
                Integrator[1][1] = Integrator[2][1] = Integrator[3][1] = 0;
                max_time = _max_time;
                Position_OK = false;
                AimPosition[1][1] = x;
                AimPosition[2][1] = y;
                AimPosition[3][1] = theta;
            }
            void OpenAuto(){
                MoveMode = OpenLoop;
            }
            /* void CloseAuto(){
                MoveMode = CloseLoop;
            } */
            void Operator(){
                MoveMode = Opcontrol;
            }
            /* void CloseAuto(){
                MoveMode = CloseLoop;
            } */

            bool IsAuto(){
                if(MoveMode != Opcontrol) return true;
                if(MoveMode != Opcontrol) return true;
                else return false;
            }
            bool IsOpcontrol(){
                if(MoveMode == Opcontrol) return true;
                if(MoveMode == Opcontrol) return true;
                else return false;
            }
            float GetAimX(){ return AimStatus[1][1];}
            float GetAimY(){ return AimStatus[2][1];}
            float GetAimW(){ return AimStatus[3][1];}
            float GetX(){
                return ActualPosition[1][1];
            }
            float GetY(){
                return ActualPosition[2][1];
            }
            float GetTheta(){
                return ActualPosition[3][1];
            }
    }; 
}
typedef RopoChassis::Chassis Chassis;