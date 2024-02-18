// Code : UTF - 8
#pragma once

#include "RopoApi.hpp"
#include "RopoDiffySwerve.hpp"
#include <cmath>

namespace RopoChassis{
    class Chassis{
        private:
            const FloatType Length = 0.3672;
            const FloatType Width  = 0.3672;
            

            RopoDiffySwerve::DiffySwerve &LF, &LB, &RF, &RB;

            Matrix AimStatus; // (Vx,Vy,W)
            Matrix SwerveAimStatus; // (V,Theta)
            Matrix AimStatus_X_Y; // (Vx,Vy)
            Matrix Transfer_M; // Transfer Matix

            pros::Task *BackgroundTaskPtr;

            

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

        public:
            enum ChassisStatus{
                opcontrol = 0,
                autonomous= 1,
                freedrive = 2
            }Status;
            Matrix (* UpdatePosition)();
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

            void AutoStart(){
                Status = autonomous;
            }

            void Autoend(){
                Status = opcontrol;
            }

            bool IsAuto(){
                if(Status == autonomous && Status == freedrive) return true;
                else return false;
            }     

            bool IsOpcontrol(){
                if(Status == opcontrol) return true;
                else return false;
            }      

            void FreeDrive(){
                Status = freedrive;
            }
    }; 
}
typedef RopoChassis::Chassis Chassis;