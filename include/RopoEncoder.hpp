#pragma once

#include "RopoApi.hpp"
#include "RopoMath/Matrix.hpp"
#include "RopoParameter.hpp"
#include "pros/rotation.hpp"
#include <math.h>

namespace RopoEncoder{
    class Encoder{
        private:
            pros::Rotation RotationX;
            pros::Rotation RotationY;
            Matrix LastEncoderPosition;
            Matrix EncoderPosition;
            Matrix EncoderDetalVector;          // 编码器位置变化量
            Matrix RotationMatrix;              // 旋转矩阵
            Matrix EncoderAbsolutePosition;     // 编码器位置矩阵
            Matrix AbsolutePosition;            // 位置矩阵
            Matrix BiasMatrix;                  // 偏置矩阵(车体中心指向编码器中心,若编码器XY分置则需要分别测量)
            const int sampleTime = 10;          // 采样时间(ms)
            const FloatType WheelRad = 0.0273;  // 定位轮半径
            pros::Task* BackgroundTask;
            FloatType (*GetHeading)();

            void UpdateEncoderPosition(){
                EncoderPosition[1][1] = RotationX.get_position();
                EncoderPosition[2][1] = RotationY.get_position();
            }
            void UpdateRotationMatrix(const FloatType Radias){
                RotationMatrix[1][1] = cos(Radias);
                RotationMatrix[1][2] = -sin(Radias);
                RotationMatrix[2][1] = sin(Radias);
                RotationMatrix[2][2] = cos(Radias);
            }
            static void EncoderBackgroundFunction(void *Parameter){
				if(Parameter == nullptr)return;
				Encoder *This = static_cast<Encoder *>(Parameter);
                while(1){
                    FloatType Radias = This->GetHeading() / 180 * M_PI;
                    This -> UpdateEncoderPosition();
                    This -> UpdateRotationMatrix(Radias);
                    This -> EncoderDetalVector = (This -> EncoderPosition - This -> LastEncoderPosition) * (1.0 /180) * M_PI * This->WheelRad;
                    This -> EncoderAbsolutePosition = This -> EncoderAbsolutePosition + (This -> RotationMatrix * This -> EncoderDetalVector);
                    This -> AbsolutePosition = This -> EncoderAbsolutePosition - (This ->RotationMatrix * This -> BiasMatrix);
                    This -> LastEncoderPosition = This -> EncoderPosition;
                    pros::delay(This -> sampleTime);
                }
            }
        public:
            Encoder(pros::Rotation& rotationx, pros::Rotation& rotationy,
                    FloatType (*GetHeading)(),
                    FloatType biasx = 0, FloatType biasy = 0):
            RotationX(rotationx), RotationY(rotationy), BackgroundTask(nullptr),GetHeading(GetHeading),
            RotationMatrix(2, 2),EncoderAbsolutePosition(2,1),EncoderDetalVector(2,1),BiasMatrix(2,1),
            LastEncoderPosition(2,1),EncoderPosition(2,1),AbsolutePosition(2,1){
                RotationX.reset_position();
                RotationY.reset_position();
                SetBiasMatrix(biasx, biasy);
                BackgroundTask = new pros::Task(EncoderBackgroundFunction, this);
            }
            FloatType GetX() {return AbsolutePosition[1][1];}
            FloatType GetY() {return AbsolutePosition[2][1];}
            Matrix GetPositionMatrix() {return AbsolutePosition;}
            void SetBiasMatrix(const Matrix &matrix){ BiasMatrix = matrix;}
            void SetBiasMatrix(const FloatType x, const FloatType y){BiasMatrix[1][1] = x; BiasMatrix[2][1] = y;}
    };
}