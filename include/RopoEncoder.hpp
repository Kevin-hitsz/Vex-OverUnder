#pragma once
#include "RopoApi.hpp"
#include "RopoDevice.hpp"
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
            Matrix DetalVector;
            Matrix RotationMatrix;
            Matrix AbsolutePosition;
            const FloatType sampleTime = 10;    //  ms
            const FloatType WheelRad = 0.0273;  //  定位轮半径
            pros::Task* BackgroundTask;
            FloatType (*GetHeading)();

            void UpdateEncoderPosition(){
                EncoderPosition[1][1] = RotationX.get_position();
                EncoderPosition[2][1] = RotationY.get_position();
            }
            void UpdateRotationMatrix(FloatType Radias){
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
                    This -> DetalVector = (This -> EncoderPosition - This -> LastEncoderPosition) * (1.0 /180) * M_PI * This->WheelRad;
                    This -> AbsolutePosition = This -> AbsolutePosition + (This -> RotationMatrix * This -> DetalVector);
                    This -> LastEncoderPosition = This -> EncoderPosition;
                    pros::delay(This -> sampleTime);
                }
            }
        public:
            Encoder(pros::Rotation& rotationx, pros::Rotation& rotationy, FloatType (*GetHeading)()):
            RotationX(rotationx), RotationY(rotationy), BackgroundTask(nullptr),GetHeading(GetHeading),
            RotationMatrix(2, 2),AbsolutePosition(2,1),DetalVector(2,1),LastEncoderPosition(2,1),EncoderPosition(2,1){
                RotationX.reset_position();
                RotationY.reset_position();
                BackgroundTask = new pros::Task(EncoderBackgroundFunction, this);
            }
            FloatType GetX() {return AbsolutePosition[1][1];} 
            FloatType GetY() {return AbsolutePosition[2][1];} 
    };
}