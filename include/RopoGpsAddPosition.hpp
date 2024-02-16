#ifndef ROPO_GPS_ADD_POSITION
#define ROPO_GPS_ADD_POSITION
#include "RopoParameter.hpp"
#include "RopoApi.hpp"
#include "RopoMath/Header.hpp"
#include "pros/motors.hpp"
#include "pros/misc.hpp"
#include "pros/gps.hpp"
#include "pros/rtos.hpp"
namespace RopoGpsAddPosition {
    class GpsAddPositionModule {
        private:
            Vector originalPosition, transformedPosition;
            Vector (*GetCurPosition)();
            FloatType gpsRelativeX, gpsRelativeY;
            FloatType gpsRelativeX0, gpsRelativeY0;
            FloatType originalX0, originalY0;
            int sampleTime;
            int updateFlag;
            int cnt_update;
            pros::Gps& gps1;
            pros::Task* BackgroundTask;

        public:
            GpsAddPositionModule(Vector (*GetPosition_)(),pros::Gps& gps_,int sampleTime_ = 2,int updateFlag_ = 0):
                                GetCurPosition(GetPosition_),gps1(gps_),
                                sampleTime(sampleTime_),updateFlag(updateFlag_),BackgroundTask(nullptr),
                                originalPosition(RopoMath::ColumnVector,3), transformedPosition(RopoMath::ColumnVector,3),
                                gpsRelativeX(0), gpsRelativeY(0), gpsRelativeX0(0), gpsRelativeY0(0), originalX0(0),originalY0(0){
                BackgroundTask = new pros::Task(GpsAddPositionBackgroundFunction,this);
            };
            ~GpsAddPositionModule() {
                delete BackgroundTask;
                BackgroundTask = nullptr;
            }
            static void GpsAddPositionBackgroundFunction(void *Parameter) {
				if(Parameter == nullptr)return;
				GpsAddPositionModule *This = static_cast<GpsAddPositionModule *>(Parameter);
                This -> cnt_update = 0;
                while(true) {
                    This -> originalPosition = This -> GetCurPosition();//从原函数指针获取的坐标三元素x,y,theta
                    This -> transformedPosition[1] = This -> gpsRelativeX0 + This -> originalPosition[1] - This -> originalX0;
                    This -> transformedPosition[2] = This -> gpsRelativeY0 + This -> originalPosition[2] - This -> originalY0;
                    //x,y坐标通过gps是否更新来使得全场定位，gps的全场坐标通过坐标转换，转换为对于车体初始化的相对坐标
                    This -> transformedPosition[3] = This -> originalPosition[3];//theta直接拿来

                    This -> GpsTransformUpdate();
                    pros::delay(This -> sampleTime);
                    if(This -> updateFlag != 0) {
                        This -> cnt_update++;
                        if(This -> cnt_update >= This -> updateFlag && This -> gps1.get_error() < 0.02){
                            This -> GpsUpdate();
                            This -> cnt_update = 0;
                        }
                    }
                }
            }	

            void GpsTransformUpdate() {
                double X = gps1.get_status().x - RopoParameter::GPSX_INITIAL;
                double Y = gps1.get_status().y - RopoParameter::GPSY_INITIAL;
                double theta = RopoParameter::ROPO_HEADING_INITIAL;
                gpsRelativeX =  X * RopoMath::Cos(theta) + Y * RopoMath::Sin(theta);
                gpsRelativeY = -X * RopoMath::Sin(theta) + Y * RopoMath::Cos(theta);
            }

            void GpsUpdate() {
                    
                    originalX0 = originalPosition[1];
                    originalY0 = originalPosition[2];
                    gpsRelativeX0 = gpsRelativeX;
                    gpsRelativeY0 = gpsRelativeY;
            }

            FloatType GetGpsTransformRelativePositionX() {return gpsRelativeX;}

            FloatType GetGpsTransformRelativePositionY() {return gpsRelativeY;}

            Vector    GetTransformedPosition() {return transformedPosition;}

            FloatType GetTransformedPositionX() {return transformedPosition[1];}

            FloatType GetTransformedPositionY() {return transformedPosition[2];}

            FloatType GetTransformedPositionTheta() {return transformedPosition[3];}

            void      SetUpdateFlag(int updateFlag_) {updateFlag = updateFlag_;}
    };
}


#endif // ROPO_GPS_ADD_POSITION