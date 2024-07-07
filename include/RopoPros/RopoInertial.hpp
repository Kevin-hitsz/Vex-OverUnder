#ifndef ROPO_INERTIAL_HPP
#define ROPO_INERTIAL_HPP
#include"api.h"
class RopoInertial
{
private:
    pros::Imu imu;
    double k;
public:
    /// @brief 构造惯性传感器
    /// @param  端口号
    /// @param  角度修正系数
    RopoInertial(int,double);
    ~RopoInertial();
    /// @brief 返回修正后的rotation
    /// @return rotation
    double get_rotation()
    {
        return -imu.get_rotation()*k;
    };
    /// @brief 返回修正后的偏航角
    /// @return yaw
    double get_yaw()
    {
        return (int((get_rotation()+180+360000)*100)%36000)/100.0-180.0;
    };

    /// @brief 校准
    /// @param blk 是否阻塞
    /// @return 成功返回1，否则返回PROS_ERR
    int32_t reset(bool blk=false)
    {
        int32_t ret=1;
        if(imu.reset(blk)!=1)
        ret=PROS_ERR;
        return ret;
    };
    /// @brief 
    /// @return 
    bool is_calibrating()
    {
        return imu.is_calibrating();
    }
};

RopoInertial::RopoInertial(int port,double arg=1):imu(port),k(arg)
{
}

RopoInertial::~RopoInertial()
{
}


#endif