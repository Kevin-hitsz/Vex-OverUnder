#ifndef ROPO_MOTOR_GROUP_HPP
#define ROPO_MOTOR_GROUP_HPP

#include <vector>
#include "api.h"
namespace RopoMotorGroup
{
    class MotorGroup
    {
    private:
        std::vector<pros::Motor *> motor;  

    public:
        MotorGroup(std::initializer_list<pros::Motor*>);
        ~MotorGroup();
        
        /// @brief 按速度控制电机转动
        /// @param velocity 速度，范围依据齿轮类型
        /// @return 成功返回1，任何一个电机失败返回PROS_ERR
        int32_t move_velocity(double velocity)
        {
            int32_t ret=1;
            for(auto&i:motor)
            {
                if(i->move_velocity(velocity)!=1)
                ret=PROS_ERR;
            }
            return ret;
        };

        /// @brief 按电压控制电机转动
        /// @param voltage 电压，范围-12000~12000
        /// @return 成功返回1，任何一个电机失败返回PROS_ERR
        int32_t move_voltage(double voltage)
        {
            int32_t ret=1;
            for(auto&i:motor)
            {
                if(i->move_voltage(voltage)!=1)
                ret=PROS_ERR;
            }
            return ret;
        };

        /// @brief 对通信正常的电机位置求均值
        /// @return 电机组的有效位置 
        double get_position()
        {
            std::vector<double> tmp;
            double rhs=0;
            double lhs=0;
            for(auto&i:motor)
            {
                tmp.push_back(i->get_position());
                lhs+=std::isinf(tmp.back())?0:tmp.back();
                rhs+=std::isinf(tmp.back())?0:1.0;
            }
            return lhs/rhs;

        };
        /// @brief  将所有电机的位置置零
        /// @return 成功返回1,任何一个电机失败返回PROS_ERR
        int32_t tare_position()
        {
            int32_t ret=1;
            for(auto&i:motor)
            {

                if(i->tare_position()!=1)
                ret=PROS_ERR;
            }
            return ret;
        };
        /// @brief 设置编码器单位
        /// @param units 单位
        /// @return 成功返回1，任何一个电机失败返回PROS_ERR
        int32_t set_encoder_units(pros::motor_encoder_units_e units)
        {
            int32_t ret=1;
            for(auto&i:motor)
            {
                if(i->set_encoder_units(units)!=1)
                ret=PROS_ERR;
            }
            return ret;
        }
        /// @brief 设置齿轮箱
        /// @param gearing 齿轮类型
        /// @return 成功返回1，任何一个电机失败返回PROS_ERR
        int32_t set_gearing(pros::motor_gearset_e_t gearing)
        {
            int32_t ret=1;
            for(auto&i:motor)
            {
                if(i->set_gearing(gearing)!=1)
                ret=PROS_ERR;
            }
            return ret;
        }
        /// @brief 设置刹车
        /// @param mode 刹车模式
        /// @return 成功返回1，任何一个电机失败返回PROS_ERR
        int32_t set_brake_mode(pros::motor_brake_mode_e_t mode)
        {
            int32_t ret=1;
            for(auto&i:motor)
            {
                if(i->set_brake_mode(mode)!=1)
                ret=PROS_ERR;
            }
            return ret;
        }

    };


    /// @brief 构造电机组
    /// @param arg 电机的指针列表 e.g.MotorGroup({&motor1,&motor2...})
    MotorGroup::MotorGroup(std::initializer_list<pros::Motor*> arg)
    {
        for(auto &i:arg)
        {
            motor.push_back(i);
        }
    }
    
    MotorGroup::~MotorGroup()
    {
    }
}







#endif