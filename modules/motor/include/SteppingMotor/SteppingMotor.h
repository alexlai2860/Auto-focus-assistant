/**
 * @file stepping_motor.h
 * @author 赖建宇 (lai.jianyu@foxmail.com)
 * @brief 步进电机头文件
 * @version 1.0
 * @date 2023-03-07
 *
 * @copyright Copyright SCUT RobotLab(c) 2021
 *
 */
#pragma once

#include "motor.h"
#include "MotorData.h"

class SteppingMotor : public Motor
{
protected:
    data_ptr __data = make_shared<Data>();

public:
    int read();
    void write(int, int);
    void setZero();
    void enable();
    void test(int, int);

    SteppingMotor() = default;
    virtual int init(TransferData &, TransferData &) override;
};
