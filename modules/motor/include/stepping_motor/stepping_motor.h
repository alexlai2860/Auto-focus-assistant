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
#include "data.h"

class SteppingMotor : public Motor
{
protected:

public:
    int readPulse(Data &);
    void writePulse(int, Data &);
    void setZero(Data &);
    
    SteppingMotor() = default;
    virtual int init(Data &, TransferData &, TransferData &) override;
};