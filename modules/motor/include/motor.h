/**
 * @file motor.h
 * @author 赖建宇 (lai.jianyu@foxmail.com)
 * @brief
 * @version 1.0
 * @date 2023-03-07
 *
 * @copyright Copyright SCUT RobotLab(c) 2021
 *
 */
#pragma once

#include "data.h"

class Motor
{
public:
    Motor() = default;
    virtual int readPulse(Data &) = 0;
    virtual void writePulse(int, Data &) = 0;
    virtual void setZero(Data &) = 0;
    virtual int init(Data &, TransferData &, TransferData &) = 0;
};

using motor_ptr = shared_ptr<Motor>;