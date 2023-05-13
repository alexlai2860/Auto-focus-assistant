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

#include "MotorData.h"

class Motor
{
protected:
    logic_ptr __logic;

public:
    Motor() = default;
    virtual int read() = 0;
    virtual int readPosition() = 0;
    virtual int readCommand() = 0;
    virtual void write(int,int) = 0;
    virtual void setZero() = 0;
    virtual void enable() = 0;
    virtual void test(int,int) = 0;
    virtual int init(TransferData &, TransferData &) = 0;
};

using motor_ptr = shared_ptr<Motor>;