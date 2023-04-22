/**
 * @file NucluesN.h
 * @author 赖建宇 (lai.jianyu@foxmail.com)
 * @brief Tilta NucluesN reverse engineering
 * @version 1.0
 * @date 2023-04-23
 * 
 * @copyright Copyright SCUT RobotLab(c) 2021
 * 
 */

#pragma once

#include "motor.h"
#include "MotorData.h"

class NucluesN : public Motor
{
protected:
    data_ptr __data = make_shared<Data>();

public:
    void writePosition(int, int);

    NucluesN() = default;
    virtual int init(TransferData &, TransferData &) override;
};
