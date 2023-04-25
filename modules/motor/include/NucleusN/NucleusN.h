/**
 * @file NucleusN.h
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

class NucleusN : public Motor
{
protected:
    data_ptr __data = make_shared<Data>();
    vector<TransferData> read_datas;

public:
    virtual void write(int, int) override;
    virtual int read() override;
    void setZero();
    void enable();
    void test(int, int);

    NucleusN() = default;
    virtual int init(TransferData &, TransferData &) override;
};
