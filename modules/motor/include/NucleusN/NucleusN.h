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
    deque<TransferData> read_datas;
    TransferData last_data; // 上一次读到的data
    int last_position;      // 上一次读到的position
    bool key_trigger;       // 按键触发器
    deque<int> position;
    deque<int> command;
    deque<int> writeposition;
    bool command_init = 0;
    int last_com_pos; // 上一次读取到command在buffer中的位置

public:
    virtual void write(int, int) override;
    virtual int read() override;
    virtual int readPosition() override;
    virtual int readCommand() override;
    void setZero();
    void enable();
    void test(int, int);

    NucleusN() = default;
    virtual int init(TransferData &, TransferData &) override;
};
