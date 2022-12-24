/**
 * @file protocol.h
 * @author 赖建宇 (lai.jianyu@foxmail.com)
 * @brief 总协议
 * @version 1.0
 * @date 2022-12-13
 *
 * @copyright Copyright SCUT RobotLab(c) 2021
 *
 */

#pragma once

#include <iomanip>
#include <vector>
#include "TransferData.h"
#include "SerialPort.h"

// 抽象协议数据包管理类
class protocol
{
protected:
    bool __is_init;    // 串口初始化情况
    SerialPort __port; // 串口对象

public:
    protocol() : __is_init(false) {}

    /**
     * @brief 读取串口
     *
     * @param head 头帧
     * @param tail 尾帧
     */
    virtual TransferData read(uint8_t head, uint8_t tail) = 0;

    /**
     * @brief 写入串口
     *
     * @param yaw 偏转角
     * @param pitch 俯仰角
     * @param flags 标志位向量
     */
    virtual void write(int mode, const TransferData &) = 0;

    // 串口是否打开
    inline bool isOpened() { return __port.isOpen(); }
};
