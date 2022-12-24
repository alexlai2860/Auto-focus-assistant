/**
 * @file TransferData.h
 * @author 赖建宇 (lai.jianyu@foxmail.com)
 * @brief
 * @version 1.0
 * @date 2022-12-13
 *
 * @copyright Copyright SCUT RobotLab(c) 2021
 *
 */

#pragma once

#include <bits/types.h>
#include <memory>
#include <vector>

// 传递数据
class TransferData
{
public:
    uint8_t command1; // 自定义命令
    uint8_t command2;
    uint16_t command3;

    uint8_t byte1 = 0xFD;         // 默认使用位置模式控制
    uint8_t direction_and_speed1; // 高半字节表示方向（0/1），其余表示速度（最大为4FF）
    uint8_t direction_and_speed2;
    uint8_t accelerated_speed = 0xFF; // 默认关闭曲线加减速
    uint8_t pulse_h;                  // 脉冲数，共三个字节
    uint8_t pulse_m;
    uint8_t pulse_l;

    // 数据信息，顺序与通信协议里 uint8_t 的顺序一致
    std::vector<uint8_t> data;

    TransferData() : command1(0x00), command2(0x00), command3(0x0000), direction_and_speed1(0x00), direction_and_speed2(0x00), pulse_h(0x00), pulse_m(0x00), pulse_l(0x00) {}
};
