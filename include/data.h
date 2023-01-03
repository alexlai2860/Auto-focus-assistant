/**
 * @file data.h
 * @author 赖建宇 (lai.jianyu@foxmail.com)
 * @brief 步进电机驱动协议头文件
 * @version 1.0
 * @date 2022-12-13
 *
 * @copyright Copyright SCUT RobotLab(c) 2021
 *
 */

#pragma once

#include "protocol.h"
#include "param.h"

#pragma pack(1)

// 视觉发送协议1:触发指令（默认为置零）
struct SendData1
{
    uint8_t add = 0x01;
    uint8_t activate_command_h = 0x0A;
    uint8_t activate_command_l = 0x6D;
    uint8_t ver = 0x6B;
};
#pragma pack()

// 视觉发送协议2:读取指令
struct SendData2
{
    uint8_t add = 0x01;
    uint8_t read_command;
    uint8_t ver = 0x6B;
};
#pragma pack()

// 视觉发送协议3:修改指令（默认为改变使能状态，使电机不使能）
struct SendData3
{
    uint8_t add = 0x01;
    uint8_t change_command = 0xF3;
    uint8_t change_content = 0x00; // 0为不使能，1为使能
    uint8_t ver = 0x6B;
};
#pragma pack()

// 视觉发送协议4:运动指令
struct SendData4
{
    uint8_t add = 0x01;
    uint8_t byte1 = 0xFD;                // 位置模式控制
    uint8_t direction_and_speed1 = 0x14; // 高半字节表示方向（0/1），其余表示速度（最大为4FF）
    uint8_t direction_and_speed2 = 0xFF;
    uint8_t accelerated_speed = (uint8_t)param.A; // 关闭曲线加减速
    uint8_t pulse_h = 0x00;                       // 脉冲数，共三个字节
    uint8_t pulse_m = 0x00;
    uint8_t pulse_l = 0x80;
    uint8_t ver = 0x6B;
};
#pragma pack()

// enum Region : uint8_t
// {
//     Nolmal = 0x00,
//     Static = 0x01
// };

// 视觉接收协议
struct ReceivePulse
{
    uint8_t read_pulse[4]; // 注意这里不能用int8_t
};
#pragma pack()
// __attribute__((packed));

// stepping motor data
class Data : public protocol
{
private:
    SendData1 __send_data1; // 视觉发送数据
    SendData2 __send_data2;
    SendData3 __send_data3;
    SendData4 __send_data4;
    ReceivePulse __receive_data; // 视觉接收数据

    TransferData __last_data; // 上一帧的传递数据

public:
    /**
     * @brief 构造函数
     */
    Data() : protocol() {}

    /**
     * @brief 读取串口
     *
     * @param head 头帧
     * @param tail 尾帧
     */
    TransferData read(uint8_t head, uint8_t tail) override;

    /**
     * @brief 写入串口
     *
     * @param transfer_data 传递数据
     */
    void write(int mode, const TransferData &transfer_data) override;
};