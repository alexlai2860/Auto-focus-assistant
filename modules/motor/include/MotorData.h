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

#include <iomanip>
#include <vector>
#include "TransferData.h"
#include "SerialPort.h"
#include "LogicTools.h"
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
    uint8_t change_content; // 0为不使能，1为使能
    uint8_t ver = 0x6B;
};
#pragma pack()

// 视觉发送协议4:运动指令
struct SendData4
{
    uint8_t add = 0x01;
    uint8_t byte1 = 0xFD;         // 位置模式控制
    uint8_t direction_and_speed1; // 高半字节表示方向（0/1），其余表示速度（最大为4FF）
    uint8_t direction_and_speed2;
    uint8_t accelerated_speed = (uint8_t)param.A; // 关闭曲线加减速
    uint8_t pulse_h;                              // 脉冲数，共三个字节
    uint8_t pulse_m;
    uint8_t pulse_l;
    uint8_t ver = 0x6B;
};
#pragma pack()

// 视觉发送协议5:铁头原力N-运动指令
struct SendData5
{
    // 全部以hex表示
    // 示例指令为 :010600010001F7
    // extreme_right:01060600270EBE
    // extreme_left:010606000000F3
    // uint8_t command[15] = ":010600010001F7";
    uint8_t start = ':';              // 起始位(:)
    uint8_t model[2] = {'0', '1'};    // 电机序号(01)(怪)
    uint8_t command[2] = {'0', '6'};  // 指令flag(默认对焦)
    uint8_t param_h[2] = {'0', '6'};  // 参数h
    uint8_t param_l[2] = {'0', '0'};  // 参数l
    uint8_t data_h[2] = {'2', '2'};   // 数据h
    uint8_t data_l[2] = {'0', 'E'};   // 数据l
    uint8_t checksum[2] = {'B', 'E'}; // 校验和
    uint8_t CR = 0x0D;
    uint8_t LF = 0x0A;
};
#pragma pack()

// 视觉发送协议5:铁头原力N-校准指令
struct SendData6
{
    // cal:960604D200048A or :960604D200008E
    // long_press::3F0600010001B9
    // uint8_t nucleus_cal[15] = {':', '9', '6', '0', '6', '0', '4', 'D', '2', '0', '0', '0', '4', '8', 'A'};
    uint8_t nucleus_cal[15] = {':', '3', 'F', '0', '6', '0', '0', '0', '1', '0', '0', '0', '1', 'B', '9'};
    uint8_t CR = 0x0D;
    uint8_t LF = 0x0A;
};
#pragma pack()

// 视觉发送协议5:铁头原力N-录制开始指令
struct SendData7
{
    // rec_on:C9060000000130*2
    uint8_t nucleus_rec_on[15] = {};
    uint8_t CR = 0x0D;
    uint8_t LF = 0x0A;
};
#pragma pack()

// 视觉发送协议5:铁头原力N-录制结束指令
struct SendData8
{
    // rec_off:C9060000000031*2
    uint8_t nucleus_rec_off[15] = {};
    uint8_t CR = 0x0D;
    uint8_t LF = 0x0A;
};
#pragma pack()

// 视觉接收协议
struct ReceivePulse
{
    uint8_t read_pulse[4]; // 注意这里不能用int8_t
};
#pragma pack()

// 视觉接收协议-NucleusN
struct ReceiveNucleusN
{
    uint8_t read_data[14]; // 从0x3A开始，到0x0D结束
};
#pragma pack()

// stepping motor data
class Data
{
private:
    SendData1 __send_data1; // 视觉发送数据
    SendData2 __send_data2;
    SendData3 __send_data3;
    SendData4 __send_data4;
    SendData5 __send_data5;
    SendData6 __send_data6;
    // SendData7 __send_data7;
    // SendData8 __send_data8;

    ReceivePulse __receive_data; // 视觉接收数据
    ReceiveNucleusN __receive_nucleusn_data;
    TransferData __last_data; // 上一帧的传递数据

    bool __is_init;    // 串口初始化情况
    SerialPort __port; // 串口对象

public:
    /**
     * @brief 构造函数
     */
    Data() : __is_init(false) {}

    /**
     * @brief 读取串口
     *
     * @param head 头帧
     * @param tail 尾帧
     */
    TransferData read(uint8_t head, uint8_t tail);
    TransferData readNucleusN(uint8_t head, uint8_t tail);

    /**
     * @brief 写入串口
     *
     * @param transfer_data 传递数据
     */
    void write(int mode, const TransferData &transfer_data);

    uint8_t Ascii2Hex(const uint8_t data_h, const uint8_t data_l);
    vector<uint8_t> Hex2Ascii(const uint8_t &data);
};

using data_ptr = shared_ptr<Data>;