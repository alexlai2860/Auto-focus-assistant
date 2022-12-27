/**
 * @file data.cpp
 * @author 赖建宇 (lai.jianyu@foxmail.com)
 * @brief 步进电机驱动协议
 * @version 1.0
 * @date 2022-12-13
 *
 * @copyright Copyright SCUT RobotLab(c) 2021
 *
 */
#include "data.h"
#include <iostream>
#include <cmath>

using namespace std;

/**
 * @brief 读取串口
 *
 * @param head 头帧 通信地址 默认为0x01
 * @param tail 尾帧 固定校验为0x6B
 */
TransferData Data::read(uint8_t head, uint8_t tail)
{
    // init
    // if (!__is_init)
    // {
    //     // 初始化接收数据
    //     TransferData transfer_data;
    //     for (int i = 0; i < 4; i++)
    //     {
    //         transfer_data.read1[i] = {0x00};
    //     }
    //     __last_data = transfer_data;

    //     __is_init = true;
    //     return __last_data;
    // }
    // 接收数据
    vector<ReceivePulse> data = __port.readStruct<ReceivePulse>(head, tail);
    // 判空
    if (!data.empty())
    {
        __receive_data = data.back();
        // 整合得到传递数据
        TransferData transfer_data;
        for (int i = 0; i < 4; i++)
        {
            transfer_data.read1[i] = data.back().read_pulse[i];
        }
        // 更新信息数据
        __last_data = transfer_data;
    }
    return __last_data;
}

/**
 * @brief 写入串口115200

 * @param transfer_data 传递数据
 */
void Data::write(int mode, const TransferData &transfer_data)
{
    switch (mode)
    {
    case 1:
    {
        __send_data1.activate_command_h = transfer_data.command1;
        __send_data1.activate_command_l = transfer_data.command2;
        __port.writeStruct<SendData1>(__send_data1);
        break;
    }
    case 2:
    {
        __send_data2.read_command = transfer_data.command1;
        __port.writeStruct<SendData2>(__send_data2);
        break;
    }
    case 3:
    {
        // __send_data3.change_command = transfer_data.command1;
        __send_data3.change_content = transfer_data.command2; // 改变使能状态
        __port.writeStruct<SendData3>(__send_data3);
        break;
    }
    case 4:
    {
        __send_data4.direction_and_speed1 = transfer_data.direction_and_speed1;
        __send_data4.direction_and_speed2 = transfer_data.direction_and_speed2;
        __send_data4.pulse_h = transfer_data.pulse_h;
        __send_data4.pulse_m = transfer_data.pulse_m;
        __send_data4.pulse_l = transfer_data.pulse_l;
        __port.writeStruct<SendData4>(__send_data4);
        // cout << hex << unsigned(__send_data4.add) << " " << unsigned(__send_data4.byte1) << " " << unsigned(__send_data4.direction_and_speed) << " " << unsigned(__send_data4.accelerated_speed) << " " <<
        //             unsigned(__send_data4.pulse_h) << " " << unsigned(__send_data4.pulse_l) << " " << unsigned(__send_data4.ver) << endl;
        break;
    }
    default:
        break;
    }
}
