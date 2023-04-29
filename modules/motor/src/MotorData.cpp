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
#include "MotorData.h"
#include <iostream>
#include <cmath>

using namespace std;

/**
 * @brief ASCII转hex
 *
 * @param data {'F','7'}
 * @return uint8_t 0xF7
 */
uint8_t Data::Ascii2Hex(const uint8_t data_h, const uint8_t data_l)
{
    uint8_t output;
    uint8_t output_h;
    uint8_t output_l;
    // cout << "h" << (int)data_h << endl;
    // cout << "l" << (int)data_l << endl;

    if ((int)data_h >= 65)
    {
        output_h = (int)data_h - 65 + 10;
    }
    else
    {
        output_h = (int)data_h - 48;
    }

    if ((int)data_l >= 65)
    {
        output_l = (int)data_l - 65 + 10;
    }
    else
    {
        output_l = (int)data_l - 48;
    }

    // cout << "oph" << (int)output_h << endl;
    // cout << "opl" << (int)output_l << endl;
    output = output_h << 4 | output_l;
    return output;
}

/**
 * @brief
 *
 * @param data
 * @return vector<uint8_t>
 */
vector<uint8_t> Data::Hex2Ascii(const uint8_t &data)
{
    vector<uint8_t> output;
    uint8_t output_h;
    uint8_t output_l;
    uint8_t data_h = data >> 4;
    uint8_t data_l = data - (data_h << 4);

    // cout << "data_h" << (int)data_h << endl;
    // cout << "data_h" << (int)data_l << endl;
    if ((int)data_h > 9)
    {
        output_h = (int)data_h - 10 + 65;
    }
    else
    {
        output_h = (int)data_h + 48;
    }

    if ((int)data_l > 9)
    {
        output_l = (int)data_l - 10 + 65;
    }
    else
    {
        output_l = (int)data_l + 48;
    }
    // cout << "oph" << (int)output_h << endl;
    // cout << "opl" << (int)output_l << endl;

    output.push_back(output_h);
    output.push_back(output_l);
    return output;
}

/**
 * @brief 读取串口()
 *
 * @param head 头帧 通信地址 默认为0x01
 * @param tail 尾帧 固定校验为0x6B
 */
TransferData Data::read(uint8_t head, uint8_t tail)
{
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
 * @brief 读取串口(NucleusN 控制手柄)
 *
 * @param head 头帧 冒号 默认为0x3A
 * @param tail 尾帧 固定校验为0x0D
 */
TransferData Data::readNucleusN(uint8_t head, uint8_t tail)
{
    // 接收数据
    vector<ReceiveNucleusN> data = __port.readStruct<ReceiveNucleusN>(head, tail);
    // 判空
    if (!data.empty())
    {
        __receive_nucleusn_data = data.back();
        // 整合得到传递数据
        TransferData transfer_data;
        for (int i = 0; i < 14; i++)
        {
            transfer_data.read2[i] = data.back().read_data[i];
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
        // 目前仅用于 set zero, 保持默认值即可
        // __send_data1.activate_command_h = transfer_data.command1;
        // __send_data1.activate_command_l = transfer_data.command2;
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
        // cout << hex << unsigned(__send_data4.add) << " " << unsigned(__send_data4.byte1) << " " << unsigned(__send_data4.direction_and_speed1) << " "
        //      << unsigned(__send_data4.direction_and_speed2) << " " << unsigned(__send_data4.accelerated_speed) << " " << unsigned(__send_data4.pulse_h)
        //      << " " << unsigned(__send_data4.pulse_l) << " " << unsigned(__send_data4.ver) << endl;
        break;
    }
    case 5:
    {
        // 驱动NucleusN
        // 转化data为ASCII
        vector<uint8_t> data_h = Hex2Ascii(transfer_data.pulse_h);
        __send_data5.data_h[0] = data_h.at(0);
        __send_data5.data_h[1] = data_h.at(1);
        vector<uint8_t> data_l = Hex2Ascii(transfer_data.pulse_l);
        __send_data5.data_l[0] = data_l.at(0);
        __send_data5.data_l[1] = data_l.at(1);

        // 计算checksum
        uint8_t Command = Ascii2Hex(__send_data5.command[0], __send_data5.command[1]);
        uint8_t paramH = Ascii2Hex(__send_data5.param_h[0], __send_data5.param_h[1]);
        uint8_t paramL = Ascii2Hex(__send_data5.param_l[0], __send_data5.param_l[1]);
        uint8_t dataH = Ascii2Hex(__send_data5.data_h[0], __send_data5.data_h[1]);
        uint8_t dataL = Ascii2Hex(__send_data5.data_l[0], __send_data5.data_l[1]);
        // cout << (int)Command << endl;
        // cout << (int)paramH << endl;
        // cout << (int)paramL << endl;
        // cout << (int)dataH << endl;
        // cout << (int)dataL << endl;
        uint8_t checksum = (char)(256 - 1 - (Command + paramH + paramL + dataH + dataL));
        // cout << (int)checksum << endl;

        // 提取checksum
        vector<uint8_t> checksum_vec = Hex2Ascii(checksum);
        __send_data5.checksum[0] = checksum_vec.at(0);
        __send_data5.checksum[1] = checksum_vec.at(1);
        // cout << (int)__send_data5.checksum[0] << (int)__send_data5.checksum[1] << endl;
        __port.writeStruct<SendData5>(__send_data5);
        break;
    }
    case 6:
    {
        // NucleusN 电机行程校准
        __port.writeStruct<SendData6>(__send_data6);
        break;
    }
    default:
        break;
    }
}
