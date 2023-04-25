/**
 * @file NucluesN.cpp
 * @author 赖建宇 (lai.jianyu@foxmail.com)
 * @brief
 * @version 1.0
 * @date 2023-04-23
 *
 * @copyright Copyright SCUT RobotLab(c) 2021
 *
 */
#include "NucleusN.h"

using namespace std;

/**
 * @brief 电机初始化
 *
 * @param readData
 * @param writeData
 * @return int
 */
int NucleusN::init(TransferData &readData, TransferData &writeData)
{
    // 初始化
    cout << "即将执行自动校准,输入1并回车以确认:" << endl;
    __logic->waitForNum(1);
    __data->write(6, writeData);
}

/**
 * @brief NucleusN 电机驱动函数
 *
 * @param position 0-9999为电机位置，-1为CAL模式
 * @param speed 预留，可能用于控制电机速度
 */
void NucleusN::write(int position, int speed)
{
    // position = 1999;
    cout << "position" << position << endl;
    TransferData writeData;
    if (position >= 0)
    {
        uint16_t hex_pos = position;
        uint8_t pos_h = (uint8_t)(hex_pos >> 8);
        uint8_t pos_l = (uint8_t)(hex_pos & 0x00FF);
        cout << "hex_pos" << (int)hex_pos << endl;
        cout << "pos_h" << (int)pos_h << endl;
        cout << "pos_l" << (int)pos_l << endl;
        writeData.pulse_h = pos_h;
        writeData.pulse_l = pos_l;
        __data->write(5, writeData);
    }
    if (position == -1)
    {
        __data->write(6, writeData);
    }
}

/**
 * @brief read函数在NucleusN中，用不同的int值表示读取到不同的指令
 *
 * @return int 0-9999:编码器值
 *             -1:cal
 *             -2:long cal
 *             -3:rec on
 *             -4:rec off
 *             -5:empty or error
 */
int NucleusN::read()
{
    TransferData read_data;
    bool is_data_empty = 1;
    read_data = __data->readNucleusN(0x3A, 0x0D);
    read_datas.push_back(read_data);

    // 先判断读取数据是否为空
    for (int i = 0; i < 14; i++)
    {
        if (read_data.read2[i] != 0x00)
        {
            is_data_empty = 0;
        }
    }
    if (is_data_empty)
    {
        cout << "read-data-empty!!!" << endl;
        return -5;
    }
    else
    {
        cout << "read_valid:";
        // for (int i = 0; i < 14; i++)
        // {
        //     cout << hex << read_data.read2[i];
        // }
        cout << endl;
    }

    // 数据非空，进一步判断数据类别
    if (read_data.read2[1] == '1')
    {
        // :01开头，表示传输编码器数值
        uint8_t num_1 = read_data.read2[8];
        uint8_t num_2 = read_data.read2[9];
        uint8_t num_3 = read_data.read2[10];
        uint8_t num_4 = read_data.read2[11];
        uint8_t num_h = __data->Ascii2Hex(num_1, num_2);
        uint8_t num_l = __data->Ascii2Hex(num_3, num_4);
        uint16_t num = num_h << 8 | num_l;
        cout << "num:" << (int)num << endl;
        if ((int)num >= 0 && (int)num <= 9999)
        {
            return (int)num;
        }
        else
        {
            return -5;
        }
    }
    return 0;
}

void NucleusN::setZero() {}
void NucleusN::enable() {}
void NucleusN::test(int x, int y) {}