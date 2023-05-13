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
    int key = 0;
    int lens_num = 0;
    // 初始化
    cout << "即将执行自动校准,输入1确认,输入2跳过:" << endl;
    // cin >> key;
    key = param.AUTO_CAL;
    if (key == 1)
    {
        __data->write(6, writeData);
    }
    // 确定镜头编号，判断是否需要新建镜头
    cout << "请输入镜头编号(1/2/3/4/5) 若新建镜头请按0 回车确认" << endl;
    // cin >> key;
    key = param.LENS_NUM;
    if (key != 0)
    {
        lens_num = key;
        int compensate;
        switch (lens_num)
        {
        // 2023.3.4：改用新的插值算法
        case 1:
            lens_param.A = lens_param.LENS_1.at<double>(0, 0);
            lens_param.B = lens_param.LENS_1.at<double>(0, 1);
            lens_param.C = lens_param.LENS_1.at<double>(0, 2);
            lens_param.D = lens_param.LENS_1.at<double>(0, 3);
            lens_param.E = lens_param.LENS_1.at<double>(0, 4);
            lens_param.F = lens_param.LENS_1.at<double>(0, 5);
            lens_param.G = lens_param.LENS_1.at<double>(0, 6);
            lens_param.INIT_DIS = lens_param.LENS_1.at<double>(0, 7);
            break;
        case 2:
            lens_param.A = lens_param.LENS_2.at<double>(0, 0);
            lens_param.B = lens_param.LENS_2.at<double>(0, 1);
            lens_param.C = lens_param.LENS_2.at<double>(0, 2);
            lens_param.D = lens_param.LENS_2.at<double>(0, 3);
            lens_param.E = lens_param.LENS_2.at<double>(0, 4);
            lens_param.F = lens_param.LENS_2.at<double>(0, 5);
            lens_param.G = lens_param.LENS_2.at<double>(0, 6);
            lens_param.INIT_DIS = lens_param.LENS_2.at<double>(0, 7);
            break;
        case 3:
            lens_param.A = lens_param.LENS_3.at<double>(0, 0);
            lens_param.B = lens_param.LENS_3.at<double>(0, 1);
            lens_param.C = lens_param.LENS_3.at<double>(0, 2);
            lens_param.D = lens_param.LENS_3.at<double>(0, 3);
            lens_param.E = lens_param.LENS_3.at<double>(0, 4);
            lens_param.F = lens_param.LENS_3.at<double>(0, 5);
            lens_param.G = lens_param.LENS_3.at<double>(0, 6);
            lens_param.INIT_DIS = lens_param.LENS_3.at<double>(0, 7);
            break;
        case 4:
            lens_param.A = lens_param.LENS_4.at<double>(0, 0);
            lens_param.B = lens_param.LENS_4.at<double>(0, 1);
            lens_param.C = lens_param.LENS_4.at<double>(0, 2);
            lens_param.D = lens_param.LENS_4.at<double>(0, 3);
            lens_param.E = lens_param.LENS_4.at<double>(0, 4);
            lens_param.F = lens_param.LENS_4.at<double>(0, 5);
            lens_param.G = lens_param.LENS_4.at<double>(0, 6);
            lens_param.INIT_DIS = lens_param.LENS_4.at<double>(0, 7);
            break;
        case 5:
            lens_param.A = lens_param.LENS_5.at<double>(0, 0);
            lens_param.B = lens_param.LENS_5.at<double>(0, 1);
            lens_param.C = lens_param.LENS_5.at<double>(0, 2);
            lens_param.D = lens_param.LENS_5.at<double>(0, 3);
            lens_param.E = lens_param.LENS_5.at<double>(0, 4);
            lens_param.F = lens_param.LENS_5.at<double>(0, 5);
            lens_param.G = lens_param.LENS_5.at<double>(0, 6);
            lens_param.INIT_DIS = lens_param.LENS_5.at<double>(0, 7);
            break;
        default:
            break;
        }
        lens_param.write();
        return lens_num;
    }
    else
    {
        cout << "选择新镜头编号(1/2/3/4/5),若重复将覆盖之前数据 回车确认" << endl;
        cin >> key;
        lens_num = key;
        key = -1;
        lens_param.write();
        return -lens_num; // 此时进入拟合函数
    }
    return 0;
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
    // cout << "position" << position << endl;
    TransferData writeData;
    if (position >= 0)
    {
        uint16_t hex_pos = position;
        uint8_t pos_h = (uint8_t)(hex_pos >> 8);
        uint8_t pos_l = (uint8_t)(hex_pos & 0x00FF);
        // cout << "hex_pos" << (int)hex_pos << endl;
        // cout << "pos_h" << (int)pos_h << endl;
        // cout << "pos_l" << (int)pos_l << endl;
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
 * @return int 1:正常情况
 *             -5:pos和com均未初始化/代码有误
 */
int NucleusN::read()
{
    TransferData read_data;
    read_data = __data->readNucleusN(0x3A, 0x0D);
    bool is_data_empty = !(read_data.com_data_init || read_data.pos_data_init);
    read_datas.push_back(read_data);
    if (read_datas.size() > 20)
    {
        read_datas.pop_front();
    }

    // 先判断读取数据是否为空
    // for (int i = 0; i < 14; i++)
    // {
    //     if (read_data.read2[i] != 0x00)
    //     {
    //         is_data_empty = 0;
    //     }
    // }
    if (is_data_empty)
    {
        // cout << "read-data-empty!!!" << endl;
        return -5;
    }
    else
    {
        // cout << "read_valid";
        // for (int i = 0; i < 14; i++)
        //     cout << hex << read_data.read2[i];
        // cout << endl;
    }

    // 数据非空，进一步判断数据类别
    if (read_data.pos_data_init)
    {
        // :01开头，表示传输编码器数值
        uint8_t num_1 = read_data.read2_position[8];
        uint8_t num_2 = read_data.read2_position[9];
        uint8_t num_3 = read_data.read2_position[10];
        uint8_t num_4 = read_data.read2_position[11];
        uint8_t num_h = __data->Ascii2Hex(num_1, num_2);
        uint8_t num_l = __data->Ascii2Hex(num_3, num_4);
        uint16_t num = num_h << 8 | num_l;
        // cout << "num:" << (int)num << endl;
        if ((int)num >= 0 && (int)num <= 9999)
        {
            key_trigger = 0;
            last_position = (int)num;
            this->position.push_back((int)num);
            if (this->position.size() > 20)
            {
                this->position.pop_front();
            }
            // return (int)num;
        }
        else
        {
            return -5;
        }
    }

    if (read_data.com_data_init)
    {
        int current_command = -5;
        if (read_data.read2_command[0] == '9')
        {
            if (read_data.read2_command[1] == '6')
            {
                // cal指令
                current_command = -1;
            }
        }
        if (read_data.read2_command[0] == '3')
        {
            if (read_data.read2_command[1] == 'F')
            {
                // longcal指令
                current_command = -2;
            }
        }
        if (read_data.read2_command[0] == 'C')
        {
            if (read_data.read2_command[1] == '9')
            {
                if (read_data.read2_command[11] == '1')
                {
                    // recon指令
                    current_command = -3;
                }
                if (read_data.read2_command[11] == '0')
                {
                    // recoff指令
                    current_command = -4;
                }
            }
        }
        if (current_command == -5)
        {
            // 如果还是-5，直接return
            return -5;
        }

        if (this->command.back() != current_command)
        {
            // 和之前的命令不一样:认为出现新命令
            this->last_com_pos = read_data.current_com_pos;
            this->command.push_back(current_command);
            this->command_init = 1;
        }
        else if (this->last_com_pos > read_data.current_com_pos)
        {
            // 比之前的命令位置更近:也认为出现新命令
            this->last_com_pos = read_data.current_com_pos;
            this->command.push_back(current_command);
            this->command_init = 1;
        }

        // else
        // {
        //     this->command_init = 0;
        // }

        if (this->command.size() > 20)
        {
            this->command.pop_front();
        }

        // if (key_trigger == 1)
        // {
        //     // 只有当读取到的数组发生变化时，才会返回指令
        //     if (last_data.read2_command[11] == read_data.read2_command[11])
        //     {
        //         // 否则持续返回位置
        //         return last_position;
        //     }
        // }
        // last_data = read_data;
        // key_trigger = 1;
    }

    return 1;
}

/**
 * @brief 读取指令
 *
 * @return int 0-9999
 */
int NucleusN::readCommand()
{
    // 当command队列不为空，且有新的命令加入队列时，允许返回命令
    if (!this->command.empty() && this->command_init)
    {
        // 并将init置零，等待下次出现新的命令
        this->command_init = 0;
        return command.back();
    }
    return -5;
}

/**
 * @brief 读取位置
 *
 * @return int -1:cal
 *             -2:long-cal
 *             -3:rec-on
 *             -4:rec-off
 */
int NucleusN::readPosition()
{
    if (!this->position.empty())
    {
        return position.back();
    }
    return -5;
}

void NucleusN::setZero() {}
void NucleusN::enable() {}
void NucleusN::test(int x, int y) {}