/**
 * @file motor_processor.cpp
 * @author 赖建宇 (lai.jianyu@foxmail.com)
 * @brief
 * @version 1.0
 * @date 2022-12-23
 *
 * @copyright Copyright SCUT RobotLab(c) 2021
 *
 */

#include "processor.h"
#include "param.h"

using namespace std;

int Motor::init(Data &data, TransferData &readData, TransferData &writeData)
{
    writeData.command2 = 0x00;
    data.write(3, writeData); // 关闭使能
    int key, lens_num;
    cout << "请将对焦环旋转至最近距离处 随后输入1并回车 " << endl;
    while (key != 1)
    {
        cin >> key;
    }
    this->readPulse(data, readData, writeData);
    this->writePulse(100, data, readData, writeData);

    key = -1;
    cout << "请输入镜头编号(1/2/3/4/5) 若新建镜头请按0 回车确认" << endl;
    cin >> key;
    if (key != 0)
    {
        lens_num = key;
        return lens_num;
    }
    else
    {
        cout << "选择新镜头编号(1/2/3/4/5),若重复将覆盖之前数据 回车确认" << endl;
        cin >> key;
        lens_num = key;
        return -lens_num;
    }
    return 0;
}

void Motor::readPulse(Data &data, TransferData &readData, TransferData &writeData)
{
    writeData.command1 = 0x33;
    data.write(2, writeData);
    int round = 0;
    while (readData.read1[0] == 0 && readData.read1[1] == 0 && readData.read1[2] == 0 && readData.read1[3] == 0)
    {
        cv::waitKey(3); // 限制发送速率，根据电脑运行速度和波特率进行调整(默认为3,默认比特率为115200)
        round++;
        readData = data.read(0x01, 0x6B);
        // if (round % 20 == 0)
        data.write(2, writeData);
    }
    int32_t init_pulse = (((int32_t)readData.read1[0] << 24) |
                          ((int32_t)readData.read1[1] << 16) |
                          ((int32_t)readData.read1[2] << 8) |
                          ((int32_t)readData.read1[3] << 0));
    cout << "current pulse : " << signed(init_pulse) << endl;
    if (signed(init_pulse) > 4000000)
    {
        cout << "ERROR" << endl;
    }
}

void Motor::writePulse(int pulse_num, Data &data, TransferData &readData, TransferData &writeData)
{
    cout << uint16_t(pulse_num) << endl; // todo : 十进制转十六进制
    writeData.direction_and_speed1 = 0x04;
    writeData.direction_and_speed2 = 0xFF;
    writeData.pulse_h = 0x00;
    writeData.pulse_m = 0x00;
    writeData.pulse_l = 0x64;
    data.write(4, writeData);
}