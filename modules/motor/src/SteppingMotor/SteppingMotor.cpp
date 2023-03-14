/**
 * @file stepping_motor.cpp
 * @author 赖建宇 (lai.jianyu@foxmail.com)
 * @brief 
 * @version 1.0
 * @date 2023-03-07
 * 
 * @copyright Copyright SCUT RobotLab(c) 2021
 * 
 */


#include "SteppingMotor.h"
#include "LensParam.h"
#include "param.h"

using namespace std;

int SteppingMotor::init(Data &data, TransferData &readData, TransferData &writeData)
{
    writeData.command2 = 0x00;
    data.write(3, writeData); // 关闭使能
    int key = 0, lens_num = 0;
    cout << "请将对焦环旋转至最近距离处 随后输入1并回车 " << endl;
    while (key != 1)
    {
        cin >> key;
    }
    // while (1)
    // {
    //     cv::Mat test = cv::imread("../SRVL.png", 0);
    //     cv::imshow("test", test);
    //     char key1 = (char)cv::waitKey(1);
    //     if (key1 == '1')
    //     {
    //         break;
    //     }
    //     continue;
    // }
    this->setZero(data);

    key = -1;
    cout << "请输入镜头编号(1/2/3/4/5) 若新建镜头请按0 回车确认" << endl;
    cin >> key;
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

int SteppingMotor::readPulse(Data &data)
{
    TransferData readData, writeData;
    int32_t pulse = 4000000;
    int round = 0;
    while (pulse >= 4000000)
    {
        writeData.command1 = 0x33;
        data.write(2, writeData);
        cv::waitKey(param.WAIT_TIME); // 限制发送速率，根据电脑运行速度和波特率进行调整(默认为3,默认比特率为115200)
        readData = data.read(0x01, 0x6B);
        // data.write(2, writeData);
        pulse = (((int32_t)readData.read1[0] << 24) |
                 ((int32_t)readData.read1[1] << 16) |
                 ((int32_t)readData.read1[2] << 8) |
                 ((int32_t)readData.read1[3] << 0));
        // cout << "current pulse : " << signed(pulse) << endl;
        round++;
        if (round >= 10)
        {
            cout << "READ ERROR" << endl;
            return 4000000;
        }
    }
    return signed(pulse);
}

void SteppingMotor::writePulse(int pulse_num, Data &data)
{
    TransferData writeData, readData;
    // writeData.command2 = 0x01;
    // data.write(3, writeData); // 打开使能
    // cv::waitKey(3);
    // step1 确定正方向和旋转方向(待观察)
    bool positive_direction = (lens_param.G - lens_param.A > 0);
    bool direction = 0;
    if (pulse_num < 0)
    {
        pulse_num = -pulse_num;
        direction = 0;
    }
    else
    {
        direction = 1;
    }

    // step2 确定脉冲数目 : 十进制转十六进制
    uint32_t u32 = uint32_t(pulse_num);
    // cout <<  << endl;
    uint8_t pulse[4] = {0};
    pulse[0] = u32 & 0xFF;
    pulse[1] = (u32 >> 8) & 0xFF;
    pulse[2] = (u32 >> 16) & 0xFF;
    pulse[3] = (u32 >> 24) & 0xFF;

    if (direction == 1)
    {
        writeData.direction_and_speed1 = 0x10;
        cout << "顺时针" << endl;
    }
    else
    {
        writeData.direction_and_speed1 = 0x00;
        cout << "逆时针" << endl;
    }
    writeData.direction_and_speed2 = uint8_t(param.SPEED);
    writeData.pulse_h = pulse[2];
    writeData.pulse_m = pulse[1];
    writeData.pulse_l = pulse[0];

    data.write(4, writeData);
}

void SteppingMotor::setZero(Data &data)
{
    // set zero
    TransferData writeData;
    data.write(1, writeData);
    cv::waitKey(5);
}