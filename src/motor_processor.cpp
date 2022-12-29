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
#include "lens_param.h"
#include "param.h"

using namespace std;

int Motor::init(Data &data, TransferData &readData, TransferData &writeData)
{
    writeData.command2 = 0x00;
    data.write(3, writeData); // 关闭使能
    int key = 0, lens_num = 0;
    cout << "请将对焦环旋转至最近距离处 随后输入1并回车 " << endl;
    while (key != 1)
    {
        cin >> key;
    }
    int init_pulse = this->readPulse(data);
    int compensate = init_pulse - lens_param.INIT_PULSE_1; // 补偿脉冲，用于矫正拟合函数的常数项
    lens_param.INIT_PULSE_1 = init_pulse;
    lens_param.write();

    // cv::waitKey(3);
    // this->writePulse(3000, data);

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
        key = -1;
        cout << "请将对焦环旋转至最远距离处 随后输入1并回车" << endl;
        while (key != 1)
        {
            cin >> key;
        }
        int infinit_pulse = this->readPulse(data);
        lens_param.INFINIT_PULSE_1 = infinit_pulse;
        lens_param.write();
        return -lens_num; // 此时进入拟合函数
    }
    return 0;
}

int Motor::readPulse(Data &data)
{
    TransferData readData, writeData;
    int32_t pulse = 4000000;
    int round = 0;
    while (pulse >= 4000000)
    {
        writeData.command1 = 0x33;
        data.write(2, writeData);
        cv::waitKey(3); // 限制发送速率，根据电脑运行速度和波特率进行调整(默认为3,默认比特率为115200)(update:取消while循环)
        readData = data.read(0x01, 0x6B);
        // data.write(2, writeData);
        pulse = (((int32_t)readData.read1[0] << 24) |
                 ((int32_t)readData.read1[1] << 16) |
                 ((int32_t)readData.read1[2] << 8) |
                 ((int32_t)readData.read1[3] << 0));
        cout << "current pulse : " << signed(pulse) << endl;
        round++;
        if (round >= 10)
        {
            cout << "ERROR" << endl;
            return 4000000;
        }
    }
    return signed(pulse);
}

void Motor::writePulse(int pulse_num, Data &data)
{
    TransferData writeData, readData;
    writeData.command2 = 0x01;
    data.write(3, writeData); // 打开使能
    cv::waitKey(3);
    // step1 确定正方向和旋转方向(待观察)
    int positive_direction = lens_param.INFINIT_PULSE_1 - lens_param.INIT_PULSE_1;
    bool direction = 0;
    if (positive_direction < 0 && pulse_num < 0)
    {
        pulse_num = -pulse_num;
        direction = 0;
    }
    else if (positive_direction > 0 && pulse_num < 0)
    {
        pulse_num = -pulse_num;
        direction = 0;
    }
    else if (positive_direction > 0 && pulse_num > 0)
    {
        direction = 0;
    }
    else
    {
        direction = 0;
    }

    // step2 确定脉冲数目 : 十进制转十六进制
    uint32_t u32 = uint32_t(pulse_num);
    // cout <<  << endl;
    uint8_t pulse[4] = {0};
    pulse[0] = u32 & 0xFF;
    pulse[1] = (u32 >> 8) & 0xFF;
    pulse[2] = (u32 >> 16) & 0xFF;
    pulse[3] = (u32 >> 24) & 0xFF;

    if (direction = 1)
    {
        writeData.direction_and_speed1 = 0x01;
    }
    else
    {
        writeData.direction_and_speed1 = 0x11;
    }
    writeData.direction_and_speed2 = 0xFF;
    writeData.pulse_h = pulse[2];
    writeData.pulse_m = pulse[1];
    writeData.pulse_l = pulse[0];

    data.write(4, writeData);
}

cv::Mat Motor::polyFit(vector<cv::Point2f> &points, int n)
{
    // test - 输入拟合点
    // std::vector<cv::Point> points;
    points.push_back(cv::Point(100., 58.));
    points.push_back(cv::Point(150., 70.));
    points.push_back(cv::Point(200., 90.));
    points.push_back(cv::Point(252., 140.));
    points.push_back(cv::Point(300., 220.));
    points.push_back(cv::Point(350., 400.));

    cv::Mat curve;
    // Number of key points
    int N = points.size();
    // 构造矩阵X
    cv::Mat X = cv::Mat::zeros(n + 1, n + 1, CV_64FC1);
    for (int i = 0; i < n + 1; i++)
    {
        for (int j = 0; j < n + 1; j++)
        {
            for (int k = 0; k < N; k++)
            {
                X.at<double>(i, j) = X.at<double>(i, j) +
                                     std::pow(points[k].x, i + j);
            }
        }
    }
    // 构造矩阵Y
    cv::Mat Y = cv::Mat::zeros(n + 1, 1, CV_64FC1);
    for (int i = 0; i < n + 1; i++)
    {
        for (int k = 0; k < N; k++)
        {
            Y.at<double>(i, 0) = Y.at<double>(i, 0) +
                                 std::pow(points[k].x, i) * points[k].y;
        }
    }

    curve = cv::Mat::zeros(n + 1, 1, CV_64FC1);
    // 求解矩阵A
    cv::solve(X, Y, curve, cv::DECOMP_LU);
    // cout << setprecision(5) << curve.at<double>(0,0) << endl;
    lens_param.A_1 = curve.at<double>(0, 3);
    lens_param.B_1 = curve.at<double>(0, 2);
    lens_param.C_1 = curve.at<double>(0, 1);
    lens_param.D_1 = curve.at<double>(0, 0);
    lens_param.write();
    cout << "curve = " << curve << endl;
    return curve;
}