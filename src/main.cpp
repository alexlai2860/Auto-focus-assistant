/**
 * @file main.cpp
 * @author 赖建宇 (lai.jianyu@foxmail.com)
 * @brief
 * @version 1.0
 * @date 2022-11-29
 *
 * @copyright Copyright SCUT RobotLab(c) 2021
 *
 */

#include "processor.h"
#include "calibrator.h"
#include <iostream>
#include <string>

using namespace std;

int main()
{
    int64 t0 = cv::getTickCount();

    Face face1;
    Dis dis1;
    Frame cam1;
    TransferData readData, writeData;
    Motor motor;
    Data data;
    Calibrator cal;
    int lens_num = motor.init(data, readData, writeData); // 区分当前镜头，并初始化电机位置 （ todo:将参数写入yml）

    if (lens_num < 0)
    {
        // 创建新镜头函数
        // todo:读取多点电机数据与距离数据，拟合曲线，储存为图片格式
        if (param.cam_module == ASTRA)
        {
            cal.astraCalibration(lens_num, dis1, t0, data);
        }
        if (param.cam_module == REALSENSE)
        {
            cal.rsCalibration(lens_num, dis1, t0, data);
        }
        // 最后对lens_num取反，传入processor
        cout << "请将对焦环旋转至最近距离处 随后输入1并回车 " << endl;
        int key;
        while (key != 1)
        {
            cin >> key;
        }
        lens_num = -lens_num;
    }
    // 每次循环：先读取当前电机位置
    // 随后根据目标距离解算电机脉冲目标值，求差
    // 写入串口，驱动电机
    // (waitkey后)写入串口，准备读取
    // 各项指令顺序有待调整，尽量利用程序本身运行的时间
    if (lens_num > 0)
    {
        writeData.command2 = 0x01;
        data.write(3, writeData); // 打开使能
        cv::waitKey(3);
        if (param.cam_module == ASTRA)
        {
            cam1.astraProcessFrame(face1, dis1, t0, data);
        }
        if (param.cam_module == REALSENSE)
        {
            cam1.rsProcessFrame(face1, dis1, t0, data);
        }
    }
    else
    {
        cout << "lens_num_error" << endl;
    }
    return 0;
}