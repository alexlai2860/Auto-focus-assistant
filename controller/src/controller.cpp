/**
 * @file controller.cpp
 * @author 赖建宇 (lai.jianyu@foxmail.com)
 * @brief
 * @version 1.0
 * @date 2023-03-08
 *
 * @copyright Copyright SCUT RobotLab(c) 2021
 *
 */

#include "controller.h"
#include <iostream>

using namespace std;

void Controller::control(int64 &t0)
{
    // 初始化
    __calibrator = make_shared<Calibrator>();
    int lens_num = __calibrator->calibratorInit(t0);

    Face face1;
    Dis dis1;
    Frame cam1;
    TransferData readData, writeData;
    Data data;

    // 每次循环：先读取当前电机位置
    // 随后根据目标距离解算电机脉冲目标值，求差
    // 写入串口，驱动电机
    // (waitkey后)写入串口，准备读取
    if (lens_num > 0)
    {
        writeData.command2 = 0x01;
        data.write(3, writeData); // 打开使能
        cv::waitKey(3);
        // motor.setZero(data); // 电机置零
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
}