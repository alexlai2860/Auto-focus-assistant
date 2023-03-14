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

#include "controller.h"
#include <iostream>
#include <string>

using namespace std;

int main()
{
    int64 t0 = cv::getTickCount();
    // rs2::region_of_interest roi;
    // roi.max_x = 1;

    Controller controller;
    controller.control(t0);
    // Face face1;
    // Dis dis1;
    // Frame cam1;
    // TransferData readData, writeData;
    // // Motor motor;
    // Data data;
    // Calibrator cal;
    // int lens_num = motor.init(data, readData, writeData); // 区分当前镜头，并初始化电机位置 （ todo:将参数写入yml）

    // if (lens_num < 0)
    // {
    //     // 创建新镜头函数
    //     // todo:读取多点电机数据与距离数据，拟合曲线，储存为图片格式
    //     if (param.cam_module == ASTRA)
    //     {
    //         cal.astraCalibration(lens_num, dis1, t0, data);
    //     }
    //     if (param.cam_module == REALSENSE)
    //     {
    //         // cal.rsCalibration(lens_num, dis1, t0, data);
    //         cal.rsCalibrationNew(lens_num, dis1, t0, data);
    //     }
    //     // 最后对lens_num取反，传入processor
    //     cout << "请将对焦环旋转至最近距离处 随后输入1并回车 " << endl;
    //     int key;
    //     while (key != 1)
    //     {
    //         cin >> key;
    //     }
    //     lens_num = -lens_num;
    // }

    return 0;
}