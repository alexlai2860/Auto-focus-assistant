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
#include <iostream>
#include <string>

using namespace std;

int main()
{
    cv::VideoCapture depthStream(cv::CAP_OPENNI2_ASTRA);
    cv::VideoCapture colorStream(4, cv::CAP_V4L2);
    int64 t0 = cv::getTickCount();

    colorStream.set(cv::CAP_PROP_FRAME_WIDTH, 640);
    colorStream.set(cv::CAP_PROP_FRAME_HEIGHT, 480);
    depthStream.set(cv::CAP_PROP_FRAME_WIDTH, 640);
    depthStream.set(cv::CAP_PROP_FRAME_HEIGHT, 480);
    depthStream.set(cv::CAP_PROP_OPENNI2_MIRROR, 0);
    cout << "Depth stream: "
         << depthStream.get(cv::CAP_PROP_FRAME_WIDTH) << "x" << depthStream.get(cv::CAP_PROP_FRAME_HEIGHT)
         << " @" << depthStream.get(cv::CAP_PROP_FPS) << " fps" << endl;

    Face face1;
    Dis dis1;
    Frame cam1;
    TransferData readData, writeData;
    Motor motor;
    Data data1;
    int lens_num = motor.init(data1, readData, writeData); // 区分当前镜头，并初始化电机位置 （ todo:将参数写入yml）
    // lens_num = -1;
    if (lens_num < 0)
    {
        // 创建新镜头函数
        // todo:读取多点电机数据与距离数据，拟合曲线，储存为图片格式
        motor.calibration(colorStream, depthStream, face1, dis1, t0, data1);
        // 最后对lens_num取反，传入processor
        // 打开使能，准备驱动电机
        lens_num = -lens_num;
        writeData.command2 = 0x01;
        data1.write(3, writeData); // 打开使能
        cv::waitKey(3);
    }
    // 每次循环：先读取当前电机位置（注意error判断，可以尝试不用waitkey）
    // 随后根据目标距离解算电机脉冲目标值，求差
    // 写入串口，驱动电机
    // (waitkey后)写入串口，准备读取
    // 顺序有待调整，尽量利用程序本身运行的时间
    if (lens_num > 0)
    {
        writeData.command2 = 0x01;
        data1.write(3, writeData); // 打开使能
        cv::waitKey(3);
        cam1.processFrame(colorStream, depthStream, face1, dis1, t0, data1);
    }
    return 0;
}