/**
 * @file FocusController.cpp
 * @author 赖建宇 (lai.jianyu@foxmail.com)
 * @brief
 * @version 1.0
 * @date 2023-03-12
 *
 * @copyright Copyright SCUT RobotLab(c) 2021
 *
 */

#include "FocusController.h"
#include "param.h"
#include "LensParam.h"
#include <iostream>
#include <fstream>
#include <string>

using namespace std;

int FocusController::init(int64 &t0, int lens_num)
{
    Face face1;
    Dis dis1;
    TransferData writeData;
    __data = make_shared<Data>();
    __dis = make_shared<Dis>();
    __motor = make_shared<SteppingMotor>();
    __logic = make_shared<LogicTools>();
    // __decider = make_shared<decider>();

    // 每次循环：先读取当前电机位置
    // 随后根据目标距离解算电机脉冲目标值，求差
    // 写入串口，驱动电机
    // (waitkey后)写入串口，准备读取
    if (lens_num > 0)
    {
        __motor->enable();
        // motor.setZero(data); // 电机置零
        if (param.cam_module == ASTRA)
        {
            // astraProcessFrame(t0);
        }
        if (param.cam_module == REALSENSE)
        {
            rsProcessFrame(t0);
        }
    }
    else
    {
        cout << "lens_num_error" << endl;
    }
    return 1;
}

/**
 * @brief Realsense相机 帧处理函数
 *
 * @param face
 * @param dis
 * @param t0
 * @param data
 */
void FocusController::rsProcessFrame(int64 &t0)
{
    int key;
    int round = 0;
    int fps = param.FPS; // support 1,2,3,5,6,10,15,30
    int detect_rate = 30 / fps;
    __reader = make_shared<RsReader>();
    __decider = make_shared<decider>();
    __face = make_shared<Face>();
    __object = make_shared<ObjectLight>();

    // 相机初始化
    __reader->camInit();

    // 在指定时间后再进入循环，避免相机初始化未完成导致程序崩溃
    cv::waitKey(2000);

    while (key != 27)
    {
        int64 t1 = cv::getTickCount();
        __reader->read();

        int DIS = 0;
        int detect_flag = 0;
        bool detected = 0;
        bool preserve = 0;

        cv::Mat d16, dColor;
        cv::Mat color = __reader->color;
        // cv::Mat color_copy = color;
        cv::Mat color_copy;
        color.copyTo(color_copy);
        cv::Mat depth = __reader->depth;

        // 绘制目标框
        cout << "vf" << __face->isValideFace() << endl;
        if (__face->isValideFace())
        {
            __face->drawBox(color, depth);
            cout << "face" << endl;
        }
        if (__object->isValideObject())
        {
            __object->drawBox(color, depth);
            cout << "obj" << endl;
        }

        // 检测器，判断该帧是否需要进行目标检测
        if (__logic->timeTrigger(t0, 10))
        {
            // detected = __face->detect(color_copy, depth);
            // __detector = __face;
            detected = __object->detect(color_copy, depth);
            __detector = __object;
            // detect_flag = 1;
            detect_flag = 2;
        }
        else
        {
            // detect_flag = 0;
            detect_flag = 3;
        }

        // 简易追踪器 & 掉帧/对焦策略处理器 & 距离解算器
        DIS = __decider->decide(d16, color, __reader, __detector, __dis, __logic, detected, detect_flag, preserve);

        // 读取当前脉冲值
        // int current_pulse = __motor->readPulse();
        int current_pulse = 0;

        // 计算目标脉冲值-方案一:使用四次函数拟合，五次函数不稳定，A暂时废弃
        // int target_pulse = (lens_param.B * pow(DIS, 4) + lens_param.C * pow(DIS, 3) + lens_param.D * pow(DIS, 2) + lens_param.E * DIS + lens_param.F);

        // 计算目标脉冲值-方案二:使用插值法
        int target_pulse = __decider->disInterPolater(DIS);
        cv::putText(color, cv::format("%d", target_pulse), cv::Point2i(15, 60), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 0, 0), 2);

        // 计算差值，写入串口，同时进行异常处理，驱动镜头
        // __motor->writePulse((target_pulse - current_pulse));
        __motor->test(0,0);

        // 输出彩色图和深度图
        imshow("Depth", depth * 10);
        imshow("Color", color);
        // imshow("color-copy", color_copy);

        // 计算运行时间
        int key = cv::waitKey(1);
        float run_time = 1000 * ((cv::getTickCount() - t1) / cv::getTickFrequency());
        cout << "run time = " << run_time << " ms" << endl;
    }
}
