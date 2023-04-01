/**
 * @file Capturer.cpp
 * @author 赖建宇 (lai.jianyu@foxmail.com)
 * @brief
 * @version 1.0
 * @date 2023-03-23
 *
 * @copyright Copyright SCUT RobotLab(c) 2021
 *
 */
#include "Capturer.h"
#include "param.h"
#include "LensParam.h"
#include <iostream>
#include <fstream>
#include <string>

using namespace std;

int Capturer::init(int64 &t0, int none)
{
    __reader = make_shared<RsReader>();
    __reader->camInit();
    cv::waitKey(2000);
    int num = 0;
    float run_time = 0;
    while (1)
    {
        __reader->read();
        cv::Mat depth = __reader->depth * 16; // 有效距离限制为4.1m
        cv::Mat color = __reader->color;
        cv::Mat depth_8bit;
        cv::Mat depth_8bit_c3;
        depth.convertTo(depth_8bit, CV_8U, 255.f / 65535.f);
        cv::cvtColor(depth_8bit, depth_8bit_c3, cv::COLOR_GRAY2BGR); // 转为8bit三通道
        // cout << "type:" << depth_8bit_c3.type() << endl;

        // cv::imshow("depth", depth);
        // cv::imshow("depth_8bit", depth_8bit);
        cv::imshow("depth_8bit_c3", depth_8bit_c3);
        char key1 = (char)cv::waitKey(1);
        string filename_side_1 = "../doc/side1/3_31_side1_" + to_string(num) + ".jpg";
        string filename_side_2 = "../doc/side2/3_31_side2_" + to_string(num) + ".jpg";
        string filename_back = "../doc/back/3_31_back_" + to_string(num) + ".jpg";
        string filename_sit = "../doc/sit/3_31_sit_" + to_string(num) + ".jpg";
        if (timeTrigger(t0, 3))
        {
            key1 = 't'; // 锁定收集的类别
            if (key1 == 'o')
            {
                num++;
                cv::imwrite(filename_side_1, depth_8bit_c3);
                cout << "capture side one pic " << num << endl;
            }
            if (key1 == 't')
            {
                num++;
                cv::imwrite(filename_side_2, depth_8bit_c3);
                cout << "capture side two pic " << num << endl;
            }
            if (key1 == 'b')
            {
                num++;
                cv::imwrite(filename_back, depth_8bit_c3);
                cout << "capture back pic " << num << endl;
            }
            if (key1 == 's')
            {
                num++;
                cv::imwrite(filename_sit, depth_8bit_c3);
                cout << "capture sit pic " << num << endl;
            }
        }
        cv::waitKey(3);
    }
}

bool Capturer::timeTrigger(int64 &t0, const float fps)
{
    int run_time = 1000 * ((cv::getTickCount() - t0) / cv::getTickFrequency());
    int delta_time = 1000.0 / fps;
    // cout << "runtime" << run_time << endl;
    // cout << "delta_time" << delta_time << endl;
    if ((run_time - last_capture_time) >= delta_time)
    {
        last_capture_time = run_time;
        return 1;
    }
    else
    {
        return 0;
    }
}