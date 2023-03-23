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
    while (1)
    {
        __reader->read();
        cv::Mat depth = __reader->depth * 15;
        cv::Mat color = __reader->color;
        cv::Mat depth_8bit;
        cv::Mat depth_8bit_c3;
        depth.convertTo(depth_8bit, CV_8U, 255.f / 65535.f);
        cv::cvtColor(depth_8bit, depth_8bit_c3, cv::COLOR_GRAY2BGR); // 转为16bit三通道
        cout << "type:" << depth_8bit_c3.type() << endl;

        cv::imshow("depth", depth);
        cv::imshow("depth_8bit", depth_8bit);
        cv::imshow("depth_8bit_c3", depth_8bit_c3);
        char key1 = (char)cv::waitKey(1);
        string filename_side = "../doc/side/side_" + to_string(num) + ".png";
        string filename_back = "../doc/back/back_" + to_string(num) + ".png";
        string filename_down = "../doc/down/down_" + to_string(num) + ".png";
        string filename_sit = "../doc/sit/sit_" + to_string(num) + ".png";
        if (key1 == 's')
        {
            num++;
            cv::imwrite(filename_side, depth);
            cout << "capture side pic " << num << endl;
        }
        if (key1 == 'b')
        {
            num++;
            cv::imwrite(filename_back, depth);
            cout << "capture back pic " << num << endl;
        }
        if (key1 == 'd')
        {
            num++;
            cv::imwrite(filename_down, depth);
            cout << "capture down pic " << num << endl;
        }
        if (key1 == 't')
        {
            num++;
            cv::imwrite(filename_sit, depth);
            cout << "capture sit pic " << num << endl;
        }
        cv::waitKey(5);
    }
}