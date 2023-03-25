/**
 * @file SleepPoseController.cpp
 * @author 赖建宇 (lai.jianyu@foxmail.com)
 * @brief
 * @version 1.0
 * @date 2023-03-24
 *
 * @copyright Copyright SCUT RobotLab(c) 2021
 *
 */
#include "SleepPoseController.h"

using namespace std;

int SleepPoseController::init(int64 &t0, int none)
{
    __sleep = make_shared<Sleep>();
    __reader = make_shared<RsReader>();

    __reader->camInit();
    cv::waitKey(2000);
    while (1)
    {
        int64 t1 = cv::getTickCount();
        __reader->read();
        cv::Mat depth = __reader->depth * 15;
        cv::Mat color = __reader->color;
        cv::Mat depth_8bit, depth_8bit_c3;
        depth.convertTo(depth_8bit, CV_8U, 255.f / 65535.f);
        cv::cvtColor(depth_8bit, depth_8bit_c3, cv::COLOR_GRAY2BGR);

        // __sleep->sleepPoseJudge(depth_8bit_c3);

        color = cv::imread("../doc/test.jpg");
        __sleep->sleepPoseJudge(color);

        cv::imshow("color", color);
        cv::imshow("depth", depth_8bit_c3);

        int key = cv::waitKey(1);
        float run_time = 1000 * ((cv::getTickCount() - t1) / cv::getTickFrequency());
        cout << "run time = " << run_time << " ms" << endl;
    }
}