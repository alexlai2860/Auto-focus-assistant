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
    __logic = make_shared<LogicTools>();

    __reader->camInit();
    cv::waitKey(2000);
    while (1)
    {
        int64 t1 = cv::getTickCount();
        __reader->read();
        cv::Mat depth = __reader->depth * 20;
        cv::Mat color = __reader->color;
        cv::Mat depth_8bit, depth_8bit_c3;
        depth.convertTo(depth_8bit, CV_8U, 255.f / 65535.f);
        // cout << "depth_8bit-type " << depth_8bit.type() << endl;
        cv::cvtColor(depth_8bit, depth_8bit_c3, cv::COLOR_GRAY2BGR);

        // __sleep->sleepPoseJudge(depth_8bit_c3);

        // color = cv::imread("../doc/back_70.jpg");
        // cv::Mat gray;
        // cv::cvtColor(color, gray, cv::COLOR_BGR2GRAY);
        // cout << "gray-type " << gray.type() << endl;
        if (__logic->timeTrigger(t0, 5))
        {
            type.push_back(__sleep->sleepPoseJudge(depth_8bit));
            if (type.size() > 10)
            {
                type.pop_front();
            }
        }

        if (!type.empty())
        {
            real_type = type.back();
            cout << "back " << real_type << endl;
            // 判断是否进入新的姿态(待优化)
            if (type.size() == 10)
            {
                real_type = __logic->maxDequeItem(type, 4);
            }
            cout << "real " << real_type << endl;
            switch (real_type)
            {
            case -1:
                cv::putText(depth_8bit, "gesture changing", cv::Point2i(param.RS_width / 2, param.RS_height - 50), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(200, 200, 255), 2);
                break;
            case 0:
                cv::putText(depth_8bit, "lying back", cv::Point2i(param.RS_width / 2, param.RS_height - 50), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(200, 200, 255), 2);
                break;
            case 1:
                cv::putText(depth_8bit, "lying on the side 1", cv::Point2i(param.RS_width / 2, param.RS_height - 50), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(200, 200, 255), 2);
                break;
            case 2:
                cv::putText(depth_8bit, "lying on the side 2", cv::Point2i(param.RS_width / 2, param.RS_height - 50), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(200, 200, 255), 2);
                break;
            case 3:
                cv::putText(depth_8bit, "sitting", cv::Point2i(param.RS_width / 2, param.RS_height - 50), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(200, 200, 255), 2);
                break;

            default:
                break;
            }
        }

        cv::imshow("color", color);
        cv::namedWindow("depth");
        cv::resizeWindow("depth", 1920, 1080);
        cv::resize(depth_8bit, depth_8bit,cv::Size(1696,960));
        cv::imshow("depth", depth_8bit);

        int key = cv::waitKey(1);
        float run_time = 1000 * ((cv::getTickCount() - t1) / cv::getTickFrequency());
        cout << "run time = " << run_time << " ms" << endl;
    }
}