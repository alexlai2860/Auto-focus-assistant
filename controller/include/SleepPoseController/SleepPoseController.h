/**
 * @file SleepPoseController.h
 * @author 赖建宇 (lai.jianyu@foxmail.com)
 * @brief
 * @version 1.0
 * @date 2023-03-24
 *
 * @copyright Copyright SCUT RobotLab(c) 2021
 *
 */

#pragma once

#include "SubController.h"

class SleepPoseController : public SubController
{
protected:
    // vector<cv::Mat> function;
    // vector<cv::Point2f> cal_points;
    // cal_ptr __cal;
    sleep_ptr __sleep;
    deque<int> type;
    int real_type;

public:
    int init(int64 &, int) override;
};
