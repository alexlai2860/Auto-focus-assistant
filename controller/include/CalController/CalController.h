/**
 * @file cal_controller.h
 * @author 赖建宇 (lai.jianyu@foxmail.com)
 * @brief
 * @version 1.0
 * @date 2023-03-10
 *
 * @copyright Copyright SCUT RobotLab(c) 2021
 *
 */
#pragma once

#include "SubController.h"

class CalController : public SubController
{
protected:
    vector<cv::Mat> function;
    vector<cv::Point2f> cal_points;
    cal_ptr __cal;

public:
    int CalInit(int64 &) override;
    bool FocusInit(int64 &, int &) override;
    void astraCalibration(int, Dis &, int64 &, Data &);
    void rsCalibration(int, Dis &, int64 &, Data &);
};

// using cal_ptr = shared_ptr<Calibrator>;