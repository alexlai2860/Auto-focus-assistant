/**
 * @file calibrator.h
 * @author 赖建宇 (lai.jianyu@foxmail.com)
 * @brief
 * @version 1.0
 * @date 2023-01-06
 *
 * @copyright Copyright SCUT RobotLab(c) 2021
 *
 */

#pragma once

#include"processor.h"

using namespace std;

class Calibrator
{
protected:
    vector<cv::Mat> function;
    vector<cv::Point2f> cal_points;

public:
    cv::Mat polyFit(vector<cv::Point2f> &, int, int);
    void astraCalibration(int, Dis &, int64 &, Data &);
    void rsCalibration(int, Dis &, int64 &, Data &);
};