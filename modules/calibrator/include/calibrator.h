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

#include "dis.h"

using namespace std;

class Calibrator
{
protected:
    vector<cv::Mat> function;
    vector<cv::Point2f> cal_points;

    // motor_ptr __motor = make_shared<SteppingMotor>();

public:
    bool calibrate(int, dis_ptr &, motor_ptr &, rs2::depth_frame &, cv::Mat &);

    cv::Mat polyFit(vector<cv::Point2f> &, int, int);
    // void astraCalibration(int, Dis &, int64 &);
    // int calibratorInit(int64 &, motor_ptr &);
    // void rsCalibration(int, Dis &, int64 &);
    // void rsCalibrationNew(int, Dis &, int64 &);

};

using cal_ptr = shared_ptr<Calibrator>;