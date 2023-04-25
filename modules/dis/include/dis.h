/**
 * @file Frame.h
 * @author 赖建宇 (lai.jianyu@foxmail.com)
 * @brief
 * @version 1.0
 * @date 2022-09-24
 *
 * @copyright Copyright SCUT RobotLab(c) 2021
 *
 */

#pragma once

// #include "face_process.h"
#include "MotorData.h"
#include "SteppingMotor.h"
#include "KalmanFilterX.hpp"
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/objdetect.hpp>
#include <ctime>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <atomic>
#include <list>
#include <librealsense2/rs.hpp>
#include <librealsense2/rsutil.h>
#include <librealsense2/rs_advanced_mode.hpp>

#define ASTRA 0
#define REALSENSE 1

using namespace std;

class Dis
{
protected:
    KalmanFilter22 dis_filter;
    deque<int64> t;
    bool __is_filter_init;
    vector<int> error_dis;

public:
    Dis();
    deque<int> target_dis;
    void updateFilter();
    void disProcessor();
    bool errorJudge(int);
    int disCalculate(int, cv::Mat &, deque<cv::Point2f> &);
    bool movDecider(int64 &, deque<cv::Point2f> &);
};

using dis_ptr = shared_ptr<Dis>;

// class Motor
// {
// public:
//     int init(Data &, TransferData &, TransferData &);
//     int read(Data &);
//     void write(int, Data &);
//     void setZero(Data &);
// };
