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
    KalmanFilter22 num_filter;
    deque<int64> t;
    deque<int64> t2;
    bool __is_filter_init;
    bool __is_filter2_init;
    vector<int> error_dis;

public:
    Dis();
    deque<int> target_dis;
    deque<int> num;
    int last_distance = 0;
    int last_distance2 = 0;

    void updateFilter();
    void updateFilter2();
    bool errorJudge(int);
    int disCalculate(int, cv::Mat &, deque<cv::Point2f> &);
    int kalmanFilter(int);
    bool movDecider(int64 &, deque<cv::Point2f> &);
    void disProcessor();
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
