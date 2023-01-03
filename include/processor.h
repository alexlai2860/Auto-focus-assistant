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
#include "data.h"
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

using namespace std;

class Face
{
public:
    deque<cv::Point2f> face_center;
    cv::Mat detected_faces;
    bool faceDetect(cv::Mat &, cv::Mat &, int &);
};

class Dis
{
    KalmanFilter22 dis_filter;
    deque<int64> t;
    bool __is_filter_init;

public:
    Dis();
    deque<int> target_dis;
    void updateFilter();
    int disCalculate(int, cv::Mat &, deque<cv::Point2f> &);
    bool movDecider(int64 &, deque<cv::Point2f> &);
};

class Motor
{
protected:
    vector<cv::Mat> function;
    vector<cv::Point2f> cal_points;

public:
    int init(Data &, TransferData &, TransferData &);
    int readPulse(Data &);
    void writePulse(int, Data &);
    cv::Mat polyFit(vector<cv::Point2f> &, int, int);
    void calibration(cv::VideoCapture &, cv::VideoCapture &, int, Dis &, int64 &, Data &);
};

class Frame
{
public:
    double timestamp;
    int drop_count; // 面部识别调帧数
    bool drop_init;
    int detect_count;        // 检测计数器（减少至0时进行一次检测）
    bool detect_init = true; // 初始化标志位（重置计数器）
    cv::Mat frame;
    list<Frame> depth_frames, color_frames;
    void processFrame(cv::VideoCapture &, cv::VideoCapture &, Face &, Dis &, int64 &, Data &);
    void dropProcess(int, Dis &, cv::Mat &);
};
