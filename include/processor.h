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
public:
    deque<int> target_dis;
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
    cv::Mat polyFit(vector<cv::Point2f> &, int n);
    void calibration(cv::VideoCapture &, cv::VideoCapture &, Face &, Dis &, int64 &, Data &);
};

class Frame
{
public:
    double timestamp;
    int drop_count;
    int detect_count;     // 检测计数器（减少至0时进行一次检测）
    bool detect_init = 1; // 初始化标志位（重置计数器）
    cv::Mat frame;
    list<Frame> depth_frames, color_frames;
    void processFrame(cv::VideoCapture &, cv::VideoCapture &, Face &, Dis &, int64 &, Data &);
};
