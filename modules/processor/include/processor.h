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
#include "stepping_motor.h"
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

class Face
{
public:
    Face() { cout << "face-constructor" << endl; }
    ~Face() { cout << "face-destructor" << endl; }
    int target_face_label;
    deque<cv::Point2f> face_center;
    cv::Mat detected_faces;
    bool faceDetect(cv::Mat &, cv::Mat &, int &);
    bool isValidFace(cv::Mat &, int);
};

using detect_ptr = shared_ptr<Face>;

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

// class Motor
// {
// public:
//     int init(Data &, TransferData &, TransferData &);
//     int readPulse(Data &);
//     void writePulse(int, Data &);
//     void setZero(Data &);
// };

struct AstraFrame
{
    double timestamp;
    cv::Mat frame;
};

class Frame
{
protected:
    int drop_count; // 面部识别掉帧数
    bool drop_init;
    int detect_count;        // 检测计数器（减少至0时进行一次检测）
    bool detect_init = true; // 初始化标志位（重置计数器）
    int Decider(Face &, Dis &, cv::Mat &, cv::Mat &, int &);
    int disInterPolater(int &);
    void dropProcess(int, Dis &, cv::Mat &);

    detect_ptr __detector;  // test
    motor_ptr __motor;

public:
    double last_color_timestamp;
    double last_depth_timestamp;
    list<rs2::frame> rsColorFrames;
    list<rs2::depth_frame> rsDepthFrames;
    list<AstraFrame> astraDepthFrames, astraColorFrames;
    void rs_read(rs2::pipeline &, rs2::frameset &);
    void astraProcessFrame(Face &, Dis &, int64 &, Data &);
    void rsProcessFrame(Face &, Dis &, int64 &, Data &);
};