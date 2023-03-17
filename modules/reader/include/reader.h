/**
 * @file reader.h
 * @author 赖建宇 (lai.jianyu@foxmail.com)
 * @brief
 * @version 1.0
 * @date 2023-03-11
 *
 * @copyright Copyright SCUT RobotLab(c) 2021
 *
 */
#pragma once
// #include "MotorData.h"
// #include "SteppingMotor.h"
// #include "KalmanFilterX.hpp"
#include "param.h"
#include "LensParam.h"
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/objdetect.hpp>
#include <ctime>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <atomic>
#include <list>
#include <fstream>
#include <string>
#include <librealsense2/rs.hpp>
#include <librealsense2/rsutil.h>
#include <librealsense2/rs_advanced_mode.hpp>

#define ASTRA 0
#define REALSENSE 1

class reader
{
public:
    double last_color_timestamp;
    double last_depth_timestamp;
    list<rs2::frame> rsColorFrames;
    list<rs2::depth_frame> rsDepthFrames;
    
    reader() = default;
    cv::Mat color;
    cv::Mat depth;

    virtual bool camInit() = 0;
    virtual bool read() = 0;
};
using reader_ptr = shared_ptr<reader>;