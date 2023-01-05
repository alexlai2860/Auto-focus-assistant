/**
 * @file realsense_reader.h
 * @author 赖建宇 (lai.jianyu@foxmail.com)
 * @brief
 * @version 1.0
 * @date 2023-01-05
 *
 * @copyright Copyright SCUT RobotLab(c) 2021
 *
 */

#pragma once

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
#include <librealsense2/rs.hpp>

class RealsenseReader
{
public:
    void rs_read();
};