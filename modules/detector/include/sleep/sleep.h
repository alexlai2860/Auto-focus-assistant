/**
 * @file sleep.h
 * @author 赖建宇 (lai.jianyu@foxmail.com)
 * @brief
 * @version 1.0
 * @date 2023-03-24
 *
 * @copyright Copyright SCUT RobotLab(c) 2021
 *
 */
#pragma once

#include "dis.h"
#include "param.h"
#include <opencv2/dnn/dnn.hpp>
#include <opencv2/opencv.hpp>
#include <iostream>

using namespace std;

class Sleep
{
public:
    Sleep() = default;
    ~Sleep() = default;
    string current_pose; // 当前睡眠姿态，分为仰卧、侧卧、俯卧、坐姿
    bool is_sleeping;    // 当前是否入睡

    cv::dnn::Net resnet_101;
    void loadModel(const string &);
    void preProcess(const cv::Mat &image, cv::Mat &image_blob);

    int sleepPoseJudge(cv::Mat &);
};

using sleep_ptr = shared_ptr<Sleep>;