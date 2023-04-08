/**
 * @file detector.h
 * @author 赖建宇 (lai.jianyu@foxmail.com)
 * @brief
 * @version 1.0
 * @date 2023-04-08
 *
 * @copyright Copyright SCUT RobotLab(c) 2021
 *
 */

#pragma once

#include "dis.h"
#include "param.h"
#include <iostream>

class Detector
{
protected:
    bool draw_face_box = 0;
    bool draw_object_box = 0;
    bool draw_human_box = 0;


public:
    deque<cv::Point2f> face_center;
    deque<cv::Point2f> target_center;
    int target_label;

    virtual bool detect(cv::Mat &, int &) = 0;
    virtual bool drawBox(cv::Mat &) = 0;
    inline bool isValideFace() { return draw_face_box; }
    inline bool isValideObject() { return draw_object_box; }
};
using detector_ptr = shared_ptr<Detector>;