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

class SingleFace
{
public:
    cv::Point2f center;
    cv::Mat single_face;
};

class SingleObject
{
public:
    cv::Point2f center;
    cv::Rect single_object_box;
};

class Detector
{
    // protected:
public:
    bool draw_face_box = 0;
    bool draw_object_box = 0;
    bool draw_human_box = 0;

    deque<vector<SingleFace>> face;     // 面部中心的时间队列
    deque<vector<SingleObject>> target; // 目标框的时间队列
    deque<vector<int>> target_id;       // 目标框的ID队列
    deque<vector<float>> target_conf;   // 目标框的置信度队列
    int face_label;                     // 锁定的面部在队列中的位置
    int target_label;                   // 锁定的目标在队列中的位置

    virtual bool detect(cv::Mat &) = 0;
    virtual bool drawBox(cv::Mat &) = 0;
    inline bool isValideFace() { return draw_face_box; }
    inline bool isValideObject() { return draw_object_box; }
    inline float getPointDis(const cv::Point2f &pt_1, const cv::Point2f &pt_2)
    {
        return sqrt(pow(pt_1.x - pt_2.x, 2) + pow(pt_1.y - pt_2.y, 2));
    }
};
using detector_ptr = shared_ptr<Detector>;
