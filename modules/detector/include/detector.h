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
    SingleFace *forward_face = nullptr;
    SingleFace *backward_face = nullptr;
    int drop_count = 0;
    int face_init_trigger = 0; // 同object的trigger,都加入strict-in策略
    bool detected;             // 用于区分该面部是真实检测出的/虚拟构建的
    float cam_dis;             // 到相机的真实距离
    float forward_dis = -1.f;  // 该帧面部中心点和下一帧对应面部中心点的欧氏距离
    float backward_dis = -1.f; // 该帧面部中心点和前一帧对应面部中心点的欧氏距离
};

class SingleObject
{
public:
    cv::Point2f center;
    cv::Rect single_face_in_object; // (if possible)
    cv::Rect single_object_box;
    SingleObject *forward_object = nullptr;
    SingleObject *backward_object = nullptr;
    Dis dis;              // 距离滤波器
    int drop_count = 0;   // 掉帧计数器
    int init_trigger = 0; // 连续识别到目标才能触发对焦动作，避免误识别导致抽搐
    bool detected;
    int id;
    float conf;
    float cam_dis;             // 到相机的真实距离
    float face_dis;            // (if possible)
    float forward_dis = -1.f;  // 该帧物体中心点和下一帧对应物体中心点的欧氏距离
    float backward_dis = -1.f; // 该帧物体中心点和前一帧对应物体中心点的欧氏距离
};

class Depth
{
public:
    int getPointDepth(const cv::Mat &, const cv::Point2i &);
    int getTargetDepth(const cv::Mat &, const cv::Rect2i &, const int type);
};

class Detector
{
protected:
    Depth depth;

public:
    bool draw_face_box = 0;
    bool draw_object_box = 0;
    bool draw_human_box = 0;

    deque<vector<SingleFace>> face;     // 面部的时间队列
    deque<vector<SingleObject>> target; // 目标的时间队列
    deque<vector<int>> target_id;       // 目标的ID队列
    deque<vector<float>> target_conf;   // 目标的置信度队列
    int face_label;                     // 锁定的面部在队列中的位置
    int target_label;                   // 锁定的目标在队列中的位置

    virtual bool detect(cv::Mat &, cv::Mat &) = 0;
    virtual bool drawBox(cv::Mat &, cv::Mat &) = 0;
    inline bool isValideFace() { return draw_face_box; }
    inline bool isValideObject() { return draw_object_box; }
    inline float getPointDis(const cv::Point2f &pt_1, const cv::Point2f &pt_2)
    {
        return sqrt(pow(pt_1.x - pt_2.x, 2) + pow(pt_1.y - pt_2.y, 2));
    }
};
using detector_ptr = shared_ptr<Detector>;
