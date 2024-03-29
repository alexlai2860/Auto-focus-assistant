/**
 * @file FocusController.h
 * @author 赖建宇 (lai.jianyu@foxmail.com)
 * @brief
 * @version 1.0
 * @date 2023-03-11
 *
 * @copyright Copyright SCUT RobotLab(c) 2021
 *
 */

#pragma once

#include "SubController.h"
#include "decider.h"

class FocusController : public SubController
{
protected:
    // int drop_count; // 面部识别掉帧数
    // bool drop_init;
    // int detect_count;        // 检测计数器（减少至0时进行一次检测）
    // bool detect_init = true; // 初始化标志位（重置计数器）
    // int Decider(cv::Mat &, cv::Mat &, int &);
    // int disInterPolater(int &);
    // void dropProcess(int, cv::Mat &);
    decider_ptr __decider;
    detector_ptr __face;
    detector_ptr __object;
    detector_ptr __detector;

    bool face_trigger;
    bool forced_drop_trigger = 1; // 默认为0-10m重投影模式
    bool MF_trigger = 0;
    int MF_init_result = -1;
    int last_target_pulse;

    cv::Mat reprojected_depth;
    cv::Mat mixed_output;
    void depthReProjection(cv::Mat &depth, int, int);
    void colorDepthMix(cv::Mat &depth, cv::Mat &color);
    void handUnitController();
    float lagrange(float t, int n, int flag);

public:
    int init(int64 &, int) override;
    void astraProcessFrame(int64 &);
    void rsProcessFrame(int64 &);
};