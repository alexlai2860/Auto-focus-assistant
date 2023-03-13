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

class FocusController : public SubController
{
protected:
    int drop_count; // 面部识别掉帧数
    bool drop_init;
    int detect_count;        // 检测计数器（减少至0时进行一次检测）
    bool detect_init = true; // 初始化标志位（重置计数器）
    int Decider(Face &, Dis &, cv::Mat &, cv::Mat &, int &);
    int disInterPolater(int &);
    void dropProcess(int, Dis &, cv::Mat &);

    detect_ptr __detector; // test
    motor_ptr __motor;

public:
    bool FocusInit(int64 &, int &) override;
    int CalInit(int64 &) override;
    void rs_read(rs2::pipeline &, rs2::frameset &) override;
    void astraProcessFrame(Face &, Dis &, int64 &, Data &);
    void rsProcessFrame(Face &, Dis &, int64 &, Data &);
};