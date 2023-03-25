/**
 * @file SubController.h
 * @author 赖建宇 (lai.jianyu@foxmail.com)
 * @brief
 * @version 1.0
 * @date 2023-03-12
 *
 * @copyright Copyright SCUT RobotLab(c) 2021
 *
 */

#pragma once

#include "dis.h"
#include "face.h"
#include "sleep.h"
#include "RsReader.h"
#include "LogicTools.h"
#include "param.h"
#include "LensParam.h"
#include "SteppingMotor.h"
// #include "decider.h"
#include "calibrator.h"

using namespace std;

struct AstraFrame
{
    double timestamp;
    cv::Mat frame;
};

class SubController
{
protected:
    logic_ptr __logic;   // 逻辑处理工具
    reader_ptr __reader; // 相机读取器
    dis_ptr __dis;       // 距离解算和滤波器
    motor_ptr __motor;   // 马达控制器
    // data_ptr __data;  // 马达数据控制器

public:
    SubController() = default;

    virtual int init(int64 &, int) = 0;
};

using subcontrol_ptr = shared_ptr<SubController>;
