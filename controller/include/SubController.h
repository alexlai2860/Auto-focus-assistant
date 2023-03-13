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
#include "RsReader.h"
#include "param.h"
#include "LensParam.h"
#include "SteppingMotor.h"
// #include "calibrator.h"

using namespace std;

struct AstraFrame
{
    double timestamp;
    cv::Mat frame;
};

class SubController
{
public:
    SubController() = default;
    // double last_color_timestamp;
    // double last_depth_timestamp;
    // list<rs2::frame> rsColorFrames;
    // list<rs2::depth_frame> rsDepthFrames;
    // list<AstraFrame> astraDepthFrames, astraColorFrames;

    reader_ptr __reader;

    virtual int CalInit(int64 &) = 0; // todo:合并为一个init 以解决undefined reference问题
    virtual bool FocusInit(int64 &, int &) = 0;
    virtual void rs_read(rs2::pipeline &, rs2::frameset &) = 0;
};

using subcontrol_ptr = shared_ptr<SubController>;
