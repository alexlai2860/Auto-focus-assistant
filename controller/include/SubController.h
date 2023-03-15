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
    reader_ptr __reader;
    dis_ptr __dis;
    data_ptr __data;
    motor_ptr __motor;

public:
    SubController() = default;

    virtual int CalInit(int64 &) = 0;
    virtual bool FocusInit(int64 &, int &) = 0;
};

using subcontrol_ptr = shared_ptr<SubController>;
