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
#include "FaceLight.h"
#include "object.h"
#include "ObjectLight.h"
// #include "body.h"
#include "detector.h"
#include "RsReader.h"
#include "LogicTools.h"
#include "param.h"
#include "LensParam.h"
#include "SteppingMotor.h"
#include "NucleusN.h"
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
    logic_ptr __logic;
    reader_ptr __reader;
    dis_ptr __dis, __filter;
    data_ptr __data;
    motor_ptr __motor;

public:
    SubController() = default;

    virtual int init(int64 &, int) = 0;
};

using subcontrol_ptr = shared_ptr<SubController>;
