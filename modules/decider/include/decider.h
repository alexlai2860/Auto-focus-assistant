/**
 * @file decider.h
 * @author 赖建宇 (lai.jianyu@foxmail.com)
 * @brief
 * @version 1.0
 * @date 2023-03-17
 *
 * @copyright Copyright SCUT RobotLab(c) 2021
 *
 */
#pragma once

#include "dis.h"
#include "face.h"
#include "object.h"
#include "RsReader.h"

class decider
{
protected:
    bool drop_init;
    int drop_count;
    bool detect_init = 1;
    int detect_count;
    const int detect_rate = 30 / param.FPS;

public:
    int decide(cv::Mat &, cv::Mat &, face_ptr &, object_ptr &, dis_ptr &, reader_ptr &);
    int disInterPolater(int &);
    void dropProcess(int, cv::Mat &, dis_ptr &, reader_ptr &);
};

using decider_ptr = shared_ptr<decider>;
