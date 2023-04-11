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
#include "detector.h"
#include "RsReader.h"

class decider
{
protected:
    bool drop_init;
    bool detect_init = 1;
    int detect_count;
    map<int, int> face_dropcount;
    map<int, int> object_dropcount;
    const int detect_rate = 30 / param.FPS;
    const string logicFile = "../param/logic.names";
    std::vector<string> logic;

public:
    int decide(cv::Mat &, detector_ptr &, dis_ptr &, logic_ptr &, bool, int, bool);
    int objectPerceptron(cv::Mat &, detector_ptr &, dis_ptr &, logic_ptr &, bool);
    int facePerceptron(cv::Mat &, detector_ptr &, dis_ptr &, logic_ptr &, bool);
    int disInterPolater(int &);
    void dropProcess(int, cv::Mat &, dis_ptr &, reader_ptr &);

    inline void faceDropInit(int i) { face_dropcount.at(i) = 0; };
    inline void faceDropInit(int i) { object_dropcount.at(i) = 0; };
    inline void dropInit(map<int, int> map)
    {
        for (int i = 0; i < 100; i++)
            map.at(i) = 0;
    };
};

using decider_ptr = shared_ptr<decider>;
