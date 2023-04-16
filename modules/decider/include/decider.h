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
    bool isDropInit = 0;
    bool detect_init = 1;
    int detect_count;
    map<int, int> face_dropcount;
    map<int, int> object_dropcount;
    const int detect_rate = 30 / param.FPS;
    const string logicFile = "../param/logic.names";
    std::vector<string> logic;

    bool isSameFace(SingleFace &last_face, SingleFace &current_face, logic_ptr &);
    bool isSameObject(SingleObject &last_object, SingleObject &current_object, logic_ptr &);
    inline void faceDropInit(int i) { face_dropcount.at(i) = 0; };
    inline void objectDropInit(int i) { object_dropcount.at(i) = 0; };
    inline void dropInit(map<int, int> map)
    {
        for (int i = 0; i < 50; i++)
            map[i] = 0;
    };

public:
    int decide(cv::Mat &, cv::Mat &, reader_ptr &, detector_ptr &, dis_ptr &, logic_ptr &, bool, int, bool);
    int objectPerceptron(cv::Mat &, detector_ptr &, dis_ptr &, logic_ptr &, bool, int);
    int facePerceptron(cv::Mat &, detector_ptr &, dis_ptr &, logic_ptr &, bool, int);
    int disInterPolater(int &);
    void dropProcess(int, cv::Mat &, dis_ptr &, reader_ptr &);
};

using decider_ptr = shared_ptr<decider>;
