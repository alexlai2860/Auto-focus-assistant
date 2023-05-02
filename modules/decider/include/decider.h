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
    bool tracked = 0;     // 是否正在追踪目标
    bool wheel_moved = 0; // 控制手柄滚轮是否转动
    int last_label;
    deque<int> read_result_deque;
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
    int situationJudger(int, int, detector_ptr &, detector_ptr &);

public:
    int decide(cv::Mat &, cv::Mat &, reader_ptr &, detector_ptr &, detector_ptr &, dis_ptr &, logic_ptr &, bool, int, int);
    int objectPerceptron(cv::Mat &, detector_ptr &, dis_ptr &, logic_ptr &, bool, int);
    int facePerceptron(cv::Mat &, detector_ptr &, dis_ptr &, logic_ptr &, bool, int);
    int disInterPolater(int &);   // dis2pulse
    int pulseInterPolater(int &); // pulse2dis
    void dropProcess(int, cv::Mat &, dis_ptr &, reader_ptr &);
};

using decider_ptr = shared_ptr<decider>;
