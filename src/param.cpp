/**
 * @file param.cpp
 * @author 赖建宇 (lai.jianyu@foxmail.com)
 * @brief
 * @version 1.0
 * @date 2022-12-09
 *
 * @copyright Copyright SCUT RobotLab(c) 2021
 *
 */

#include "param.h"
#include <opencv2/core/persistence.hpp>

using namespace std;

Param::Param(const string &param)
{
    param_path = param;
    cv::FileStorage arm(param, cv::FileStorage::READ);
    assert(arm.isOpened());

    arm["FPS"] >> FPS;
    arm["DIS_DEQUE"] >> DIS_DEQUE;
    arm["FACE_DEQUE"] >> FACE_DEQUE;
    arm["MOVE_JUDGE"] >> MOVE_JUDGE;
    arm["WAIT_TIME"] >> WAIT_TIME;
    arm["SPEED"] >> SPEED;
    arm["A"] >> A;
    arm["MAX_DROP_FRAME"] >> MAX_DROP_FRAME;
    arm["DROP_PROCESS_MODE"] >> DROP_PROCESS_MODE;
};
struct Param param("../param.yml");