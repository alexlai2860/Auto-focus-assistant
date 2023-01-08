/**
 * @file param.h
 * @author 赖建宇 (lai.jianyu@foxmail.com)
 * @brief
 * @version 1.0
 * @date 2022-12-09
 *
 * @copyright Copyright SCUT RobotLab(c) 2021
 *
 */

#pragma once
#include <iostream>

using namespace std;

struct Param
{
    int FPS;
    int DIS_DEQUE;
    int FACE_DEQUE;
    int MOVE_JUDGE;
    int WAIT_TIME;
    int SPEED;
    int A;
    int MAX_DROP_FRAME;
    int DROP_PROCESS_MODE;

    int cam_module;
    bool INVERT_ON;
    bool ROI_ON;
    int ASTRA_width;
    int ASTRA_height;
    int RS_width;
    int RS_height;
    int RS_fps;

    string param_path;

    Param(const std::string &param);
};
extern Param param;