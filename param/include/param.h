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
    bool KALMAN_ON;
    float KALMAN_PROCESS;
    float KALMAN_MEASURE;
    int LENS_NUM;
    int AUTO_CAL;

    int MAX_DROP_FRAME_OBJECT;
    int MAX_DROP_FRAME_FACE;
    int DROP_PROCESS_MODE;
    int FACE_INIT_LIMIT;
    int TARGET_INIT_LIMIT;

    int cam_module;
    bool INVERT_ON;
    bool ROI_ON;
    int ASTRA_width;
    int ASTRA_height;
    int RS_width;
    int RS_height;
    int RS_fps;

    float DRAW_BOX_MIN;
    float DIS_RATIO_MAX;
    float AREA_RATIO_COEF;
    float POINT_DIS_COEF;
    int LENS_LENGTH;
    int ROI_LENGTH;
    int width_compensate;
    int height_compensate;

    string param_path;

    Param(const std::string &param);
};
extern Param param;