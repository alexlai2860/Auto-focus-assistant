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
    arm["KALMAN_PROCESS"] >> KALMAN_PROCESS;
    arm["KALMAN_MEASURE"] >> KALMAN_MEASURE;

    arm["MAX_DROP_FRAME"] >> MAX_DROP_FRAME;
    arm["DROP_PROCESS_MODE"] >> DROP_PROCESS_MODE;

    arm["cam_module"] >> cam_module;
    arm["INVERT_ON"] >> INVERT_ON;
    arm["ROI_ON"] >> ROI_ON;
    arm["ASTRA_width"] >> ASTRA_width;
    arm["ASTRA_height"] >> ASTRA_height;
    arm["RS_width"] >> RS_width;
    arm["RS_height"] >> RS_height;
    arm["RS_fps"] >> RS_fps;
    arm["width_compensate"] >> width_compensate;
    arm["height_compensate"] >> height_compensate;

    arm["DRAW_BOX_MIN"] >> DRAW_BOX_MIN;
    arm["DIS_RATIO_MAX"] >> DIS_RATIO_MAX;
    arm["AREA_RATIO_COEF"] >> AREA_RATIO_COEF;
    arm["POINT_DIS_COEF"] >> POINT_DIS_COEF;
    arm["LENS_LENGTH"] >> LENS_LENGTH;
};
struct Param param("../param/param.yml"); // from build folder