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

#include "lens_param.h"
#include <opencv2/core/persistence.hpp>

using namespace std;

LensParam::LensParam(const string &param)
{
    cv::FileStorage arm(param, cv::FileStorage::READ);
    assert(arm.isOpened());

    arm["INIT_PULSE_1"] >> INIT_PULSE_1;
    arm["INFINIT_PULSE_1"] >> INFINIT_PULSE_1;
};
struct LensParam lens_param("../lens_param.yml");