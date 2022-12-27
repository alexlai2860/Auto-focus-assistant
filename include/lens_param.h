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

struct LensParam
{
    int INIT_PULSE_1;
    int INFINIT_PULSE_1;

    LensParam(const std::string &param);
};
extern LensParam lens_param;