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
    // 起点（最近对焦位置）和终点（最远对焦位置）坐标
    int INIT_PULSE_1;
    int INFINIT_PULSE_1;
    // 拟合函数: Y（脉冲差） = AX^3 + BX^2 + CX + D
    double A_1;
    double B_1;
    double C_1;
    double D_1;

    string param_path;

    LensParam(const std::string &param);
    void write();
};
extern LensParam lens_param;