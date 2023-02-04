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
#include <opencv2/core.hpp>

using namespace std;

struct LensParam
{
    // 最近对焦距离
    int INIT_DIS;
    // 拟合函数: Y（脉冲差） = AX^5 + BX^4 + CX^3 + DX^2 + EX +F (目前A不启用)
    double A;
    double B;
    double C;
    double D;
    double E;
    double F;
    double G;

    // 镜头数据矩阵（8*1），依次为 [A，B，C，D，E, F, G, INIT_DIS]
    cv::Mat LENS_1, LENS_2, LENS_3, LENS_4, LENS_5;

    string param_path;

    LensParam(const std::string &param);
    void write();
};
extern LensParam lens_param;