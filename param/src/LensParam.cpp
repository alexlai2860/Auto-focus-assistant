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

#include "LensParam.h"
#include <opencv2/core/persistence.hpp>

using namespace std;

LensParam::LensParam(const string &param)
{
    param_path = param;
    cv::FileStorage lens(param, cv::FileStorage::READ);
    assert(lens.isOpened());

    lens["INIT_DIS"] >> INIT_DIS;
    lens["A"] >> A;
    lens["B"] >> B;
    lens["C"] >> C;
    lens["D"] >> D;
    lens["E"] >> E;
    lens["F"] >> F;
    lens["G"] >> G;

    lens["LENS_1"] >> LENS_1;
    lens["LENS_2"] >> LENS_2;
    lens["LENS_3"] >> LENS_3;
    lens["LENS_4"] >> LENS_4;
    lens["LENS_5"] >> LENS_5;
};

/**
 * @brief 将数据写入yml文件
 */
void LensParam::write()
{
    cv::FileStorage lens(param_path, cv::FileStorage::WRITE);
    assert(lens.isOpened());
    lens.write("INIT_DIS", INIT_DIS);
    lens.write("A", A);
    lens.write("B", B);
    lens.write("C", C);
    lens.write("D", D);
    lens.write("E", E);
    lens.write("F", F);
    lens.write("G", G);

    lens.write("LENS_1", LENS_1);
    lens.write("LENS_2", LENS_2);
    lens.write("LENS_3", LENS_3);
    lens.write("LENS_4", LENS_4);
    lens.write("LENS_5", LENS_5);
}

struct LensParam lens_param("../param/lens_param.yml");