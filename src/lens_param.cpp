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
    param_path = param;
    cv::FileStorage lens(param, cv::FileStorage::READ);
    assert(lens.isOpened());

    lens["INIT_PULSE_1"] >> INIT_PULSE_1;
    lens["INFINIT_PULSE_1"] >> INFINIT_PULSE_1;
    lens["A_1"] >> A_1;
    lens["B_1"] >> B_1;
    lens["C_1"] >> C_1;
    lens["D_1"] >> D_1;
};

/**
 * @brief 将数据写入yml文件
 */
void LensParam::write()
{
    cv::FileStorage lens(param_path, cv::FileStorage::WRITE);
    assert(lens.isOpened());
    lens.write("INIT_PULSE_1", INIT_PULSE_1);
    lens.write("INFINIT_PULSE_1", INFINIT_PULSE_1);
    lens.write("A_1", A_1);
    lens.write("B_1", B_1);
    lens.write("C_1", C_1);
    lens.write("D_1", D_1);
}

struct LensParam lens_param("../lens_param.yml");