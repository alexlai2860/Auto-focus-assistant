/**
 * @file main.cpp
 * @author 赖建宇 (lai.jianyu@foxmail.com)
 * @brief
 * @version 1.0
 * @date 2022-11-29
 *
 * @copyright Copyright SCUT RobotLab(c) 2021
 *
 */

#include "controller.h"
#include <iostream>
#include <string>

using namespace std;

int main()
{
    int64 t0 = cv::getTickCount();
    Controller controller;
    controller.control(t0);

    return 0;
}