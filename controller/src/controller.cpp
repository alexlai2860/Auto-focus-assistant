/**
 * @file controller.cpp
 * @author 赖建宇 (lai.jianyu@foxmail.com)
 * @brief
 * @version 1.0
 * @date 2023-03-08
 *
 * @copyright Copyright SCUT RobotLab(c) 2021
 *
 */

#include "controller.h"
#include <iostream>

using namespace std;

void Controller::control(int64 &t0)
{
    // 初始化
    // __controller = make_shared<Capturer>();
    // __controller->init(t0, 0);
    __controller = make_shared<CalController>();
    int lens_num = __controller->init(t0, 0);
    __controller = make_shared<FocusController>();
    bool init_status = __controller->init(t0, lens_num);
}