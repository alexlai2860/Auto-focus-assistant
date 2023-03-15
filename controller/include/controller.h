/**
 * @file controller.h
 * @author 赖建宇 (lai.jianyu@foxmail.com)
 * @brief
 * @version 1.0
 * @date 2023-03-08
 *
 * @copyright Copyright SCUT RobotLab(c) 2021
 *
 */

#include "CalController.h"
#include "FocusController.h"
// #include "calibrator.h"

class Controller
{
private:
    subcontrol_ptr __controller;

public:
    void control(int64 &);
};