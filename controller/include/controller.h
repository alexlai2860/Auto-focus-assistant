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

#include "processor.h"
#include "calibrator.h"

class Controller
{
private:
    motor_ptr __motor;
    cal_ptr __calibrator;

public:
    void control(int64 &);
};