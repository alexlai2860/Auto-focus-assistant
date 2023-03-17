/**
 * @file LogicTools.h
 * @author 赖建宇 (lai.jianyu@foxmail.com)
 * @brief
 * @version 1.0
 * @date 2023-03-17
 *
 * @copyright Copyright SCUT RobotLab(c) 2021
 *
 */
#pragma once
#include "param.h"
#include "LensParam.h"
#include <iostream>

class LogicTools
{
public:
    bool waitForNum(int num);
};
using logic_ptr = shared_ptr<LogicTools>;