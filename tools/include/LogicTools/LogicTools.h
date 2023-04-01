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
    int64 last_capture_time;

public:
    bool waitForNum(int num);
    bool timeTrigger(int64 &, float fps);

    template <typename T>
    int maxMapItem(map<int, T> map, int threshold);
    
    int maxDequeItem(deque<int> &, int threshold);
};
using logic_ptr = shared_ptr<LogicTools>;