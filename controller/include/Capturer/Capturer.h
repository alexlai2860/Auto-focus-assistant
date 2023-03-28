/**
 * @file Capturer.h
 * @author 赖建宇 (lai.jianyu@foxmail.com)
 * @brief
 * @version 1.0
 * @date 2023-03-23
 *
 * @copyright Copyright SCUT RobotLab(c) 2021
 *
 */

#pragma once

#include "SubController.h"
#include "decider.h"

class Capturer : public SubController
{
protected:
    // face_ptr __face;
    // decider_ptr __decider;
    int64 last_capture_time = 0;
    bool timeTrigger(int64 &, float);

public:
    int init(int64 &, int) override;
    void capture();
};