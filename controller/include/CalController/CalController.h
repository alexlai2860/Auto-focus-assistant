/**
 * @file cal_controller.h
 * @author 赖建宇 (lai.jianyu@foxmail.com)
 * @brief 
 * @version 1.0
 * @date 2023-03-10
 * 
 * @copyright Copyright SCUT RobotLab(c) 2021
 * 
 */

// #include "processor.h"
// #include "calibrator.h"

// class CalController
// {
// private:
//     motor_ptr __motor = make_shared<SteppingMotor>();
//     cal_ptr __calibrator = make_shared<Calibrator>();
//     detect_ptr __detector = make_shared<Face>();


// public:
//     void CalControl(int64 &);
// };
// using calcontrol_ptr = shared_ptr<CalController>;

#pragma once

#include "SubController.h"

class CalController : public SubController
{
protected:
    vector<cv::Mat> function;
    vector<cv::Point2f> cal_points;

    motor_ptr __motor = make_shared<SteppingMotor>();

public:
    int CalInit(int64 &) override;
    bool FocusInit(int64 &, int &) override;
    void rs_read(rs2::pipeline &, rs2::frameset &) override;
    cv::Mat polyFit(vector<cv::Point2f> &, int, int);
    void astraCalibration(int, Dis &, int64 &, Data &);
    void rsCalibration(int, Dis &, int64 &, Data &);
    void rsCalibrationNew(int, Dis &, int64 &, Data &);
};

// using cal_ptr = shared_ptr<Calibrator>;