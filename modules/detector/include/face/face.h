/**
 * @file face.h
 * @author 赖建宇 (lai.jianyu@foxmail.com)
 * @brief
 * @version 1.0
 * @date 2023-03-11
 *
 * @copyright Copyright SCUT RobotLab(c) 2021
 *
 */
#pragma once

#include"detector.h"

using namespace std;

class Face : public Detector
{
public:
    Face() { cout << "face-constructor" << endl; }
    ~Face() { cout << "face-destructor" << endl; }
    // bool draw_face_box = 0;
    cv::Mat detected_faces;

    bool virtual detect(cv::Mat &, int &) override;
    bool virtual drawBox(cv::Mat &) override;
    bool isValidFace(cv::Mat &, int);
};