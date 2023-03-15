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

#include "dis.h"
#include "param.h"
#include <iostream>

using namespace std;

class Face
{
public:
    Face() { cout << "face-constructor" << endl; }
    ~Face() { cout << "face-destructor" << endl; }
    int target_face_label;
    deque<cv::Point2f> face_center;
    cv::Mat detected_faces;
    bool faceDetect(cv::Mat &, cv::Mat &, int &);
    bool isValidFace(cv::Mat &, int);
};

using face_ptr = shared_ptr<Face>;