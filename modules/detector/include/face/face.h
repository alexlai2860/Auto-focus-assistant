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

#include "detector.h"

using namespace std;

class Face : public Detector
{
private:
    cv::Ptr<cv::FaceDetectorYN> faceDetector;
    bool isYN_init;
    bool YNinit();

public:
    // bool draw_face_box = 0;
    cv::Mat detected_faces;
    deque<cv::Mat> faces_deque;

    bool virtual detect(cv::Mat &) override;
    bool virtual drawBox(cv::Mat &) override;
    bool isValidFace(cv::Mat &, int);
};