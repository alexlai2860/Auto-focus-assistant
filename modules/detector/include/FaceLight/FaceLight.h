/**
 * @file FaceLight.h
 * @author 赖建宇 (lai.jianyu@foxmail.com)
 * @brief
 * @version 1.0
 * @date 2023-05-06
 *
 * @copyright Copyright 2023 lai.jianyu
 *
 */
#pragma once

#include <opencv2/dnn.hpp>
#include <opencv2/opencv.hpp>
#include <onnxruntime_cxx_api.h>
#include <fstream>
#include <sstream>
#include "detector.h"

struct Net_config
{
    float confThreshold; // class Confidence threshold
    float nmsThreshold;  // Non-maximum suppression threshold
    string modelfile;
};

class SCRFD
{
public:
    bool init(Net_config config);
    bool detect(cv::Mat &frame, cv::Mat &depth_frame);
    vector<cv::Rect> boxes;
    deque<vector<SingleFace>> scrfd_face;
    Depth depth;

private:
    const float stride[3] = {8.0, 16.0, 32.0};
    const int inpWidth = 640;
    const int inpHeight = 640;
    float confThreshold;
    float nmsThreshold;
    const bool keep_ratio = true;
    cv::dnn::Net net;
    bool isValidFace(SingleFace single_face);
    cv::Mat resize_image(cv::Mat srcimg, int *newh, int *neww, int *top, int *left);
};

class FaceLight : public Detector
{
public:
    Net_config default_cfg = {0.4, 0.5, "../onnx/scrfd_500m_kps.onnx"};
    SCRFD scrfd;
    bool init = 0;
    bool virtual detect(cv::Mat &, cv::Mat &) override;
    bool virtual drawBox(cv::Mat &, cv::Mat &) override;
};
