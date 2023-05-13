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

#define _CRT_SECURE_NO_WARNINGS
#include <opencv2/dnn.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
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

    // Ort_test
    bool OrtInit(string model_path, float confThreshold);
    void normalize_(cv::Mat img);
    // int inpWidth;
    // int inpHeight;
    vector<float> input_image_;
    Ort::Env env = Ort::Env(ORT_LOGGING_LEVEL_ERROR, "SCRFD");
    Ort::Session *ort_session = nullptr;
    Ort::SessionOptions sessionOptions = Ort::SessionOptions();
    vector<char *> input_names;
    vector<char *> output_names;
    vector<vector<int64_t>> input_node_dims;  // >=1 outputs
    vector<vector<int64_t>> output_node_dims; // >=1 outputs

private:
    const float stride[3] = {8.0, 16.0, 32.0};
    const int inpWidth = 320;
    const int inpHeight = 320;
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
    Net_config default_cfg = {0.4, 0.5, "../onnx/scrfd_500m_kps_320.onnx"};
    SCRFD scrfd;
    bool init = 0;
    bool virtual detect(cv::Mat &, cv::Mat &) override;
    bool virtual drawBox(cv::Mat &, cv::Mat &) override;
};
