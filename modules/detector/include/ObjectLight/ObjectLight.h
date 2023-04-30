/**
 * @file ObjectLight.h
 * @author 赖建宇 (lai.jianyu@foxmail.com)
 * @brief
 * @version 1.0
 * @date 2023-04-08
 *
 * @copyright Copyright SCUT RobotLab(c) 2021
 *
 */
#pragma once

#include <opencv2/dnn.hpp>
#include <opencv2/opencv.hpp>
#include <fstream>
#include <sstream>
#include "detector.h"

class yolo_fast
{
public:
    vector<int> classIds;
    vector<float> confidences;
    vector<cv::Rect> boxes;
    deque<vector<SingleObject>> yolo_target;
    Depth depth;

    bool init(string modelpath, float objThreshold, float confThreshold, float nmsThreshold);
    bool detect(cv::Mat &srcimg, const cv::Mat &depth_frame);
    bool insideROI(cv::Point2f &);
    void simpleNMS(vector<SingleObject> &);
    void drawPred(cv::Mat &frame, const cv::Mat &depth_frame);
    void drawSinglePred(int classId, float conf, int left, int top, int right, int bottom, cv::Mat &frame, const cv::Mat &depth_frame);

private:
    const float anchors[2][6] = {{12.64, 19.39, 37.88, 51.48, 55.71, 138.31}, {126.91, 78.23, 131.57, 214.55, 279.92, 258.87}};
    const float stride[3] = {16.0, 32.0};
    const int inpWidth = 352;
    const int inpHeight = 352;
    const int num_stage = 2;
    const int anchor_num = 3;
    float objThreshold;
    float confThreshold;
    float nmsThreshold;
    std::vector<string> classes;
    const string classesFile = "../param/coco.names";
    int num_class;
    cv::dnn::Net net;
};

class ObjectLight : public Detector
{
public:
    yolo_fast yolo;
    bool init = 0;
    bool virtual detect(cv::Mat &, cv::Mat &) override;
    bool virtual drawBox(cv::Mat &, cv::Mat &) override;
};