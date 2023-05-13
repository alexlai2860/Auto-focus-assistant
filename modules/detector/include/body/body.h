/**
 * @file BodyProcessor.h
 * @author 赖建宇 (lai.jianyu@foxmail.com)
 * @brief 
 * @version 1.0
 * @date 2023-05-10
 * 
 * @copyright Copyright 2023 lai.jianyu
 * 
 */
#pragma once

#include "detector.h"
#define _CRT_SECURE_NO_WARNINGS
#include <iostream>
#include <fstream>
#include <string>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <onnxruntime_cxx_api.h>

using namespace std;

class E2Pose
{
public:
	bool init(string model_path, float confThreshold);
	bool detect(cv::Mat& cv_image);
private:
	float confThreshold;

	void normalize_(cv::Mat img);
	int inpWidth;
	int inpHeight;
	vector<float> input_image_;

	Ort::Env env = Ort::Env(ORT_LOGGING_LEVEL_ERROR, "E2Pose");
	Ort::Session *ort_session = nullptr;
	Ort::SessionOptions sessionOptions = Ort::SessionOptions();
	vector<char*> input_names;
	vector<char*> output_names;
	vector<vector<int64_t>> input_node_dims; // >=1 outputs
	vector<vector<int64_t>> output_node_dims; // >=1 outputs
};

class Body : public Detector
{
public:
    E2Pose e2pose;
    bool init = 0;
    bool virtual detect(cv::Mat &, cv::Mat &) override;
    bool virtual drawBox(cv::Mat &, cv::Mat &) override;
};