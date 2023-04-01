/**
 * @file sleep.h
 * @author 赖建宇 (lai.jianyu@foxmail.com)
 * @brief
 * @version 1.0
 * @date 2023-03-24
 *
 * @copyright Copyright SCUT RobotLab(c) 2021
 *
 */
#include "sleep.h"

using namespace std;

void Sleep::loadModel(const string &path)
{
    string model = path;
    resnet_101 = cv::dnn::readNetFromONNX(model);
    // resnet_101.setPreferableBackend(cv::dnn::DNN_BACKEND_OPENCV);
    resnet_101.setPreferableTarget(cv::dnn::DNN_TARGET_CPU);
}

int Sleep::sleepPoseJudge(cv::Mat &depth)
{
    cv::Mat depth_resized;
    cv::resize(depth, depth_resized, cv::Size(224, 224));
    cv::Mat depth_blob;
    preProcess(depth_resized, depth_blob);
    string path;
    if (depth_blob.type() == 0 || depth_blob.type() == 5)
    {
        path = "../onnx/resnet50_mergecam_gray_side.onnx";
    }
    else
    {
        path = "../onnx/resnet50_mergecam.onnx";
    }
    loadModel(path);
    cv::Mat input = cv::dnn::blobFromImage(depth_blob, 1.0, cv::Size(224, 224), cv::Scalar(0, 0, 0), false, true, CV_32F);
    resnet_101.setInput(input);
    cv::Mat prob = resnet_101.forward();
    cout << prob << endl;

    cv::Point classIdPoint;
    double confidence;
    // 查找最大值和最小值
    minMaxLoc(prob.reshape(1, 1), 0, &confidence, 0, &classIdPoint);
    int classId = classIdPoint.x;
    cout << "classID:" << classId << endl;

    return classId;
}

// 图像处理  标准化处理
void Sleep::preProcess(const cv::Mat &image, cv::Mat &image_blob)
{
    cv::Mat input;
    image.copyTo(input);
    int type = input.type();
    input.convertTo(input, CV_32F); // 一定要转为CV_32F使用！

    // 数据处理 标准化
    std::vector<cv::Mat> channels, channel_p;
    split(input, channels);
    cv::Mat R, G, B;

    // 区分单通道和多通道
    if (type == 0)
    {
        B = channels.at(0);
        channel_p.push_back(B);
        B = (B / 255. - 0.5) / 0.5;
        image_blob = B;
        image_blob.convertTo(image_blob, CV_32FC1);
    }
    else
    {
        B = channels.at(0);
        G = channels.at(1);
        R = channels.at(2);

        B = (B / 255. - 0.5) / 0.5;
        G = (G / 255. - 0.5) / 0.5;
        R = (R / 255. - 0.5) / 0.5;

        channel_p.push_back(R);
        channel_p.push_back(G);
        channel_p.push_back(B);
        cv::Mat outt;
        merge(channel_p, outt);
        image_blob = outt;
    }
    // cout << image_blob.type() << endl;
}
