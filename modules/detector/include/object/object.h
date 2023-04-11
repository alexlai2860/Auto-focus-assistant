/**
 * @file object.h
 * @author 赖建宇 (lai.jianyu@foxmail.com)
 * @brief
 * @version 1.0
 * @date 2023-04-07
 *
 * @copyright Copyright SCUT RobotLab(c) 2021
 *
 */
#pragma once

#include "detector.h"

#define YOLO_P6 true // 是否使用P6模型

struct Output
{
    int id;           // 结果类别id
    float confidence; // 结果置信度
    cv::Rect box;     // 矩形框
};

class Yolov5
{
public:
    Yolov5()
    {
    }
    ~Yolov5() {}
    bool readModel(cv::dnn::Net &net, std::string &netPath, bool isCuda);
    bool Detect(cv::Mat &SrcImg, cv::dnn::Net &net, std::vector<Output> &output);
    void drawPred(cv::Mat &img, std::vector<Output> result, std::vector<cv::Scalar> color);

private:
    void LetterBox(const cv::Mat &image, cv::Mat &outImage,
                   cv::Vec4d &params, //[ratio_x,ratio_y,dw,dh]
                   const cv::Size &newShape = cv::Size(640, 640),
                   bool autoShape = false,
                   bool scaleFill = false,
                   bool scaleUp = true,
                   int stride = 32,
                   const cv::Scalar &color = cv::Scalar(114, 114, 114));

#if (defined YOLO_P6 && YOLO_P6 == true)
    const int _netWidth = 640;  // ONNX图片输入宽度
    const int _netHeight = 640; // ONNX图片输入高度
#else
    const int _netWidth = 640;  // ONNX图片输入宽度
    const int _netHeight = 640; // ONNX图片输入高度
#endif // YOLO_P6

    float _classThreshold = 0.25;
    float _nmsThreshold = 0.45;

    std::vector<std::string> _className = {"person", "bicycle", "car", "motorcycle", "airplane", "bus", "train", "truck", "boat", "traffic light",
                                           "fire hydrant", "stop sign", "parking meter", "bench", "bird", "cat", "dog", "horse", "sheep", "cow",
                                           "elephant", "bear", "zebra", "giraffe", "backpack", "umbrella", "handbag", "tie", "suitcase", "frisbee",
                                           "skis", "snowboard", "sports ball", "kite", "baseball bat", "baseball glove", "skateboard", "surfboard",
                                           "tennis racket", "bottle", "wine glass", "cup", "fork", "knife", "spoon", "bowl", "banana", "apple",
                                           "sandwich", "orange", "broccoli", "carrot", "hot dog", "pizza", "donut", "cake", "chair", "couch",
                                           "potted plant", "bed", "dining table", "toilet", "tv", "laptop", "mouse", "remote", "keyboard", "cell phone",
                                           "microwave", "oven", "toaster", "sink", "refrigerator", "book", "clock", "vase", "scissors", "teddy bear",
                                           "hair drier", "toothbrush"};
};

class Object : public Detector
{
public:
    Yolov5 test;
    deque<cv::Point2f> object_center;
    bool draw_object_box = 0;
    vector<Output> result;

    bool virtual detect(cv::Mat &) override;
    bool virtual drawBox(cv::Mat &) override;
};
