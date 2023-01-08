/**
 * @file face_processor.cpp
 * @author 赖建宇 (lai.jianyu@foxmail.com)
 * @brief
 * @version 1.0
 * @date 2022-12-01
 *
 * @copyright Copyright SCUT RobotLab(c) 2021
 *
 */

#include "processor.h"
#include "param.h"
#include <iostream>

using namespace std;

bool Face::faceDetect(cv::Mat &color_frame, cv::Mat &faces, int &count)
{
    // detect faces from the colorframe
    string onnx_path = "../onnx/yunet.onnx";
    cv::Ptr<cv::FaceDetectorYN> faceDetector;
    if (param.cam_module == ASTRA)
    {
        faceDetector = cv::FaceDetectorYN::create(onnx_path, "", cv::Size(param.ASTRA_width, param.ASTRA_height));
    }
    if (param.cam_module == REALSENSE)
    {
        faceDetector = cv::FaceDetectorYN::create(onnx_path, "", cv::Size(param.RS_width, param.RS_height));
    }
    faceDetector->detect(color_frame, faces);
    if (faces.cols > 0)
    {
        count = 0;
        for (int i = 0; i < faces.rows; i++)
        {
            // cout << "faces" << faces << endl;
            // 画人脸框
            int thickness = 2;
            cv::rectangle(color_frame, cv::Rect2i(int(faces.at<float>(i, 0)), int(faces.at<float>(i, 1)), int(faces.at<float>(i, 2)), int(faces.at<float>(i, 3))), cv::Scalar(0, 255, 0), thickness);

            // 画方框中心
            cv::Point2f center(faces.at<float>(i, 0) + faces.at<float>(i, 2) / 2, faces.at<float>(i, 1) + faces.at<float>(i, 3) / 2);
            cv::circle(color_frame, center, 2, cv::Scalar(0, 200, 200), 5);

            // 画关键点
            cv::Point2i left_eye(int(faces.at<float>(i, 4)), int(faces.at<float>(i, 5)));      // blue
            cv::Point2i right_eye(int(faces.at<float>(i, 6)), int(faces.at<float>(i, 7)));     // red
            cv::Point2i nose(int(faces.at<float>(i, 8)), int(faces.at<float>(i, 9)));          // green
            cv::Point2i left_mouth(int(faces.at<float>(i, 10)), int(faces.at<float>(i, 11)));  // pink
            cv::Point2i right_mouth(int(faces.at<float>(i, 12)), int(faces.at<float>(i, 13))); // yellow

            cv::circle(color_frame, left_eye, 2, cv::Scalar(255, 0, 0), thickness);      // blue
            cv::circle(color_frame, right_eye, 2, cv::Scalar(0, 0, 255), thickness);     // red
            cv::circle(color_frame, nose, 2, cv::Scalar(0, 255, 0), thickness);          // green
            cv::circle(color_frame, left_mouth, 2, cv::Scalar(255, 0, 255), thickness);  // purple
            cv::circle(color_frame, right_mouth, 2, cv::Scalar(0, 255, 255), thickness); // yellow

            // 判断人脸朝向
            if (left_eye.x < center.x && right_eye.x > center.x)
            {
                if (left_mouth.x < center.x && right_eye.x > center.x)
                {
                    cv::putText(color_frame, "-center-", cv::Point2i(int(faces.at<float>(i, 0)), int(faces.at<float>(i, 1)) - 10), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 255));
                }
            }
            if (left_eye.x < center.x && right_eye.x < center.x)
            {
                if (left_mouth.x < center.x && right_eye.x < center.x)
                {
                    cv::putText(color_frame, "-left-", cv::Point2i(int(faces.at<float>(i, 0)), int(faces.at<float>(i, 1)) - 10), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 255));
                }
            }
            if (left_eye.x > center.x && right_eye.x > center.x)
            {
                if (left_mouth.x > center.x && right_eye.x > center.x)
                {
                    cv::putText(color_frame, "-right-", cv::Point2i(int(faces.at<float>(i, 0)), int(faces.at<float>(i, 1)) - 10), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 255));
                }
            }

            // 画置信度
            cv::putText(color_frame, cv::format("%.4f", faces.at<float>(i, 14)), cv::Point2i(int(faces.at<float>(i, 0)), int(faces.at<float>(i, 1)) + 15), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 0));

            // 储存face_center
            this->face_center.push_back(center);
            // cout << "center: " << center << endl;
        }
        if (this->face_center.size() >= param.FACE_DEQUE)
        {
            this->face_center.pop_front();
        }
        return 1;
    }
    else
    {
        // Frame drop process
        count++;
        if (face_center.empty())
        {
            return 0;
        }
        cv::Point2f last_center = face_center.back();
        this->face_center.push_back(last_center);
        // if (count > 15)
        // {
        //     this->face_center.clear();
        // }
        return 0;
    }
}