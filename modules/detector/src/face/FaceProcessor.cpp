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

#include "face.h"

using namespace std;

bool Face::YNinit()
{
    string onnx_path = "../onnx/yunet.onnx";
    if (param.cam_module == ASTRA)
    {
        faceDetector = cv::FaceDetectorYN::create(onnx_path, "", cv::Size(param.ASTRA_width, param.ASTRA_height), 0.96);
    }
    if (param.cam_module == REALSENSE)
    {
        faceDetector = cv::FaceDetectorYN::create(onnx_path, "", cv::Size(param.RS_width, param.RS_height), 0.9);
    }
    isYN_init = 1;
    return 1;
}

/**
 * @brief 面部识别主函数
 *
 * @param color_frame 彩色图像
 * @param faces 储存识别到的面部(后续可以删掉？)
 * @param count 掉帧(未识别到面部)数计算
 * @return true 识别到人脸
 * @return false 未识别到人脸
 */
bool Face::detect(cv::Mat &color_frame, cv::Mat &depth_frame)
{
    // cv::Mat &faces = detected_faces;
    cv::Mat faces;
    int last_deque_size = face.size();
    int last_face_num = 0;
    if (last_deque_size != 0)
    {
        last_face_num = face.back().size();
    }

    // detect
    if (isYN_init)
    {
        faceDetector->detect(color_frame, faces);
    }
    else
    {
        YNinit();
        faceDetector->detect(color_frame, faces);
    }

    // process
    if (faces.cols > 0)
    {
        cout << "cols>0:" << faces.cols << endl;
        vector<cv::Point2f> new_centers;
        cv::Mat selected_faces;
        vector<SingleFace> current_faces;
        for (int i = 0; i < faces.rows; i++)
        {
            if (isValidFace(faces, i))
            {
                cout << "is_valid_face:" << i << endl;
                draw_face_box = 1;
                // 计算方框中心
                cv::Point2f center(faces.at<float>(i, 0) + faces.at<float>(i, 2) / 2, faces.at<float>(i, 1) + faces.at<float>(i, 3) / 2);
                cv::Mat selected_face = faces.rowRange(i, i + 1);
                new_centers.push_back(center);
                selected_faces.push_back(selected_face);
                // 新建single_face并载入current_faces
                SingleFace single_face;
                single_face.center = center;
                single_face.single_face = selected_face;
                single_face.cam_dis = depth.getPointDepth(depth_frame, center);
                single_face.detected = 1;
                current_faces.push_back(single_face);
            }
        }
        if (!current_faces.empty())
        {
            face.push_back(current_faces);
        }
        cout << "current-face-size " << current_faces.size() << endl;
        cout << "face-size " << face.size() << endl;
        // faces_deque.push_back(selected_faces);
        // cout << faces_deque.size() << endl;
        // cout << faces_deque.back() << endl;
        if (face.size() >= param.FACE_DEQUE)
        {
            face.pop_front();
        }
        if (!current_faces.empty())
        {
            return 1;
        }
        else
        {
            return 0;
        }
    }
    else
    {
        cout << "faces-cols:" << faces.cols << endl;
        // // Frame drop process
        // if (face.empty())
        // {
        //     // 只有在人脸队列为空时，才不绘制人脸框
        //     draw_face_box = 0;
        //     return 0;
        // }
        // vector<SingleFace> last_face = face.back();
        // this->face.push_back(last_face);
        return 0;
    }
}


bool Face::drawBox(cv::Mat &color_frame, cv::Mat &depth_frame)
{
    // cv::Mat faces = detected_faces;
    // cv::Mat faces = faces_deque.back();
    const vector<SingleFace> face_vector = face.back();
    cv::Mat faces;
    for (int i = 0; i < face_vector.size(); i++)
    {
        faces.push_back(face_vector.at(i).single_face);
        cout << "draw-faces-vector-" << i << ": " << face_vector.at(i).single_face << endl;
    }
    for (int i = 0; i < faces.rows; i++)
    {
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
        cout << "draw-face-" << i << endl;
    }
    return 1;
}

/**
 * @brief 面部识别结果筛选函数（异常结果/超出roi）
 *
 * @param faces 储存的面部数据
 * @param i 第i个面部
 * @return true valid_face
 * @return false invalid_face
 */
bool Face::isValidFace(cv::Mat &faces, int i)
{
    // 筛除面积过大/面积过小/处于ROI区域外的faces
    int frame_area = param.RS_height * param.RS_width;
    cv::Point2f center(faces.at<float>(i, 0) + faces.at<float>(i, 2) / 2, faces.at<float>(i, 1) + faces.at<float>(i, 3) / 2);
    cv::Point2i ROI(param.RS_width / 2, param.RS_height / 2);
    cv::Rect2i face_box(int(faces.at<float>(i, 0)), int(faces.at<float>(i, 1)),
                        int(faces.at<float>(i, 2)), int(faces.at<float>(i, 3)));
    int border_x = (param.RS_width - ROI.x) / 2;
    int border_y = (param.RS_height - ROI.y) / 2;
    if (int(faces.at<float>(i, 2)) * int(faces.at<float>(i, 3)) > 0.25 * frame_area)
    {
        return 0;
    }
    else if (center.x < param.RS_width - border_x - ROI.x || center.x > param.RS_width - border_x)
    {
        return 0;
    }
    else if (center.y < param.RS_height - border_y - ROI.y || center.y > param.RS_height - border_y)
    {
        return 0;
    }
    else if (face_box.area() < param.DRAW_BOX_MIN * (param.DRAW_BOX_MIN + 10))
    {
        return 0;
    }
    return 1;
}