/**
 * @file detector.cpp
 * @author 赖建宇 (lai.jianyu@foxmail.com)
 * @brief
 * @version 1.0
 * @date 2023-04-18
 *
 * @copyright Copyright SCUT RobotLab(c) 2021
 *
 */

#include "detector.h"

using namespace std;

/**
 * @brief 获取单一点的距离
 *
 * @param depth_frame
 * @param point
 * @return int
 */
int Depth::getPointDepth(const cv::Mat &depth_frame, const cv::Point2i &point)
{
    // cv::Point2i valid_point = point;
    // return depth_frame.at<uint16_t>(point.y, point.x); // 这里是反过来的...
    int min_dis = 65535;
    int scale = 2;
    bool is_valid_point = 0;
    while (!is_valid_point)
    {
        if (scale >= 16)
        {
            // length >= 31,依然找不到有明确距离的点(黑洞)
            cout << "POINT_DIS_INVALID!" << point << endl;
            return 25000;
        }
        int length = 2 * scale - 1; // length = 3/7/11/15/...
        int half_lenght = (length - 1) / 2;
        cv::Rect2i center_rect(point.x - half_lenght, point.y - half_lenght, length, length);
        for (int y = center_rect.y; y < center_rect.y + center_rect.height; y++)
        {
            // cout << "y" << y << endl;
            const uint16_t *data_y = depth_frame.ptr<uint16_t>(y);
            for (int x = center_rect.x; x < center_rect.x + center_rect.width; x++)
            {
                // cout << "x" << x << endl;
                if (x > 0 && y > 0)
                {
                    // cout << "dis" << data_y[x] << endl;
                    // int dis = depth_frame.at<uint16_t>(y, x);
                    int dis = data_y[x];
                    if (dis < min_dis)
                    {
                        min_dis = dis;
                    }
                }
            }
        }
        scale += 2;
        if (min_dis != 0 && min_dis < 60000)
        {
            is_valid_point = 1;
            break;
        }
    }
    // cout << "valid-length - " << scale << endl;
    return min_dis;
}

/**
 * @brief 获取目标框的距离
 *
 * @param depth_frame
 * @param rect
 * @return int
 */
int Depth::getTargetDepth(const cv::Mat &depth_frame, const cv::Rect2i &rect, const int type)
{
    int depth = 25000; // 默认对焦在25m，基本为无穷远
    int height = rect.height;
    int width = rect.width;
    int location = 0;
    float wh_ratio = (float)width / (float)height;
    // 补丁
    int min_lenght = MIN(height, width);
    if (min_lenght <= 20)
    {
        cv::Point2i center(rect.x + width / 2, rect.y + width / 2);
        return getPointDepth(depth_frame, center);
    }
    if (type == 0)
    {
        // 基于人体框的长宽比，盲猜头部的位置:
        float k = 0.5;
        height = k * wh_ratio * height;
    }
    // cv::Point2i center;
    // center.x = rect.x + (width / 2);
    // center.y = rect.y + (height / 2);
    cout << "target-height " << height << endl;
    cout << "target-width " << width << endl;
    int point_dis = 0;
    // 建立深度队列
    deque<int> depth_deque;
    int invalid_point_num = 0;
    int valid_point_num = 0;
    // 自动设置步长，确保采样点大于10*10
    int stride = MIN(height, width) / 10;
    // cout << "stride " << stride << endl;
    // 模仿相机对焦点的操作
    // 每 stride*stride 像素进行一次采样，降低计算压力（伪池化?）
    for (int i = rect.x; i < rect.x + width; i += stride)
    {
        for (int j = rect.y; j < rect.y + height; j += stride)
        {
            // cout << "i-j:" << i << j << endl;
            cv::Point2i target(i, j);
            point_dis = getPointDepth(depth_frame, target);
            // cout << "point-dis:" << point_dis << endl;
            if (point_dis != 0 && point_dis < 25000)
            {
                // invalid的点和无穷远点不计入
                depth_deque.push_back(point_dis);
                valid_point_num++;
            }
            else
            {
                invalid_point_num++;
            }
        }
    }
    cout << "valid-point-rate" << (float)valid_point_num / (float)(invalid_point_num + valid_point_num) << endl;
    cout << "deque-size " << depth_deque.size() << endl;
    // 升序排列
    if (!depth_deque.empty())
    {
        sort(depth_deque.begin(), depth_deque.end());
    }
    else
    {
        // cout << "invalid-target : dis error" << endl;
        // depth_deque.push_back(25000);
        return 25000;
    }
    // 对depth_vector进行统计
    // 也许可以试试k-means？
    // 暂时取中值，有待改进
    location = depth_deque.size() / 2;
    if (type == 0)
    {
        // 0定义为人体识别框
        // 此时背景面积较大，倾向于选择较近的点
        location = depth_deque.size() / 7;
    }
    if (type == -1)
    {
        // 1定义为面部侧对
        // 此时背景面积较大，倾向于选择较近的点
        location = depth_deque.size() / 4;
    }
    if (type == -2)
    {
        // 1定义为面部正对
        // 此时背景面积较小，取中值
        location = depth_deque.size() / 2;
    }
    return depth_deque.at(location);
    // int temp_depth = getPointDepth(depth_frame, center);
    // return temp_depth;
}

/**
 * @brief 距离刷新函数
 *
 * @param depth_frame
 * @param object
 */
void Depth::updateDis(cv::Mat &depth_frame, SingleObject &object)
{
    if (object.single_face_in_object.empty())
    {
        int new_dis = this->getTargetDepth(depth_frame, object.single_object_box, 0);
        if (abs(new_dis - object.cam_dis) < 400)
        {
            object.cam_dis = new_dis;
        }
    }
    else
    {
        cv::Point2i eye1(object.face_landmarks[0], object.face_landmarks[1]);
        cv::Point2i eye2(object.face_landmarks[2], object.face_landmarks[3]);
        int new_dis_1 = this->getTargetDepth(depth_frame, object.single_object_box, 0);
        int eye_dis_1 = this->getPointDepth(depth_frame, eye1);
        int eye_dis_2 = this->getPointDepth(depth_frame, eye2);
        int face_dis;
        int eye_dis = sqrt(pow(eye1.x - eye2.x, 2) + pow(eye1.y - eye2.y, 2));
        if (eye_dis < object.single_object_box.width / 4)
        {
            face_dis = this->getTargetDepth(depth_frame, object.single_face_in_object, -2);
        }
        else
        {
            face_dis = this->getTargetDepth(depth_frame, object.single_face_in_object, -1);
        }
        // 判断可靠性(对快速运动目标可能失效)
        if (abs(new_dis_1 - object.cam_dis) < 400.f)
        {
            // string target_position = "delta-1:" + cv::format("%d", abs(new_dis_1 - object.cam_dis));
            // cv::putText(depth_frame, target_position, cv::Point2i(15, 60), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(255, 255, 255), 2);
            object.cam_dis = new_dis_1;
        }
        if (abs(eye_dis_1 - object.face_landmarks[4]) < 300.f)
        {
            // string target_position = "delta-2:" + cv::format("%d", abs(eye_dis_1 - object.face_landmarks[4]));
            // cv::putText(depth_frame, target_position, cv::Point2i(15, 90), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(255, 255, 255), 2);
            object.face_landmarks[4] = eye_dis_1;
        }
        if (abs(eye_dis_2 - object.face_landmarks[4]) < 300.f)
        {
            // string target_position = "delta-3:" + cv::format("%d", abs(eye_dis_2 - object.face_landmarks[4]));
            // cv::putText(depth_frame, target_position, cv::Point2i(15, 120), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(255, 255, 255), 2);
            object.face_landmarks[4] = eye_dis_2;
        }
        if (abs(face_dis - object.face_dis) < 400.f)
        {
            // string target_position = "delta-4:" + cv::format("%d", abs(face_dis - object.face_dis));
            // cv::putText(depth_frame, target_position, cv::Point2i(15, 150), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(255, 255, 255), 2);
            object.face_dis = face_dis;
        }
    }
}