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
    cv::Point2i valid_point = point;
    int min_dis = 65535;
    int scale = 1;
    while (min_dis == 65535 && min_dis != 0)
    {
        int length = 2 * scale;
        cv::Rect2i center_rect(point.x - length / 2, point.y - length / 2, length, length);
        for (int x = center_rect.x; x < center_rect.x + center_rect.width; x++)
        {
            for (int y = center_rect.y; y < center_rect.y + center_rect.height; y++)
            {
                int dis = depth_frame.at<uint16_t>(x, y);
                if (dis < min_dis)
                {
                    min_dis = dis;
                }
            }
        }
        scale *= 2;
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
int Depth::getTargetDepth(const cv::Mat &depth_frame, const cv::Rect2i &rect)
{
    int depth = 25000; // 默认对焦在25m，基本为无穷远
    int height = rect.height;
    int width = rect.width;
    cv::Point2i center;
    center.x = rect.x + width / 2;
    center.y = rect.y + height / 2;
    // cout << "target-height " << height << endl;
    // cout << "target-width " << width << endl;
    int point_dis = 0;
    // 建立深度队列
    deque<int> depth_deque;
    // 自动设置步长，确保采样点大于10*10
    int stride = MIN(height, width) / 10;
    // cout << "stride " << stride << endl;
    // 模仿相机对焦点的操作
    // 每 stride*stride 像素进行一次采样，降低计算压力（伪池化）
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
                depth_deque.push_back(point_dis);
            }
        }
    }
    // cout << "deque-size " << depth_deque.size() << endl;
    // 升序排列
    if (!depth_deque.empty())
    {
        sort(depth_deque.begin(), depth_deque.end());
    }
    else
    {
        // cout << "invalid-target : dis error" << endl;
        depth_deque.push_back(25000);
    }
    // 对depth_vector进行统计
    // 也许可以试试k-means？
    // 暂时取中值，有待改进
    int medium = depth_deque.size() / 2;
    // return depth_deque.at(medium);
    int temp_depth = getPointDepth(depth_frame, center);
    return temp_depth;
}