/**
 * @file dis_processor.cpp
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

using namespace std;

int Dis::disCalculate(int mode, cv::Mat &d16, deque<cv::Point2f> &points)
{
    // select the point
    uint16_t dis = d16.at<uint16_t>(int(points.back().y), int(points.back().x));
    // uint16_t dis = d16.at<uint16_t>(393,232);
    // cout << "selected_point: " << points.back() << endl;
    // output and store the distance
    int target_dis = int(dis) * 8000 / 65535;
    // ERROR process
    if (target_dis != 0)
    {
        this->target_dis.push_back(target_dis);
        // cout << "dis : " << target_dis << endl;
        if (this->target_dis.size() >= param.DIS_DEQUE)
        {
            this->target_dis.pop_front();
        }
        return target_dis;
    }
    else if (!this->target_dis.empty())
    {
        this->target_dis.push_back(this->target_dis.back());
        if (this->target_dis.size() >= param.DIS_DEQUE)
        {
            this->target_dis.pop_front();
        }
        // cout << "dis : " << this->target_dis.back() << " (currection) " << endl;
        return this->target_dis.back();
    }
    else
    {
        // cout << "OUT OF DETECT LIMIT" << endl;
        return -1;
    }
}
/**
 * @brief
 *
 * @param t0 用于计算时间戳
 * @param points （可选）面部关键点，用于辅助判断是否移动
 * @return true
 * @return false
 */
bool Dis::movDecider(int64 &t0, deque<cv::Point2f> &points)
{
    if (!this->target_dis.empty())
    {
        int min_dis = 8000, max_dis = 0;
        // find the max and min distance in the last few seconds
        for (int i = 0; i < this->target_dis.size(); i++)
        {
            if (target_dis.at(i) < min_dis)
            {
                min_dis = target_dis.at(i);
            }
            if (target_dis.at(i) > max_dis)
            {
                max_dis = target_dis.at(i);
            }
        }
        if (max_dis - min_dis >= param.MOVE_JUDGE)
        {
            cout << "move" << endl;
            return 1;
        }
        else
        {
            cout << "static" << endl;
            return 0;
        }
    }
    else
    {
        return 0;
    }
}