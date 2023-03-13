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

#include "dis.h"
#include "param.h"

using namespace std;

Dis::Dis()
{
    __is_filter_init = false;
    dis_filter = KalmanFilter22(0.01, 0.01);
    dis_filter.setH(cv::Matx22f::eye());
}

int Dis::disCalculate(int rs_dis, cv::Mat &d16, deque<cv::Point2f> &points)
{
    // select the point
    int current_dis;
    if (param.cam_module == ASTRA)
    {
        uint16_t dis = d16.at<uint16_t>(int(points.back().y), int(points.back().x));
        // uint16_t dis = d16.at<uint16_t>(393,232);
        // cout << "selected_point: " << points.back() << endl;
        // output and store the distance
        current_dis = int(dis) * 8000 / 65535;
    }
    if (param.cam_module == REALSENSE)
    {
        current_dis = rs_dis;
    }

    // 卡尔曼滤波需要使用t
    this->t.push_front(cv::getTickCount());
    if (t.size() > 10)
    {
        t.pop_back();
    }

    if (target_dis.size() >= param.DIS_DEQUE)
    {
        target_dis.pop_front();
    }
    // ERROR process
    if (current_dis != 0)
    {
        if (errorJudge(current_dis))
        {
            target_dis.push_back(current_dis);
            // cout << "error_dis_dize : " << error_dis.size() << endl;
            this->updateFilter();
            // cout << "newest_dis : " << target_dis.back() << endl;

            return target_dis.back();
        }
        else if (!target_dis.empty())
        {
            target_dis.push_back(target_dis.back());
            this->updateFilter();
            return target_dis.back();
        }
    }
    else if (!target_dis.empty())
    {
        target_dis.push_back(target_dis.back());
        this->updateFilter();
        // cout << "dis : " << this->target_dis.back() << " (currection) " << endl;
        return target_dis.back();
    }
    else
    {
        target_dis.push_back(1000);
        cout << "OUT OF DETECT LIMIT" << endl;
        return -1;
    }
    return 0;
}

/**
 * @brief 判断距离数据是否属于错误数据
 *
 * @return true
 * @return false
 */
bool Dis::errorJudge(int current_dis)
{
    // error队列长度大于2，认为进入新平面
    if (error_dis.size() > 2)
    {
        error_dis.clear();
        return 1;
    }
    if (!error_dis.empty())
    {
        // 比较队尾值和当前值，若偏差小于阈值（默认为0.1倍的队尾值）加入error队列(新平面)
        if (abs(current_dis - error_dis.back()) < error_dis.back() * 0.1)
        {
            error_dis.push_back(current_dis);
            if (error_dis.size() <= 2)
            {
                return 0;
            }
        }
    }
    // 基于均值
    // int avg_dis = 0;
    // if (target_dis.size() > 11)
    // {
    //     for (int i = target_dis.size() - 11; i < target_dis.size() - 1; i++)
    //     {
    //         avg_dis = avg_dis + target_dis.at(i);
    //     }
    //     avg_dis = avg_dis / 10;
    //     // cout << "avg_dis : " << avg_dis << endl;
    //     // 比较平均值和当前值，若偏差过大（默认为0.25倍的平均值）认为出现错误
    //     if (abs(avg_dis - current_dis) > avg_dis * 0.25)
    //     {
    //         // cout << abs(avg_dis - current_dis) << endl;
    //         // cout << error_dis.size() << endl;
    //         error_dis.push_back(current_dis);
    //         return 0;
    //     }
    // }
    // 基于前一帧的值
    if (target_dis.size() > 1)
    {
        // cout << "avg_dis : " << avg_dis << endl;
        // 比较平均值和当前值，若偏差过大（默认为0.25倍的平均值）认为出现错误
        if (abs(target_dis.at(target_dis.size() - 1) - current_dis) > target_dis.at(target_dis.size() - 1) * 0.25)
        {
            // cout << abs(avg_dis - current_dis) << endl;
            // cout << error_dis.size() << endl;
            error_dis.push_back(current_dis);
            return 0;
        }
    }
    return 1;
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

/**
 * @brief 更新距离滤波器
 */
void Dis::updateFilter()
{
    int &distance = target_dis.back();
    static int last_distance = 0;
    if (__is_filter_init)
    {
        float time = ((t[0] - t[1]) / cv::getTickFrequency());
        // 设置状态转移矩阵
        dis_filter.setA(cv::Matx22f{1, time,
                                    0, 1});
        int delta_distance = distance - last_distance;
        // 预测
        dis_filter.predict();
        // 更新
        cv::Matx21f correct_vec = dis_filter.correct({(float)distance,
                                                      (float)delta_distance});
        distance = correct_vec(0);
    }
    else
    {
        cv::Matx21f init_vec = {(float)distance, 0};
        dis_filter.init(init_vec, 1e-2);
        __is_filter_init = true;
    }
    last_distance = distance;
}