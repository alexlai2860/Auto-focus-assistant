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

void Dis::disCalculate(int mode, cv::Mat &d16, deque<cv::Point2f> &points)
{
    // select the point
    uint16_t dis = d16.at<uint16_t>(int(points.back().y), int(points.back().x));
    // uint16_t dis = d16.at<uint16_t>(393,232);
    cout << "selected_point: " << points.back() << endl;
    // output and store the distance
    int face_dis = int(dis) * 8000 / 65535;
    // ERROR process
    if (face_dis != 0)
    {
        this->face_dis.push_back(face_dis);
        cout << "dis : " << face_dis << endl;
        if (this->face_dis.size() >= param.DIS_DEQUE)
        {
            this->face_dis.pop_front();
        }
    }
    else if (!this->face_dis.empty())
    {
        this->face_dis.push_back(this->face_dis.back());
        if (this->face_dis.size() >= param.DIS_DEQUE)
        {
            this->face_dis.pop_front();
        }
        cout << "dis : " << this->face_dis.back() << " (currection) " << endl;
        // TransferData data1;
        // data1.command1 = 0x00;
        // SteppingMotor motor;
        // motor.write(1, data1);
    }
    else
    {
        cout << "OUT OF DETECT LIMIT" << endl;
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
    if (!this->face_dis.empty())
    {
        int min_dis = 8000, max_dis = 0;
        // find the max and min distance in the last few seconds
        for (int i = 0; i < this->face_dis.size(); i++)
        {
            if (face_dis.at(i) < min_dis)
            {
                min_dis = face_dis.at(i);
            }
            if (face_dis.at(i) > max_dis)
            {
                max_dis = face_dis.at(i);
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