/**
 * @file decider.cpp
 * @author 赖建宇 (lai.jianyu@foxmail.com)
 * @brief
 * @version 1.0
 * @date 2023-03-17
 *
 * @copyright Copyright SCUT RobotLab(c) 2021
 *
 */
#include "decider.h"

/**
 * @brief 判断进入掉帧模式(即中心或区域对焦模式)/物体追踪模式
 *
 * @param face
 * @param dis
 * @param color
 * @param d16
 * @param detect_count
 * @return int DIS
 */
int decider::decide(cv::Mat &color, cv::Mat &d16, face_ptr &__face, object_ptr &__object, dis_ptr &__dis, reader_ptr &__reader)
{
    // 掉帧控制
    if (drop_init)
    {
        drop_count = 0;
    }
    // FPS（目标检测帧率）控制
    if (detect_init)
    {
        detect_count = detect_rate - 1;
    }
    // 复杂的判断过程(待简化)
    int DIS;
    int situation = 0;
    if (detect_count == 0)
    {
        // 进行检测的帧
        // bool detected = __face->faceDetect(color, drop_count);
        bool detected = 1;
        __object->objectDetect(color, drop_count);
        // __object->
        if (detected)
        {
            if (!__face->face_center.empty())
            {
                // 若检测到人脸：锁定人脸（todo：多人脸策略）
                situation = 1;
                cout << "----1----" << endl;
                drop_init = 1; // 重新初始化掉帧计算器
            }
        }
        else
        {
            if (!__dis->target_dis.empty())
            {
                drop_count++;
                if (drop_count >= param.MAX_DROP_FRAME)
                {
                    // 掉帧数超过阈值，则进入掉帧处理
                    situation = 0;
                    cout << "----2----" << endl;
                }
                else
                {
                    if (!__face->face_center.empty())
                    {
                        // 掉帧数低于阈值,且面部队列不为空，则锁定面部队列末尾的点
                        situation = 1;
                        cout << "----3----" << endl;
                    }
                    else
                    {
                        // 掉帧数低于阈值，但面部队列为空，则进入掉帧处理
                        situation = 0;
                        cout << "----4----" << endl;
                    }
                }
            }
            else
            {
                // 距离队列为空，进入掉帧处理
                situation = 0;
                cout << "----5----" << endl;
            }
            // 未检测到，标志位置0
            drop_init = 0;
        }
        detect_init = 1; // 重新初始化面部识别帧率计数器
    }
    else
    {
        if (!__face->face_center.empty())
        {
            if (drop_count < param.MAX_DROP_FRAME)
            {
                // 不检测的帧：锁定人脸队列末尾的点
                situation = 1;
                cout << "----6----" << endl;
            }
            else
            {
                // 掉帧过多，进入掉帧处理
                situation = 0;
                cout << "----7----" << endl;
            }
        }
        else
        {
            // 面部队列为空，进入掉帧处理
            situation = 0;
            cout << "----8----" << endl;
        }
        detect_init = 0;
        detect_count = detect_count - 1; // 计数器递减至0
    }

    // 对判断得到的状态进行决策
    switch (situation)
    {
    case 0:
    {
        // situation=0:进入掉帧处理
        this->dropProcess(param.DROP_PROCESS_MODE, d16, __dis, __reader);
        if (param.DROP_PROCESS_MODE == 1 && param.cam_module == REALSENSE)
        {
            cv::circle(color, cv::Point2f(param.RS_width / 2, param.RS_height / 2), 4, cv::Scalar(0, 0, 255), 5); // 用红色圆点表示对焦位置
        }
        DIS = __dis->target_dis.back();
        __face->face_center.clear();
        break;
    }
    case 1:
    {
        if (param.cam_module == REALSENSE)
        {
            // situation=1:锁定队列末尾的面部
            for (int i = 0; i < __face->face_center.size(); i++)
            {
                DIS = 20000;
                float center_y = __face->face_center.at(i).y;
                float center_x = __face->face_center.at(i).x;
                if (param.INVERT_ON)
                {
                    center_y = param.RS_height - __face->face_center.at(i).y;
                }
                // cout << "list_size-3 " << rsDepthFrames.size() << endl;
                int face_dis = int(1000 * __reader->rsDepthFrames.back().get_distance(center_x, center_y));
                // 对realsense相机来说，discalculate并不承担计算距离的功能
                // 通过第一个int直接传入距离，函数中只是对距离进行滤波和错误处理
                int current_dis = __dis->disCalculate(face_dis, d16, __face->face_center);
                if (current_dis < DIS)
                {
                    DIS = current_dis;
                    __face->target_face_label = i;
                }
            }
            break;
        }
        else
        {
            DIS = __dis->disCalculate(1, d16, __face->face_center);
            break;
        }
    }
    default:
        this->dropProcess(param.DROP_PROCESS_MODE, d16, __dis, __reader);
        break;
    }

    if (DIS != 0)
    {
        if (DIS < 30000)
        {
            // DIS = dis.target_dis.back();
            // cout << "DIS" << DIS << endl;
            if (!__face->face_center.empty())
            {
                cv::circle(color, __face->face_center.at(__face->target_face_label), 4, cv::Scalar(0, 0, 255), 5); // 用红色圆点表示对焦位置
            }
            cv::putText(color, cv::format("%d", DIS), cv::Point2i(15, 30), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 0, 0), 2);
        }
        else
        {
            DIS = 30000;
            cout << "ERROR!-距离过远" << endl;
        }
    }
    else
    {
        DIS = 1000;
        cout << "ERROR!-距离队列异常" << endl;
    }
    return DIS;
}

/**
 * @brief 掉帧（未检测到人脸）处理函数
 *
 * @param mode
 * @param dis
 * @param d16
 */
void decider::dropProcess(int mode, cv::Mat &d16, dis_ptr &__dis, reader_ptr &__reader)
{
    cout << "mode : " << mode << endl;
    switch (mode)
    {
    // 中心点对焦
    case 1:
    {
        deque<cv::Point2f> points;
        if (param.cam_module == ASTRA)
        {
            // 选取中心点
            cv::Point2f center(param.ASTRA_width + param.width_compensate / 2, param.ASTRA_height + param.height_compensate / 2);
            points.push_back(center);
            int center_dis = __dis->disCalculate(0, d16, points);
        }
        if (param.cam_module == REALSENSE)
        {
            int img_width = param.RS_width;
            int img_height = param.RS_height;
            int center_dis = int(1000 * __reader->rsDepthFrames.back().get_distance((img_width + param.width_compensate) / 2, (img_height + param.height_compensate) / 2));
            cout << "CENTER_DIS:" << center_dis << endl;
            __dis->disCalculate(center_dis, d16, points);
        }
        break;
    }
    // 中心重点区域对焦
    case 2:
    {
        deque<cv::Point2f> points;
        if (param.cam_module == ASTRA)
        {
            // 选取中心点
            cv::Point2f center(param.ASTRA_width / 2, param.ASTRA_height / 2);
            points.push_back(center);
            int center_dis = __dis->disCalculate(0, d16, points);
        }
        if (param.cam_module == REALSENSE)
        {
            // 选取1/5 * 1/5中心区域
            int img_width = param.RS_width;
            int img_height = param.RS_height;
            int current_dis = 0;
            int min_dis = int(1000 * __reader->rsDepthFrames.back().get_distance(param.RS_width / 2, param.RS_height / 2));
            for (int i = 0.4 * img_width + param.width_compensate; i <= 0.6 * img_width + param.width_compensate && i >= 0.4 * img_width + param.width_compensate; i++)
            {
                for (int j = 0.4 * img_height + param.height_compensate; j <= 0.6 * img_height + param.height_compensate && j >= 0.4 * img_height + param.height_compensate; j++)
                {
                    // cout << "i:" << i << " j:" << j << endl;
                    current_dis = int(1000 * __reader->rsDepthFrames.back().get_distance(i, j));
                    // cout << "current-dis:" << current_dis << endl;
                    if (current_dis < min_dis && current_dis != 0)
                    {
                        min_dis = current_dis;
                    }
                }
            }
            cout << "MIN-DIS: " << min_dis << endl;
            __dis->disCalculate(min_dis, d16, points);
        }
        break;
    }
    default:
        break;
    }
}

/**
 * @brief 内插法计算目标脉冲
 *
 * @param dis
 * @return int
 */
int decider::disInterPolater(int &dis)
{
    int target_pulse = 0;

    if (dis < 500)
    {
        // 最近对焦0.5m
        target_pulse = 0;
    }
    else if (dis < lens_param.INIT_DIS)
    {
        // 小于最近对焦距离，设为0
        target_pulse = 0;
    }
    else if (dis < 1500 && dis >= lens_param.INIT_DIS)
    {
        // todo:这里貌似有bug(更新：已经修复bug)
        target_pulse = lens_param.A + (lens_param.B - lens_param.A) * (dis - lens_param.INIT_DIS) / (1500 - lens_param.INIT_DIS);
    }
    else if (dis < 2500 && dis >= 1500)
    {
        target_pulse = lens_param.B + (lens_param.C - lens_param.B) * (dis - 1500) / 1000;
    }
    else if (dis < 4000 && dis >= 2500)
    {
        target_pulse = lens_param.C + (lens_param.D - lens_param.C) * (dis - 2500) / 1500;
    }
    else if (dis < 6000 && dis >= 4000)
    {
        target_pulse = lens_param.D + (lens_param.E - lens_param.D) * (dis - 4000) / 2000;
    }
    else if (dis < 8000 && dis >= 6000)
    {
        target_pulse = lens_param.E + (lens_param.F - lens_param.E) * (dis - 6000) / 2000;
    }
    else if (dis < 12000 && dis >= 8000)
    {
        target_pulse = lens_param.F + (lens_param.G - lens_param.F) * (dis - 8000) / 4000;
    }
    else
    {
        target_pulse = lens_param.G;
    }
    return target_pulse;
}