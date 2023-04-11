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

using namespace std;

/**
 * @brief 对焦策略决策器
 *
 * @param t0
 * @param color
 * @param d16
 * @param __detector __face/__object
 * @param __dis
 * @param __reader
 * @param __tool
 * @param control_flag 0:face/1:object
 * @return int
 */
int decider::decide(cv::Mat &d16, detector_ptr &__detector, dis_ptr &__dis, logic_ptr &__tool, bool detected, int control_flag, bool preserve)
{
    int DIS;
    ifstream ifs(this->logicFile.c_str());
    string line;
    while (getline(ifs, line))
    {
        this->logic.push_back(line);
    }
    switch (control_flag)
    {
    case 0:
        facePerceptron(d16, __detector, __dis, __tool, detected);
        break;
    case 1:
        objectPerceptron(d16, __detector, __dis, __tool, detected);
        break;
    default:
        facePerceptron(d16, __detector, __dis, __tool, detected);
        break;
    }

    // 复杂的判断过程(待简化)
    int situation = 0;
    // 掉帧控制
    if (drop_init)
    {
        drop_count = 0;
    }
    if (__tool->timeTrigger(t0, 20))
    {
        // 进行检测的帧
        // bool detected = __face->faceDetect(color, drop_count);
        // bool detected = 1;
        // __object->objectDetect(color, drop_count);
        bool detected = __detector->detect(color);
        // __object->
        if (detected)
        {
            if (!__detector->face_center.empty())
            {
                // 若检测到人脸：锁定人脸
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
                    if (!__detector->face_center.empty())
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
        if (!__detector->face_center.empty())
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
    cout << "face-size" << __detector->face_center.size() << endl;

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
        __detector->face_center.clear();
        break;
    }
    case 1:
    {
        if (param.cam_module == REALSENSE)
        {
            // situation=1:锁定队列末尾距离最近的面部
            for (int i = 0; i < __detector->face_center.back().size(); i++)
            {
                DIS = 20000;
                float center_y = __detector->face_center.back().at(i).y;
                float center_x = __detector->face_center.back().at(i).x;
                if (param.INVERT_ON)
                {
                    center_y = param.RS_height - __detector->face_center.back().at(i).y;
                }
                int face_dis = int(1000 * __reader->rsDepthFrames.back().get_distance(center_x, center_y));
                // 对realsense相机来说，discalculate并不承担计算距离的功能
                // 通过第一个int直接传入距离，函数中只是对距离进行滤波和错误处理
                deque<cv::Point2f> empty;
                int current_dis = __dis->disCalculate(face_dis, d16, empty);
                // 锁定距离最近的面部
                if (current_dis < DIS)
                {
                    DIS = current_dis;
                    __detector->face_label = i;
                }
            }
            break;
        }
        else
        {
            deque<cv::Point2f> empty;
            DIS = __dis->disCalculate(1, d16, empty);
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
            if (!__detector->face_center.empty())
            {
                cv::circle(color, __detector->face_center.back().at(__detector->face_label), 4, cv::Scalar(0, 0, 255), 5); // 用红色圆点表示对焦位置
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
 * @brief 面部识别状态感知器
 *
 * @return int
 */
int decider::facePerceptron(cv::Mat &d16, detector_ptr &__detector, dis_ptr &__dis, logic_ptr &__tool, bool detected)
{
    // 复杂的判断过程(待简化)
    int situation = 0;
    dropInit(face_dropcount);
    // 掉帧控制
    // if ()
    // {
    //     drop_count = 0;
    // }
    if (detected)
    {
        // 简易追踪器：队列末尾重新排序
        int last_deque_size = __detector->face_center.size();
        int last_face_num = 0;
        if (last_deque_size > 1)
        {
            last_face_num = __detector->face_center.at(last_deque_size - 1).size();
        }

        // 队列长度=1时 不做处理
        if (last_deque_size > 1)
        {
            // 三组中心vector,分别为前一次检测的数据、此次检测的数据和等待重新排序的数据
            vector<cv::Point2f> current_centers = __detector->face_center.back();
            vector<cv::Point2f> last_centers = __detector->face_center.at(last_deque_size - 2);
            vector<cv::Point2f> new_centers;
            
            // *****当前一次面部与当前面部数量相同时*****
            if (last_centers.size() == current_centers.size())
            {
                // 匹配
                int label = 0;
                int min_dis = INT_MAX;
                new_centers = current_centers;
                for (int i = 0; i < current_centers.size(); i++)
                {
                    for (int j = 0; j < last_centers.size(); j++)
                    {
                        if (__detector->getPointDis(current_centers.at(i), last_centers.at(j)) < min_dis)
                        {
                            min_dis = __detector->getPointDis(current_centers.at(i), last_centers.at(j));
                            label = j; // label更新为欧氏距离最接近的j
                        }
                    }
                    if (label != i)
                    {
                        new_centers.at(label) = current_centers.at(i);
                    }
                }
                std::cout << "trackstatus-1" << endl;
            }

            // *****当前一次面部比当前面部数量多时（掉帧）*****
            if (last_centers.size() > current_centers.size())
            {
                // 匹配
                int label = 0;
                int min_dis = INT_MAX;
                vector<int> match_label;
                new_centers = last_centers;
                for (int i = 0; i < current_centers.size(); i++)
                {
                    for (int j = 0; j < last_centers.size(); j++)
                    {
                        if (__detector->getPointDis(current_centers.at(i), last_centers.at(j)) < min_dis)
                        {
                            min_dis = __detector->getPointDis(current_centers.at(i), last_centers.at(j));
                            label = j; // label更新为欧氏距离最接近的j
                        }
                    }
                    if (label != i)
                    {
                        new_centers.at(label) = current_centers.at(i);
                        match_label.push_back(label);
                    }
                }
                // 标记掉帧的序列(so uncivilized):
                for (int i = 0; i < last_centers.size(); i++)
                {
                    int max_drop_num = 15;
                    bool is_drop_label = 1;
                    for (int j = 0; j < match_label.size(); j++)
                    {
                        if (i == match_label.at(j))
                        {
                            is_drop_label = 0;
                        }
                    }
                    if (is_drop_label)
                    {
                        face_dropcount.at(i)++;
                    }
                    if (face_dropcount.at(i) > max_drop_num)
                    {
                        face_dropcount.at(i) = 0;
                        auto iter = new_centers.erase(new_centers.begin() + i); // 删除指定元素
                    }
                }
                std::cout << "trackstatus-2" << endl;
            }

            // *****当前一次面部比当前面部数量少时（新增）*****
            if (last_centers.size() < current_centers.size())
            {
                // 匹配
                int label = 0;
                int min_dis = INT_MAX;
                vector<int> match_label;
                vector<cv::Point2f> new_faces; // 新识别到的面部中心
                map<int, int> match_dis;
                new_centers = last_centers;
                for (int i = 0; i < last_centers.size(); i++)
                {
                    match_dis.at(i) = 0;
                }
                for (int i = 0; i < current_centers.size(); i++)
                {
                    for (int j = 0; j < last_centers.size(); j++)
                    {
                        if (__detector->getPointDis(current_centers.at(i), last_centers.at(j)) < min_dis)
                        {
                            min_dis = __detector->getPointDis(current_centers.at(i), last_centers.at(j));
                            label = j;
                        }
                    }
                    // 判断该label是否已经使用过
                    bool isValidLabel = 1;
                    if (match_label.size() > 0)
                    {
                        for (auto lb : match_label)
                        {
                            if (lb == label)
                            {
                                isValidLabel = 0;
                            }
                        }
                    }
                    if (isValidLabel)
                    {
                        // 未使用过
                        new_centers.at(label) = current_centers.at(i);
                        match_label.push_back(label);
                        match_dis.at(label) = min_dis;
                    }
                    else
                    {
                        // 使用过
                        // int dis = __detector->getPointDis(current_centers.at(i),last_centers.at(label));
                        if (min_dis < match_dis.at(label))
                        {
                            // 替换label处的面部
                            match_dis.at(label) = min_dis;
                            new_faces.push_back(new_centers.at(label));
                            new_centers.at(label) = current_centers.at(i);
                        }
                        else
                        {
                            new_faces.push_back(current_centers.at(i));
                        }
                    }
                }
                // 将匹配好的center和新识别到的center拼接在一起
                new_centers.insert(new_centers.end(), new_faces.begin(), new_faces.end());
                std::cout << "trackstatus-3" << endl;
            }
        }
    }
    else
    {
        for (int i = 0; i < face_dropcount.size(); i++)
        {
            face_dropcount.at(i)++;
        }
    }
}

/**
 * @brief 目标识别状态感知器
 *
 * @return int
 */
int decider::objectPerceptron(cv::Mat &d16, detector_ptr &__detector, dis_ptr &__dis, logic_ptr &__tool)
{
    // 复杂的判断过程(待简化)
    int situation = 0;
    // 掉帧控制
    if (drop_init)
    {
        drop_count = 0;
    }
    if (__tool->timeTrigger(t0, 20))
    {
        // 进行检测的帧
        // bool detected = __face->faceDetect(color, drop_count);
        // bool detected = 1;
        // __object->objectDetect(color, drop_count);
        bool detected = __detector->detect(color);
        // __object->
        if (detected)
        {
            if (!__detector->face_center.empty())
            {
                // 若检测到人脸：锁定人脸
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
                    if (!__detector->face_center.empty())
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
        if (!__detector->face_center.empty())
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