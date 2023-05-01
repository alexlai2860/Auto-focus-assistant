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
 * @param control_flag 0:对应face检测
 *                     1:对应object检测
 *                     2:不检测的帧
 *                     3/4:直接进入掉帧处理
 * @return int
 */
int decider::decide(cv::Mat &d16, cv::Mat &color, reader_ptr &__reader, detector_ptr &__face, detector_ptr &__object,
                    dis_ptr &__dis, logic_ptr &__tool, bool detected, const int control_flag, int read_result)
{
    int DIS, DIS2;
    int situation;
    int situation_face;
    int situation_object;
    ifstream ifs(this->logicFile.c_str());
    string line;
    while (getline(ifs, line))
    {
        this->logic.push_back(line);
    }

    cout << "detected? " << detected << endl;
    cout << "read_result? " << read_result << endl;
    cout << "face-size-1 ? " << __face->face.size() << endl;
    cout << "target-size-1 ? " << __object->target.size() << endl;

    switch (control_flag)
    {
    case 0:
        cout << "case " << 0 << endl;
        situation_face = facePerceptron(d16, __face, __dis, __tool, detected, control_flag);
        situation_object = objectPerceptron(d16, __object, __dis, __tool, 0, 2);
        situation = situationJudger(situation_face, situation_object, __face, __object);
        cout << "obj-face-overall situation" << situation_object << situation_face << situation << endl;
        break;
    case 1:
        cout << "case " << 1 << endl;
        situation_object = objectPerceptron(d16, __object, __dis, __tool, detected, control_flag);
        situation_face = facePerceptron(d16, __face, __dis, __tool, 0, 2);
        situation = situationJudger(situation_face, situation_object, __face, __object);
        cout << "obj-face-overall situation" << situation_object << situation_face << situation << endl;
        break;
    case 2:
        cout << "case " << 2 << endl;
        situation_face = facePerceptron(d16, __face, __dis, __tool, 0, 2);
        situation_object = objectPerceptron(d16, __object, __dis, __tool, 0, 2);
        situation = situationJudger(situation_face, situation_object, __face, __object);
        cout << "obj-face-overall situation" << situation_object << situation_face << situation << endl;
        break;
    case 3:
        cout << "case " << 3 << endl;
        situation = 0;
        // situation = objectPerceptron(d16, __object, __dis, __tool, detected, control_flag);
        break;
    case 4:
        cout << "case " << 4 << endl;
        situation = 0;
        break;
    default:
        situation = 0;
        break;
    }

    cout << "face-size-2 ? " << __face->face.size() << endl;
    cout << "target-size-2 ? " << __object->target.size() << endl;
    cout << "situation? " << situation << endl;

    // 对判断得到的状态进行决策
    switch (situation)
    {
    case 0:
    {
        // 掉帧处理/无目标检测
        // situation=0:进入掉帧处理
        this->dropProcess(param.DROP_PROCESS_MODE, d16, __dis, __reader);
        if (param.DROP_PROCESS_MODE == 1 && param.cam_module == REALSENSE)
        {
            cv::circle(color, cv::Point2f(param.RS_width / 2, param.RS_height / 2), 4, cv::Scalar(0, 0, 255), 5); // 用红色圆点表示对焦位置
        }
        DIS = __dis->target_dis.back();
        __face->face.clear();
        // __object->target.clear();
        break;
    }
    case 1:
    {
        if (param.cam_module == REALSENSE)
        {
            DIS = 25000;  // 目标距离限制
            DIS2 = 25000; // 目标距离限制
            // 首先判断滚轮有没有动
            int total_result = 0;
            read_result_deque.push_back(read_result);
            if (read_result_deque.size() > 5)
            {
                read_result_deque.pop_front();
            }
            for (auto result : read_result_deque)
            {
                total_result += result;
            }
            int avg_result = total_result / read_result_deque.size();
            cout << "TOTAL_RESULT" << total_result << endl;
            cout << "AVG_RESULT" << avg_result << endl;
            if (abs(avg_result - read_result) > 8)
            {
                // 均值和最新值差大于8，认为滚轮出现滚动
                cout << "DELTA-RESULT:" << abs(avg_result - read_result) << endl;
                this->wheel_moved = 1;
            }
            else
            {
                cout << "DELTA-RESULT:" << abs(avg_result - read_result) << endl;
                this->wheel_moved = 0;
            }
            // situation=1:锁定队列末尾距离最近的目标
            // 更新控制手柄策略

            for (int i = 0; i < __object->target.back().size(); i++)
            {
                if (__object->target.back().at(i).init_trigger == -1)
                {
                    cout << "deciding-target-back-" << i << endl;
                    cout << "target-back-size-" << __object->target.back().size() << endl;
                    int target_dis = __object->target.back().at(i).cam_dis;
                    int face_dis = 0;
                    int dis = target_dis;
                    if (!__object->target.back().at(i).single_face_in_object.empty())
                    {
                        face_dis = __object->target.back().at(i).face_dis;
                    }
                    // cout << "deciding-1" << endl;
                    float dis_ratio = (float)face_dis / (float)target_dis;
                    int delta_dis = abs(face_dis - target_dis);
                    if (dis_ratio >= 0.75 && dis_ratio <= 1.25)
                    {
                        if (delta_dis < 500)
                        {
                            // 认定为面部距离可靠
                            dis = face_dis;
                        }
                    }
                    // cout << "deciding-2" << endl;
                    // 通过第一个int直接传入距离，函数中只是对距离进行滤波和错误处理
                    deque<cv::Point2f> empty;
                    int current_dis = __dis->disCalculate(dis, d16, empty);
                    // 控制手柄策略
                    if (read_result == 0)
                    {
                        // 手柄策略1:对焦在最近的物体上
                        if (current_dis < DIS)
                        {
                            DIS = current_dis;
                            __object->target_label = i;
                        }
                    }
                    else if (read_result <= 9990 && read_result > 0)
                    {
                        // 手柄策略2:根据距离选择
                        // 如果滚轮有移动
                        if (wheel_moved)
                        {
                            // 切换目标:找到和滚轮指示距离最近的目标
                            // 暂时不进行非线性映射
                            if (abs(current_dis - read_result) < DIS2)
                            {
                                DIS2 = abs(current_dis - read_result);
                                __object->target_label = i;
                                last_label = i;
                            }
                        }
                        else if (__object->target.back().size() > last_label)
                        {
                            // 锁定目标
                            __object->target_label = last_label;
                        }
                        else
                        {
                            // 丢失锁定目标:对焦最近的object
                            if (current_dis < DIS)
                            {
                                DIS = current_dis;
                                __object->target_label = i;
                            }
                        }
                    }
                    // cout << "deciding-3" << endl;
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

    // 距离信息判断和绘制
    if (DIS != 0)
    {
        if (DIS < 30000)
        {
            if (situation == 1)
            {
                if (!__object->target.empty())
                {
                    cv::circle(color, __object->target.back().at(__object->target_label).center, 4, cv::Scalar(0, 0, 255), 5); // 用红色圆点表示对焦位置
                }
                cv::putText(color, cv::format("%d", DIS), cv::Point2i(15, 30), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 0, 0), 2);

                // if (!__face->face.empty())
                // {
                //     cv::circle(color, __face->face.back().at(__face->face_label).center, 4, cv::Scalar(0, 0, 255), 5); // 用红色圆点表示对焦位置
                // }
                // cv::putText(color, cv::format("%d", DIS), cv::Point2i(15, 30), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 0, 0), 2);
            }
            // else if (situation == 2)
            // {
            //     if (!__object->target.empty())
            //     {
            //         cv::circle(color, __object->target.back().at(__object->target_label).center, 4, cv::Scalar(0, 0, 255), 5); // 用红色圆点表示对焦位置
            //     }
            //     cv::putText(color, cv::format("%d", DIS), cv::Point2i(15, 30), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 0, 0), 2);
            // }
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
    cout << "decide-done" << endl;
    return DIS;
}

/**
 * @brief 综合状态判别器，计划以目标识别为主，面部识别为辅
 *
 * @param face_situation
 * @param object_situation
 * @param __face
 * @param __object
 * @return int
 */
int decider::situationJudger(int face_situation, int object_situation, detector_ptr &__face, detector_ptr &__object)
{
    // 匹配面部和人体/目标
    int overall_situation = 0;
    if (face_situation == 1 && object_situation == 2)
    {
        // face和object都检测到
        for (int i = 0; i < __face->face.back().size(); i++)
        {
            cv::Mat current_face;
            current_face = __face->face.back().at(i).single_face;
            float current_face_dis = __face->face.back().at(i).cam_dis;
            cv::Rect2i current_face_rect(int(current_face.at<float>(i, 0)), int(current_face.at<float>(i, 1)),
                                         int(current_face.at<float>(i, 2)), int(current_face.at<float>(i, 3)));

            cv::Point2i tl = current_face_rect.tl();
            cv::Point2i tr(current_face_rect.x + current_face_rect.width, current_face_rect.y);
            cv::Point2i dl(current_face_rect.x, current_face_rect.y + current_face_rect.height);
            cv::Point2i dr(current_face_rect.x + current_face_rect.width, current_face_rect.y + current_face_rect.height);
            cv::Point2i center(current_face_rect.x + (current_face_rect.width / 2), current_face_rect.y + (current_face_rect.height / 2));

            for (int j = 0; j < __object->target.back().size(); j++)
            {
                if (__object->target.back().at(j).init_trigger == -1)
                {
                    bool matched_face = 0;
                    bool tl_inside = __object->target.back().at(j).single_object_box.contains(tl);
                    bool tr_inside = __object->target.back().at(j).single_object_box.contains(tr);
                    bool dl_inside = __object->target.back().at(j).single_object_box.contains(dl);
                    bool dr_inside = __object->target.back().at(j).single_object_box.contains(dr);
                    bool center_inside = __object->target.back().at(j).single_object_box.contains(center);
                    matched_face = tl_inside || tr_inside || dl_inside || dr_inside;
                    if (matched_face)
                    {
                        if (__object->target.back().at(j).single_face_in_object.empty())
                        {
                            __object->target.back().at(j).face_dis = current_face_dis;
                            __object->target.back().at(j).single_face_in_object = current_face_rect;
                        }
                        else
                        {
                            // 若不为空，则进一步比较是否满足center_inside
                            if (center_inside)
                            {
                                __object->target.back().at(j).face_dis = current_face_dis;
                                __object->target.back().at(j).single_face_in_object = current_face_rect;
                            }
                        }
                        break;
                    }
                }
            }
        }
        overall_situation = 1;
    }
    else if (object_situation == 2)
    {
        // 只检测到object，未检测到face
        overall_situation = 1;
    }
    else
    {
        overall_situation = 0;
    }
    return overall_situation;
}

/**
 * @brief 面部识别状态感知器/控制situation
 *
 * @return int
 */
int decider::facePerceptron(cv::Mat &d16, detector_ptr &__detector, dis_ptr &__dis, logic_ptr &__tool, bool detected, int control_flag)
{
    cout << "-----FACE_PERCEPTRON-----" << endl;
    // 复杂的判断过程(待简化)
    int situation = 0;
    int max_drop_num = param.MAX_DROP_FRAME_FACE;
    if (!isDropInit)
    {
        dropInit(face_dropcount);
        isDropInit = 1;
        cout << "dropinit" << endl;
    }
    cout << "temp-face-size " << __detector->face.size() << endl;

    // TODO:新数据的重新排序 和 状态感知更新 分离解耦
    if (detected)
    {
        // 简易追踪器：队列末尾重新排序
        int last_deque_size = __detector->face.size();
        int last_face_num = 0;
        if (last_deque_size > 1)
        {
            last_face_num = __detector->face.at(last_deque_size - 1).size();
        }

        // 队列长度>1时:已经初始化
        if (last_deque_size > 1)
        {
            cout << "1-init" << endl;
            // 三组中心vector,分别为前一次检测的数据、此次检测的数据和等待重新排序的数据
            vector<SingleFace> current_faces = __detector->face.back();
            vector<SingleFace> last_faces = __detector->face.at(last_deque_size - 2);

            cout << "last_faces:" << last_faces.size() << endl;
            cout << "current_faces:" << current_faces.size() << endl;

            // 初始化new_faces
            int num = MAX(last_faces.size(), current_faces.size());
            vector<SingleFace> new_faces(num);
            // SingleFace empty_face;
            // for (int i = 0; i < num; i++)
            // {
            //     new_faces.push_back(empty_face);
            // }
            // 配对last_faces和current_faces
            for (int i = 0; i < last_faces.size(); i++)
            {
                for (int j = 0; j < current_faces.size(); j++)
                {
                    if (isSameFace(last_faces.at(i), current_faces.at(j), __tool))
                    {
                        new_faces.at(i) = current_faces.at(j);
                    }
                }
            }
            cout << "stage-1" << endl;
            // 加入临时筛选策略（-2剔除）
            for (int i = 0; i < current_faces.size(); i++)
            {
                if (current_faces.at(i).backward_dis == -2.f)
                {
                    cout << "delect-current-" << i << endl;
                    auto iter = current_faces.erase(current_faces.begin() + i);
                }
            }
            // 检查是否存在未能成功配对的last_faces（先做掉帧处理）
            for (int i = 0; i < last_faces.size(); i++)
            {
                if (last_faces.at(i).forward_dis < 0)
                {
                    // 认为出现掉帧
                    if (new_faces.at(i).backward_dis < 0)
                    {
                        new_faces.at(i) = last_faces.at(i);
                        new_faces.at(i).backward_dis = 0;
                        new_faces.at(i).drop_count++;
                    }
                    else
                    {
                        cout << "掉帧且冲突!!" << endl;
                        cout << "oh,我的上帝,真是见鬼,这真是糟糕透了" << endl;
                    }
                }
                else
                {
                    if (new_faces.at(i).backward_dis < 0)
                    {
                        // 重置掉帧数
                        cout << "??serious??" << endl;
                    }
                    else
                    {
                        // last有forward
                        // new有backward
                        cout << "match" << endl;
                        new_faces.at(i).drop_count = 0;
                    }
                }
            }
            cout << "stage-2" << endl;
            // 检查是否存在未能成功配对的current_face（新增）（后做新增处理）
            for (auto &single_current_face : current_faces)
            {
                cout << "stage-2-1" << endl;
                if (single_current_face.backward_dis < 0)
                {
                    // 认为出现新的face
                    bool uninserted = 1;
                    for (auto &single_new_face : new_faces)
                    {
                        // 随便找个没有前驱节点的newface，赋值
                        if (single_new_face.backward_dis < 0)
                        {
                            single_new_face = single_current_face;
                            single_new_face.backward_dis = 0; // 重要
                            uninserted = 0;
                            cout << "insert-in-vector:" << single_new_face.single_face << endl;
                            break;
                        }
                    }
                    cout << "stage-2-2" << endl;
                    if (uninserted)
                    {
                        // 找不到就新增
                        new_faces.push_back(single_current_face);
                        cout << "pushback-in-vector:" << single_current_face.single_face << endl;
                    }
                }
            }
            cout << "stage-3" << endl;
            // 后处理
            int size = new_faces.size();
            auto new_faces_copy = new_faces;
            int erase_num = 0;
            for (int i = 0; i < size; i++)
            {
                cout << "drop_count_" << i << new_faces.at(i).drop_count << endl;
                if (new_faces.at(i).drop_count > max_drop_num)
                {
                    cout << "erase-" << i << endl;
                    auto iter = new_faces_copy.erase(new_faces_copy.begin() + i - erase_num); // 删除指定元素
                    erase_num++;
                    continue;
                }
                if (new_faces.at(i).single_face.empty())
                {
                    cout << "DANGEROUS!!!-EMPTY-FACE-OCCUR!!!" << endl;
                    auto iter = new_faces_copy.erase(new_faces_copy.begin() + i - erase_num); // 删除指定元素
                    erase_num++;
                    continue;
                }
            }
            cout << "stage-4" << endl;
            cout << "new-faces-copy-size " << new_faces_copy.size() << endl;
            for (auto face : new_faces_copy)
            {
                cout << face.single_face << endl;
            }
            // 完成new_faces的处理
            __detector->face.back() = new_faces_copy;
            if (!new_faces_copy.empty())
            {
                __detector->draw_face_box = 1;
                return 1;
            }
            else
            {
                __detector->draw_face_box = 0;
                return 0;
            }
        }
        else
        {
            // 队列小于或等于1
            // 标准判断drawbox状态语句
            if (!__detector->face.empty())
            {
                if (__detector->face.back().empty())
                {
                    __detector->draw_face_box = 0;
                    cout << "empty" << endl;
                    return 0;
                }
                else
                {
                    __detector->draw_face_box = 1;
                    cout << "!empty" << endl;
                    return 1;
                }
            }
            else
            {
                __detector->draw_face_box = 0;
                return 0;
            }
        }
    }
    else
    {
        // 没检测到目标
        // 当面部时间队列不为空(已经初始化)
        if (!__detector->face.empty())
        {
            cout << "face-back-size1 " << __detector->face.back().size() << endl;
            // 判断是否为进行检测的帧
            if (control_flag == 0)
            {
                for (int i = 0; i < __detector->face.back().size(); i++)
                {
                    __detector->face.back().at(i).drop_count++;
                    cout << "drop_count2_" << i << __detector->face.back().at(i).drop_count << endl;
                    if (__detector->face.back().at(i).drop_count > max_drop_num)
                    {
                        auto iter = __detector->face.back().erase(__detector->face.back().begin() + i); // 删除指定元素
                    }
                    // if (face_dropcount[i] > max_drop_num)
                    // {
                    //     face_dropcount[i] = 0;
                    //     auto iter = __detector->face.back().erase(__detector->face.back().begin() + i); // 删除指定元素
                    // }
                }
            }
        }
        // 标准判断drawbox状态语句
        if (!__detector->face.empty())
        {
            if (__detector->face.back().empty())
            {
                __detector->draw_face_box = 0;
                cout << "empty" << endl;
                return 0;
            }
            else
            {
                __detector->draw_face_box = 1;
                cout << "!empty" << endl;
                return 1;
            }
        }
        else
        {
            __detector->draw_face_box = 0;
            return 0;
        }
    }
}

/**
 * @brief 目标识别状态感知器
 *
 * @return int
 */
int decider::objectPerceptron(cv::Mat &d16, detector_ptr &__detector, dis_ptr &__dis, logic_ptr &__tool, bool detected, int control_flag)
{
    cout << "-----OBJECT_PERCEPTRON-----" << endl;
    // 复杂的判断过程(待简化)
    int situation = 0;
    int max_drop_num = param.MAX_DROP_FRAME_OBJECT;
    if (!isDropInit)
    {
        dropInit(face_dropcount);
        isDropInit = 1;
        cout << "dropinit" << endl;
    }
    // 掉帧控制
    // if ()
    // {
    //     drop_count = 0;
    // }
    cout << "temp-object-size " << __detector->target.size() << endl;

    // TODO:新数据的重新排序 和 状态感知更新 分离解耦
    if (detected)
    {
        // 简易追踪器：队列末尾重新排序
        int last_deque_size = __detector->target.size();
        int last_face_num = 0;
        if (last_deque_size > 1)
        {
            last_face_num = __detector->target.at(last_deque_size - 1).size();
        }

        // 队列长度>1时:已经初始化
        if (last_deque_size > 1)
        {
            cout << "1-init" << endl;
            // 三组中心vector,分别为前一次检测的数据、此次检测的数据和等待重新排序的数据
            vector<SingleObject> current_objects = __detector->target.back();
            vector<SingleObject> last_objects = __detector->target.at(last_deque_size - 2);

            cout << "last_objects_size_1:" << last_objects.size() << endl;
            cout << "current_objects_size_1:" << current_objects.size() << endl;

            // 初始化new_objects
            int num = MAX(last_objects.size(), current_objects.size());
            vector<SingleObject> new_objects(num);
            // SingleObject empty_object;
            // for (int i = 0; i < num; i++)
            // {
            //     new_objects.push_back(empty_object);
            // }
            // 配对last_faces和current_faces
            for (int i = 0; i < last_objects.size(); i++)
            {
                for (int j = 0; j < current_objects.size(); j++)
                {
                    if (isSameObject(last_objects.at(i), current_objects.at(j), __tool))
                    {
                        new_objects.at(i) = current_objects.at(j);
                        new_objects.at(i).init_trigger = last_objects.at(i).init_trigger;
                        if (last_objects.at(i).init_trigger != -1)
                        {
                            new_objects.at(i).init_trigger++;
                        }
                        cout << "last_init_trigger_" << i << ":" << last_objects.at(i).init_trigger << endl;
                        cout << "init_trigger_" << i << ":" << new_objects.at(i).init_trigger << endl;
                    }
                }
            }
            cout << "stage-1" << endl;
            // 加入临时筛选策略（-2剔除）
            for (int i = 0; i < current_objects.size(); i++)
            {
                if (current_objects.at(i).backward_dis == -2.f)
                {
                    // cout << "delect-current-" << i << endl;
                    auto iter = current_objects.erase(current_objects.begin() + i);
                }
            }
            cout << "last_objects_size_2:" << last_objects.size() << endl;
            cout << "current_objects_size_2:" << current_objects.size() << endl;
            // 检查是否存在未能成功配对的last_faces（先做掉帧处理）
            for (int i = 0; i < last_objects.size(); i++)
            {
                if (last_objects.at(i).forward_dis < 0)
                {
                    // 认为出现掉帧
                    if (new_objects.at(i).backward_dis < 0)
                    {
                        cout << "drop_occur!" << endl;
                        new_objects.at(i) = last_objects.at(i);
                        new_objects.at(i).backward_dis = 0;
                        new_objects.at(i).drop_count++;
                        if (new_objects.at(i).init_trigger != -1)
                        {
                            new_objects.at(i).init_trigger = 0;
                        }
                    }
                    else
                    {
                        cout << "last-f<0 & new-b>0" << endl;
                    }
                }
                else
                {
                    if (new_objects.at(i).backward_dis < 0)
                    {
                        // 重置掉帧数
                        cout << "??serious??" << endl;
                    }
                    else
                    {
                        // last有forward
                        // new有backward
                        cout << "match" << endl;
                        new_objects.at(i).drop_count = 0;
                    }
                }
            }
            cout << "stage-2" << endl;
            // 检查是否存在未能成功配对的current_face（新增）（后做新增处理）
            for (auto &single_current_object : current_objects)
            {
                cout << "stage-2-1" << endl;
                if (single_current_object.backward_dis < 0)
                {
                    // 认为single_current_object为新的face
                    if (single_current_object.init_trigger != -1)
                    {
                        single_current_object.init_trigger++;
                    }
                    bool uninserted = 1;
                    for (auto &single_new_object : new_objects)
                    {
                        // 随便找个没有前驱节点的newobject，赋值
                        if (single_new_object.backward_dis < 0)
                        {
                            single_new_object = single_current_object;
                            single_new_object.backward_dis = 0; // 重要
                            uninserted = 0;
                            cout << "insert-in-vector:" << single_new_object.single_object_box << endl;
                            break;
                        }
                    }
                    cout << "stage-2-2" << endl;
                    if (uninserted)
                    {
                        // 找不到就新增
                        new_objects.push_back(single_current_object);
                        cout << "pushback-in-vector:" << single_current_object.single_object_box << endl;
                    }
                }
            }
            cout << "stage-3" << endl;
            // 后处理
            int size = new_objects.size();
            int erase_num = 0;
            auto new_objects_copy = new_objects;
            for (int i = 0; i < size; i++)
            {
                cout << "drop_count_" << i << ":" << new_objects.at(i).drop_count << endl;
                if (new_objects.at(i).drop_count > max_drop_num)
                {
                    cout << "erase-" << i << endl;
                    auto iter = new_objects_copy.erase(new_objects_copy.begin() + i - erase_num); // 删除指定元素
                    erase_num++;
                    continue;
                }
                if (new_objects.at(i).single_object_box.empty())
                {
                    cout << "DANGEROUS!!!-EMPTY-OBJECT-OCCUR!!!" << endl;
                    auto iter = new_objects_copy.erase(new_objects_copy.begin() + i - erase_num); // 删除指定元素
                    erase_num++;
                    continue;
                }
            }
            cout << "stage-4" << endl;
            cout << "new-objects-copy-size " << new_objects_copy.size() << endl;
            for (auto object : new_objects_copy)
            {
                cout << object.single_object_box << endl;
            }
            // init_trigger处理
            bool exist_triggered_object = 0;
            for (auto &object : new_objects_copy)
            {
                cout << "TRIGGER:" << object.init_trigger << endl;
                if (object.init_trigger >= 4 || object.init_trigger == -1)
                {
                    // 允许触发
                    object.init_trigger = -1;
                    exist_triggered_object = 1;
                }
            }
            // 完成new_faces的处理
            __detector->target.back() = new_objects_copy;
            if (!new_objects_copy.empty() && exist_triggered_object)
            {
                __detector->draw_object_box = 1;
                return 2;
            }
            else
            {
                __detector->draw_object_box = 0;
                return 0;
            }
        }
        else
        {
            // 队列小于或等于1
            // 标准判断drawbox状态语句
            if (!__detector->target.empty())
            {
                if (__detector->target.back().empty())
                {
                    __detector->draw_object_box = 0;
                    cout << "empty" << endl;
                    return 0;
                }
                else
                {
                    // 避免误识别，等队列长了再显示和对焦
                    // __detector->draw_object_box = 1;
                    // cout << "!empty" << endl;
                    // return 2;
                    __detector->draw_object_box = 0;
                    cout << "!empty" << endl;
                    return 0;
                }
            }
            else
            {
                __detector->draw_object_box = 0;
                return 0;
            }
        }
    }
    else
    {
        // 没检测到目标
        // 当面部时间队列不为空(已经初始化)
        if (!__detector->target.empty())
        {
            cout << "face-back-size1 " << __detector->target.back().size() << endl;
            // 判断是否为进行检测的帧
            if (control_flag == 1)
            {
                for (int i = 0; i < __detector->target.back().size(); i++)
                {
                    if (__detector->target.back().at(i).init_trigger != -1)
                    {
                        __detector->target.back().at(i).init_trigger = 0;
                    }
                    __detector->target.back().at(i).drop_count++;
                    cout << "drop_count2_" << i << __detector->target.back().at(i).drop_count << endl;
                    if (__detector->target.back().at(i).drop_count > max_drop_num)
                    {
                        auto iter = __detector->target.back().erase(__detector->target.back().begin() + i); // 删除指定元素
                    }
                    // if (face_dropcount[i] > max_drop_num)
                    // {
                    //     face_dropcount[i] = 0;
                    //     auto iter = __detector->face.back().erase(__detector->face.back().begin() + i); // 删除指定元素
                    // }
                }
            }
        }
        // 标准判断drawbox状态语句
        if (!__detector->target.empty())
        {
            if (!__detector->target.back().empty())
            {
                bool exist_triggered_object = 0;
                for (auto target : __detector->target.back())
                {
                    cout << "init-trigger:" << target.init_trigger << endl;
                    if (target.init_trigger == -1)
                    {
                        exist_triggered_object = 1;
                    }
                }
                if (exist_triggered_object)
                {
                    __detector->draw_object_box = 1;
                    cout << "!empty & exist_triggered" << endl;
                    return 2;
                }
                else
                {
                    __detector->draw_object_box = 0;
                    cout << "!empty but all untriggered" << endl;
                    return 0;
                }
            }
            else
            {
                __detector->draw_object_box = 0;
                cout << "empty" << endl;
                return 0;
            }
        }
        else
        {
            __detector->draw_object_box = 0;
            return 0;
        }
    }
}

/**
 * @brief
 *
 * @param last_face
 * @param current_face
 * @return true
 * @return false
 */
bool decider::isSameFace(SingleFace &last_face, SingleFace &current_face, logic_ptr &__tool)
{
    cout << "face-judge-init" << endl;
    cout << "lst-face" << last_face.single_face << endl;
    cout << "crt-face" << current_face.single_face << endl;

    bool is_same_face = 0;
    cv::Rect2i last_face_box(int(last_face.single_face.at<float>(0, 0)), int(last_face.single_face.at<float>(0, 1)),
                             int(last_face.single_face.at<float>(0, 2)), int(last_face.single_face.at<float>(0, 3)));
    cv::Rect2i current_face_box(int(current_face.single_face.at<float>(0, 0)), int(current_face.single_face.at<float>(0, 1)),
                                int(current_face.single_face.at<float>(0, 2)), int(current_face.single_face.at<float>(0, 3)));
    float area_ratio = float(last_face_box.area()) / float(current_face_box.area());
    cv::Point2i last_tl = last_face_box.tl();
    cv::Point2i current_tl = current_face_box.tl();
    cv::Point2i last_center(last_tl.x + last_face_box.x / 2, last_tl.y + last_face_box.y / 2);
    cv::Point2i currnet_center(current_tl.x + current_face_box.x / 2, current_tl.y + current_face_box.y / 2);

    cout << "last-center " << last_center << endl;
    cout << "current-center " << currnet_center << endl;

    float tl_dis = __tool->getPointDis(last_tl, current_tl);
    float center_dis = __tool->getPointDis(last_center, currnet_center);
    float last_cam_dis = last_face.cam_dis;
    float current_cam_dis = current_face.cam_dis;
    cout << "current_dis" << current_cam_dis << endl;
    cout << "last_cam_dis" << last_cam_dis << endl;
    float dis_ratio = last_cam_dis / current_cam_dis;

    // 动态判别参数：x = x0 + b
    float dis_ratio_max = 1.45 + (last_face.drop_count + 1) / 25; // +1避免出现0
    float dis_ratio_min = 0.65 - (last_face.drop_count + 1) / 50; // 避免小于0
    if (dis_ratio == 0)
    {
        dis_ratio_min = -1; // 异常情况：自动关闭dis_ratio_judge
    }
    // dis_ratio_max = 1000;
    // dis_ratio_min = -1; // 手动关闭dis_ratio_judge

    float area_ratio_max = param.AREA_RATIO_COEF + (last_face.drop_count - 3) / 20;
    float area_ratio_min = 1 / param.AREA_RATIO_COEF - (last_face.drop_count - 3) / 40;
    float tl_dis_max = MAX(float(last_face_box.width), float(last_face_box.height)) / (param.POINT_DIS_COEF - 0.05 * (last_face.drop_count));
    float center_dis_max = MAX(float(last_face_box.width), float(last_face_box.height)) / (param.POINT_DIS_COEF - 0.05 * (last_face.drop_count));

    // 判别条件
    if (dis_ratio < dis_ratio_max && dis_ratio > dis_ratio_min)
    {
        if (area_ratio < area_ratio_max && area_ratio > area_ratio_min)
        {
            if (tl_dis < tl_dis_max)
            {
                if (center_dis < center_dis_max)
                {
                    if (last_face.forward_dis < 0)
                    {
                        // last不存在后继节点(面部)
                        if (current_face.backward_dis < 0)
                        {
                            // 且current不存在前驱节点，则配对成功
                            is_same_face = 1;
                            last_face.forward_dis = center_dis;
                            last_face.forward_face = &current_face;
                            current_face.backward_dis = center_dis;
                            current_face.backward_face = &last_face;
                        }
                        else
                        {
                            // current已经使用过
                            cout << "current has been used" << endl;
                            is_same_face = 0;
                            // current_face.backward_dis = -2.f;
                        }
                    }
                    else
                    {
                        // last已经使用过
                        cout << "last has been used" << endl;
                        // 小概率，待完善：目标过于密集才会出现
                        // last存在后继节点：判断
                        is_same_face = 0;
                        current_face.backward_dis = -2.f; // <0 且不同于-1
                        if (last_face.forward_dis > center_dis)
                        {
                            // 更新后继节点
                            is_same_face = 1;
                            last_face.forward_dis = center_dis;
                            last_face.forward_face = &current_face;
                            current_face.backward_dis = center_dis;
                            current_face.backward_face = &last_face;
                            // 重置原先后继节点的backward设置为-1
                            // 注意这里使用指针，要求目标的地址不能改变
                            last_face.forward_face->backward_dis = -1.f;
                            last_face.forward_face->backward_face = nullptr;
                        }
                        else
                        {
                            // 不更新后继节点
                        }
                    }
                }
                else
                {
                    // cout << "center_dis_unmatch:" << center_dis << endl;
                }
            }
            else
            {
                // cout << "tl_dis_unmatch:" << tl_dis << endl;
            }
        }
        else
        {
            // cout << "area lst" << last_face_box.area() << endl;
            // cout << "area crt" << current_face_box.area() << endl;
            // cout << "area_ratio_unmatch:" << area_ratio << endl;
        }
    }
    else
    {
        // cout << "dis_ratio_unmatch:" << dis_ratio << endl;
    }
    cout << "judge-complete:" << is_same_face << endl;
    return is_same_face;
}

/**
 * @brief
 *
 * @param last_object
 * @param current_object
 * @return true
 * @return false
 */
bool decider::isSameObject(SingleObject &last_object, SingleObject &current_object, logic_ptr &__tool)
{
    cout << "obj-judge-init" << endl;

    bool is_same_object = 0;
    cv::Rect2i last_object_box = last_object.single_object_box;
    cv::Rect2i current_object_box = current_object.single_object_box;
    float area_ratio = float(last_object_box.area()) / float(current_object_box.area());
    cv::Point2i last_tl = last_object_box.tl();
    cv::Point2i current_tl = current_object_box.tl();
    cv::Point2i last_center = last_object.center;
    cv::Point2i currnet_center = current_object.center;

    cout << "last-center " << last_center << endl;
    cout << "current-center " << currnet_center << endl;

    float tl_dis = __tool->getPointDis(last_tl, current_tl);
    float center_dis = __tool->getPointDis(last_center, currnet_center);
    float last_cam_dis = last_object.cam_dis;
    float current_cam_dis = current_object.cam_dis;
    cout << "current_dis" << current_cam_dis << endl;
    cout << "last_cam_dis" << last_cam_dis << endl;
    float dis_ratio = last_cam_dis / current_cam_dis;

    // 动态判别参数：x = x0 + b
    float dis_ratio_max = 1.45 + (last_object.drop_count + 1) / 25; // +1避免出现0
    float dis_ratio_min = 0.65 - (last_object.drop_count + 1) / 50; // 避免小于0
    if (dis_ratio == 0)
    {
        dis_ratio_min = -1; // 异常情况：关闭dis_ratio_judge
    }
    dis_ratio_max = 10000;
    dis_ratio_min = -1; // 手动关闭dis_ratio_judge

    float area_ratio_max = param.AREA_RATIO_COEF + (last_object.drop_count - 3) / 20;
    float area_ratio_min = 1 / param.AREA_RATIO_COEF - (last_object.drop_count - 3) / 40;
    float tl_dis_max = MAX(float(last_object_box.width), float(last_object_box.height)) / (param.POINT_DIS_COEF - 0.05 * (last_object.drop_count));
    float center_dis_max = MAX(float(last_object_box.width), float(last_object_box.height)) / (param.POINT_DIS_COEF - 0.05 * (last_object.drop_count));

    // 判别条件
    // if (dis_ratio < dis_ratio_max && dis_ratio > dis_ratio_min)
    // {
    if (area_ratio < area_ratio_max && area_ratio > area_ratio_min)
    {
        if (tl_dis < tl_dis_max)
        {
            if (center_dis < center_dis_max)
            {
                if (last_object.forward_dis < 0)
                {
                    // last不存在后继节点(object)
                    if (current_object.backward_dis < 0)
                    {
                        // 且current不存在前驱节点，则配对成功
                        is_same_object = 1;
                        last_object.forward_dis = center_dis;
                        last_object.forward_object = &current_object;
                        current_object.backward_dis = center_dis;
                        current_object.backward_object = &last_object;
                    }
                    else
                    {
                        // current已经使用过
                        // 不匹配，返回0
                        cout << "current has been used" << endl;
                        is_same_object = 0;
                        // current_face.backward_dis = -2.f;
                    }
                }
                else
                {
                    // last已经使用过
                    // 小概率，待完善：目标过于密集才会出现
                    // 直接删除current_object（认定为是重复的框）
                    cout << "last has been used" << endl;
                    // last存在后继节点：判断
                    is_same_object = 0;
                    current_object.backward_dis = -2.f; // <0 且不同于-1
                    if (last_object.forward_dis > center_dis)
                    {
                        // 更新后继节点
                        is_same_object = 1;
                        last_object.forward_dis = center_dis;
                        last_object.forward_object = &current_object;
                        current_object.backward_dis = center_dis;
                        current_object.backward_object = &last_object;
                        // 重置原先后继节点的backward设置为-1
                        // 注意这里使用指针，要求目标的地址不能改变
                        last_object.forward_object->backward_dis = -1.f;
                        last_object.forward_object->backward_object = nullptr;
                    }
                    else
                    {
                        // 不更新后继节点
                    }
                }
            }
            else
            {
                cout << "center_dis_unmatch:" << center_dis << endl;
            }
        }
        else
        {
            cout << "tl_dis_unmatch:" << tl_dis << endl;
        }
    }
    else
    {
        cout << "area lst" << last_object_box.area() << endl;
        cout << "area crt" << current_object_box.area() << endl;
        cout << "area_ratio_unmatch:" << area_ratio << endl;
    }
    // }
    // else
    // {
    //     cout << "dis_ratio_unmatch:" << dis_ratio << endl;
    // }
    cout << "judge-complete:" << is_same_object << endl;
    return is_same_object;
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
    int closest_pulse;
    // 通过正方向，反推最近距离脉冲
    if ((lens_param.B - lens_param.A) < 0)
    {
        closest_pulse = 9999;
    }
    else
    {
        closest_pulse = 0;
    }

    if (dis < lens_param.INIT_DIS)
    {
        // 小于最近对焦距离，设为0
        // target_pulse = 0;
        // target_pulse = lens_param.A;
        target_pulse = closest_pulse;
    }
    else if (dis < 500 && dis > lens_param.INIT_DIS)
    {
        // 最近对焦0.5m
        // target_pulse = 0;
        // target_pulse = lens_param.A;
        target_pulse = closest_pulse + (lens_param.A - closest_pulse) * (dis - lens_param.INIT_DIS) / (500 - lens_param.INIT_DIS);
    }
    else if (dis < 1500 && dis >= lens_param.INIT_DIS)
    {
        // todo:这里貌似有bug(更新：已经修复bug)
        if (lens_param.INIT_DIS > 0.5)
        {
            // 此时lens_param.A应该为接近9999/0
            target_pulse = lens_param.A + (lens_param.B - lens_param.A) * (dis - lens_param.INIT_DIS) / (1500 - lens_param.INIT_DIS);
        }
        else
        {
            target_pulse = lens_param.A + (lens_param.B - lens_param.A) * (dis - 500) / 1000;
        }
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