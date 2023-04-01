/**
 * @file LogicTools.cpp
 * @author 赖建宇 (lai.jianyu@foxmail.com)
 * @brief
 * @version 1.0
 * @date 2023-03-17
 *
 * @copyright Copyright SCUT RobotLab(c) 2021
 *
 */
#include "LogicTools.h"

using namespace std;

bool LogicTools::waitForNum(int num)
{
    int key = 0;
    cout << "请输入" << num << "并回车以继续" << endl;
    while (key != num)
    {
        cin >> key;
    }
    return 1;
}

bool LogicTools::timeTrigger(int64 &t0, float fps)
{
    int run_time = 1000 * ((cv::getTickCount() - t0) / cv::getTickFrequency());
    int delta_time = 1000.0 / fps;
    // cout << "runtime" << run_time << endl;
    // cout << "delta_time" << delta_time << endl;
    if ((run_time - last_capture_time) >= delta_time)
    {
        last_capture_time = run_time;
        return 1;
    }
    else
    {
        return 0;
    }
}

template <typename T>
int LogicTools::maxMapItem(map<int, T> map, int threshold)
{
    int max_item = 0;
    T max_value = 0;
    for (int i = 0; i < map.size(); i++)
    {
        if (map[i] >= map[max_item])
        {
            max_value = map[i];
            max_item = i;
        }
    }
    if (max_value <= threshold)
    {
        // cout << "max_value" << max_value << endl;
        return -1;
    }
    else
    {
        return max_item;
    }
}

int LogicTools::maxDequeItem(deque<int> &deque, int threshold)
{
    map<int, int> map;
    // 先建立map，在依靠上一个函数查找MaxItem
    for (int i = 0; i < deque.size(); i++)
    {
        map[deque[i]]++;
    }
    // cout << "map_0:" << map[0] << endl;
    int max_item = this->maxMapItem(map, threshold);
    // cout << "max_item" << max_item << endl;
    return max_item;
}