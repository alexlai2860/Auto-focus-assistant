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