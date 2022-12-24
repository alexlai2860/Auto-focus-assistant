/**
 * @file motor_processor.cpp
 * @author 赖建宇 (lai.jianyu@foxmail.com)
 * @brief
 * @version 1.0
 * @date 2022-12-23
 *
 * @copyright Copyright SCUT RobotLab(c) 2021
 *
 */

#include "processor.h"
#include "param.h"

using namespace std;

int Motor::init(Data &data)
{

    TransferData init;
    init.command2 = 0x00;
    data.write(3, init); // 关闭使能
    int key;
    cout << "请将对焦环旋转至最近距离处 随后输入1并回车 " << endl;
    while (key != 1)
    {
        cin >> key;
    }
    data.read(0x01, 0x6B);

    key = -1;
    cout << "请输入镜头编号(1/2/3/4/5) 若新建镜头请按0 回车确认" << endl;
    cin >> key;
    if (key != 0)
    {
        return key - 1;
    }
    cout << "选择新镜头编号(1/2/3/4/5),若重复则抹除之前数据 回车确认" << endl;
}
