/**
 * @file SerialPort.cpp
 * @author 杨泽霖 (scut.bigeyoung@qq.com)
 * @brief Linux串口类
 * @version 2.0
 * @date 2018-12-08
 *
 * @copyright Copyright South China Tiger(c) 2018
 *
 */

#include "SerialPort.h"
#include <iostream>
#include <unistd.h>
#include <stdio.h>
#include <dirent.h>
#include <string.h>
#include <sys/stat.h>
#include <stdlib.h>
#include <string>

using namespace std;

/**
 * @brief 构建一个新的串口实例
 * 将自动调用 open() 函数
 * @param baud_rate 波特率，以大写B开头
 * 拥有默认值 B115200
 */
SerialPort::SerialPort(int baud_rate)
{
    this->baud_rate = baud_rate;
    open();
}

/**
 * @brief 打开串口
 * 自动搜索所有可用设备，并尝试打开第一个
 */
void SerialPort::open()
{
    this->is_open = false;

    DIR *dir1 = nullptr;
    DIR *dir2 = nullptr;
    struct dirent *dire = nullptr;
    struct dirent *dire1 = nullptr;
    struct dirent *dire2 = nullptr;
    string file_name;
    string tilta_motor;
    string tilta_hand_unit;
    const char dir_path[] = "/dev/";
    if ((dir1 = opendir(dir_path)) != nullptr)
    {
        cout << "1" << endl;
        while ((dire1 = readdir(dir1)) != nullptr)
        {
            // tilta
            if (strstr(dire1->d_name, "ttyUSB") != nullptr)
            {
                tilta_hand_unit = dire1->d_name;
                // cout << "hand-unit-name:" << tilta_hand_unit << endl;
                break;
            }
        }
        closedir(dir1);
    }
    // if ((dir2 = opendir(dir_path)) != nullptr)
    // {
    //     cout << "2" << endl;
    //     while ((dire2 = readdir(dir2)) != nullptr)
    //     {
    //         // tilta
    //         if (strstr(dire2->d_name, "nucleusn_motor") != nullptr)
    //         {
    //             tilta_motor = dire2->d_name;
    //             // cout << "motor-name:" << tilta_motor << endl;
    //             break;
    //         }
    //     }
    //     closedir(dir2);
    // }

    // // stepping motor
    // if (file_name.empty())
    // {
    //     cout << "找不到串口" << endl;
    //     return;
    // }
    // else
    //     file_name = dir_path + file_name;

    // tilta
    // if (tilta_motor.empty())
    // {
    //     cout << "找不到原力N电机" << endl;
    //     return;
    // }
    if (tilta_hand_unit.empty())
    {
        cout << "找不到原力N" << endl;
        return;
    }
    else
    {
        // tilta_motor = dir_path + tilta_motor;
        tilta_hand_unit = dir_path + tilta_hand_unit;
    }

    // cout << "正在打开串口 " << file_name << endl;
    // cout << "正在打开电机 " << tilta_motor << endl;
    cout << "正在打开原力N " << tilta_hand_unit << endl;

    // fd = ::open(file_name.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);        // 非堵塞情况，steppingmotor
    // fd1 = ::open(tilta_motor.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);     // tilta电机
    fd2 = ::open(tilta_hand_unit.c_str(), O_RDWR | O_NOCTTY | O_NDELAY); // tilta手柄

    // // stepping motor
    // if (fd == -1)
    // {
    //     perror("\033[31m串口打开失败");
    //     printf("\033[0m");
    //     return;
    // }
    // tcgetattr(fd, &option);

    // tilta
    // if (fd1 == -1)
    // {
    //     perror("\033[31m串口打开失败");
    //     printf("\033[0m");
    //     return;
    // }
    // tcgetattr(fd1, &option);
    if (fd2 == -1)
    {
        perror("\033[31m串口打开失败");
        printf("\033[0m");
        return;
    }
    tcgetattr(fd2, &option);

    // 修改所获得的参数
    option.c_iflag = 0;                 // 原始输入模式
    option.c_oflag = 0;                 // 原始输出模式
    option.c_lflag = 0;                 // 关闭终端模式
    option.c_cflag |= (CLOCAL | CREAD); // 设置控制模式状态，本地连接，接收使能
    option.c_cflag &= ~CSIZE;           // 字符长度，设置数据位之前一定要屏掉这个位
    option.c_cflag &= ~CRTSCTS;         // 无硬件流控
    option.c_cflag |= CS8;              // 8位数据长度
    option.c_cflag &= ~CSTOPB;          // 1位停止位
    option.c_cc[VTIME] = 0;
    option.c_cc[VMIN] = 0;
    cfsetospeed(&option, baud_rate); // 设置输入波特率
    cfsetispeed(&option, baud_rate); // 设置输出波特率

    // 设置新属性，TCSANOW：所有改变立即生效
    // tcsetattr(fd, TCSANOW, &option);
    // tcsetattr(fd1, TCSANOW, &option);
    tcsetattr(fd2, TCSANOW, &option);

    this->is_open = true;
}

SerialPort::~SerialPort()
{
    close();
}

void SerialPort::close()
{
    // if (this->is_open)
    //     ::close(fd);

    // if (this->is_open)
    //     ::close(fd1);
    if (this->is_open)
        ::close(fd2);

    this->is_open = false;
}

/**
 * @brief 写入数据
 * @param data 数据起始位置
 * @param len 想要写入的数据长度
 * @return bool 是否完整写入
 */
ssize_t SerialPort::write(void *data, size_t len)
{
    // cout << "数据长度" << len << endl;
    ssize_t len_result = -1;
    if (is_open)
    {
        // tcflush(fd, TCOFLUSH); // 清空，防止数据累积在缓存区
        // len_result = ::write(fd, data, len);

        // tcflush(fd1, TCOFLUSH); // 清空，防止数据累积在缓存区
        // len_result = ::write(fd1, data, len);
        tcflush(fd2, TCOFLUSH); // 清空，防止数据累积在缓存区
        len_result = ::write(fd2, data, len);
    }

    if (len_result != static_cast<ssize_t>(len))
    {
        cout << "无法写入串口，重新打开中..." << endl;
        open();
    }
    else
    {
        // cout << "写入数据成功" << endl;
    }

    return len_result;
}

/**
 * @brief 读取数据
 * @param data 数据起始位置
 * @param len 想要读取的数据长度
 * @return ssize_t 读取到的长度
 */
ssize_t SerialPort::read(void *data, size_t len)
{
    ssize_t len_result = -1;

    if (is_open)
    {
        // len_result = ::read(fd, data, len);
        // tcflush(fd, TCIFLUSH);

        len_result = ::read(fd2, data, len);
        tcflush(fd2, TCIFLUSH);
    }

    if (len_result == -1)
    {
        cout << "无法读取串口，重新打开中..." << endl;
        open();
    }
    else if (len_result == 0)
    {
        // cout << "串口读取 空" << endl;
    }
    else
    {
        // cout << "串口读取成功" << endl;
    }
    return len_result;
}
