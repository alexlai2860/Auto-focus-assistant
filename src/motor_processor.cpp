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
#include "lens_param.h"
#include "param.h"

using namespace std;

int Motor::init(Data &data, TransferData &readData, TransferData &writeData)
{
    writeData.command2 = 0x00;
    data.write(3, writeData); // 关闭使能
    int key = 0, lens_num = 0;
    cout << "请将对焦环旋转至最近距离处 随后输入1并回车 " << endl;
    while (key != 1)
    {
        cin >> key;
    }
    int init_pulse = this->readPulse(data);
    int compensate = init_pulse - lens_param.INIT_PULSE_1; // 补偿脉冲，用于矫正拟合函数的常数项
    lens_param.INIT_PULSE_1 = init_pulse;
    lens_param.INFINIT_PULSE_1 = lens_param.INFINIT_PULSE_1 + compensate;
    lens_param.write();

    // cv::waitKey(3);
    // this->writePulse(3000, data);

    key = -1;
    cout << "请输入镜头编号(1/2/3/4/5) 若新建镜头请按0 回车确认" << endl;
    cin >> key;
    if (key != 0)
    {
        lens_num = key;
        return lens_num;
    }
    else
    {
        cout << "选择新镜头编号(1/2/3/4/5),若重复将覆盖之前数据 回车确认" << endl;
        cin >> key;
        lens_num = key;
        key = -1;
        cout << "请将对焦环旋转至最远距离处 随后输入1并回车" << endl;
        while (key != 1)
        {
            cin >> key;
        }
        int infinit_pulse = this->readPulse(data);
        lens_param.INFINIT_PULSE_1 = infinit_pulse;
        lens_param.write();
        return -lens_num; // 此时进入拟合函数
    }
    return 0;
}

int Motor::readPulse(Data &data)
{
    TransferData readData, writeData;
    int32_t pulse = 4000000;
    int round = 0;
    while (pulse >= 4000000)
    {
        writeData.command1 = 0x33;
        data.write(2, writeData);
        cv::waitKey(3); // 限制发送速率，根据电脑运行速度和波特率进行调整(默认为3,默认比特率为115200)(update:取消while循环)
        readData = data.read(0x01, 0x6B);
        // data.write(2, writeData);
        pulse = (((int32_t)readData.read1[0] << 24) |
                 ((int32_t)readData.read1[1] << 16) |
                 ((int32_t)readData.read1[2] << 8) |
                 ((int32_t)readData.read1[3] << 0));
        // cout << "current pulse : " << signed(pulse) << endl;
        round++;
        if (round >= 10)
        {
            cout << "ERROR" << endl;
            return 4000000;
        }
    }
    return signed(pulse);
}

void Motor::writePulse(int pulse_num, Data &data)
{
    TransferData writeData, readData;
    // writeData.command2 = 0x01;
    // data.write(3, writeData); // 打开使能
    // cv::waitKey(3);
    // step1 确定正方向和旋转方向(待观察)
    bool positive_direction = (lens_param.INFINIT_PULSE_1 - lens_param.INIT_PULSE_1 > 0);
    bool direction = 0;
    if (positive_direction && pulse_num < 0)
    {
        pulse_num = -pulse_num;
        direction = 0;
    }
    else if (positive_direction && pulse_num < 0)
    {
        pulse_num = -pulse_num;
        direction = 0;
    }
    else if (!positive_direction && pulse_num > 0)
    {
        direction = 1;
    }
    else
    {
        direction = 1;
    }

    // step2 确定脉冲数目 : 十进制转十六进制
    uint32_t u32 = uint32_t(pulse_num);
    // cout <<  << endl;
    uint8_t pulse[4] = {0};
    pulse[0] = u32 & 0xFF;
    pulse[1] = (u32 >> 8) & 0xFF;
    pulse[2] = (u32 >> 16) & 0xFF;
    pulse[3] = (u32 >> 24) & 0xFF;

    if (direction = 1)
    {
        writeData.direction_and_speed1 = 0x10;
    }
    else
    {
        writeData.direction_and_speed1 = 0x00;
    }
    writeData.direction_and_speed2 = 0x02;
    writeData.pulse_h = pulse[2];
    writeData.pulse_m = pulse[1];
    writeData.pulse_l = pulse[0];

    data.write(4, writeData);
}

cv::Mat Motor::polyFit(vector<cv::Point2f> &points, int n)
{
    // test - 输入拟合点
    // std::vector<cv::Point> points;

    cv::Mat curve;
    // Number of key points
    int N = points.size();
    // 构造矩阵X
    cv::Mat X = cv::Mat::zeros(n + 1, n + 1, CV_64FC1);
    for (int i = 0; i < n + 1; i++)
    {
        for (int j = 0; j < n + 1; j++)
        {
            for (int k = 0; k < N; k++)
            {
                X.at<double>(i, j) = X.at<double>(i, j) +
                                     std::pow(points[k].x, i + j);
            }
        }
    }
    // 构造矩阵Y
    cv::Mat Y = cv::Mat::zeros(n + 1, 1, CV_64FC1);
    for (int i = 0; i < n + 1; i++)
    {
        for (int k = 0; k < N; k++)
        {
            Y.at<double>(i, 0) = Y.at<double>(i, 0) +
                                 std::pow(points[k].x, i) * points[k].y;
        }
    }

    curve = cv::Mat::zeros(n + 1, 1, CV_64FC1);
    // 求解矩阵A
    cv::solve(X, Y, curve, cv::DECOMP_LU);
    // cout << setprecision(5) << curve.at<double>(0,0) << endl;
    lens_param.A_1 = curve.at<double>(0, 3);
    lens_param.B_1 = curve.at<double>(0, 2);
    lens_param.C_1 = curve.at<double>(0, 1);
    lens_param.D_1 = curve.at<double>(0, 0);
    lens_param.write();
    cout << "curve = " << curve << endl;
    return curve;
}

void Motor::calibration(cv::VideoCapture &colorStream, cv::VideoCapture &depthStream, Face &face, Dis &dis, int64 &t0, Data &data)
{
    std::list<Frame> depthFrames, colorFrames;
    const std::size_t maxFrames = 32;
    // Synchronization objects
    std::mutex mtx;
    std::condition_variable dataReady;
    std::atomic<bool> isFinish;
    TransferData readData, writeData;
    Motor motor;
    int round = 0;
    cout << "************** calibration initalized *****************" << endl;
    cout << "现在进入校准程序,请旋转对焦环并带动电机齿轮，将相机对焦在彩色图像中 蓝色中心点 所瞄准的目标上,随后长按 g 采集" << endl;
    cout << "至少采集5组数据校准(如0.6m、1m、2m、4m、6m),便于拟合函数,数据越多则拟合精度越高" << endl;
    cout << "采集完毕后,长按 c 进行拟合" << endl;
    cout << "按输入1并回车,开始校准" << endl;
    cout << "ps1: 请关注图片上的距离值,单位为mm,若与实际情况偏差过大请重新校准" << endl;
    cout << "ps2: 校准完毕后会自动退出,请重新启动程序;校准过程中按esc退出" << endl;
    int key0;
    while (key0 != 1)
    {
        cin >> key0;
    }

    isFinish = false;
    while (!isFinish)
    {
        // Start depth reading thread
        std::thread depthReader([&]
                                {
         while (!isFinish)
         {
             // Grab and decode new frame
             if (depthStream.grab())
             {
                 Frame f;
                 f.timestamp = cv::getTickCount();
                 depthStream.retrieve(f.frame, cv::CAP_OPENNI_DEPTH_MAP);
                 if (f.frame.empty())
                 {
                     cerr << "ERROR: Failed to decode frame from depth stream" << endl;
                     break;
                 }

                 {
                     std::lock_guard<std::mutex> lk(mtx);
                     if (depthFrames.size() >= maxFrames)
                         depthFrames.pop_front();
                     depthFrames.push_back(f);
                 }
                 dataReady.notify_one();
             }
         } });

        // Start color reading thread
        std::thread colorReader([&]
                                {
         while (!isFinish)
         {
             // Grab and decode new frame
             if (colorStream.grab())
             {
                 Frame f;
                 f.timestamp = cv::getTickCount();
                 colorStream.retrieve(f.frame);
                 if (f.frame.empty())
                 {
                     cerr << "ERROR: Failed to decode frame from color stream" << endl;
                     break;
                 }
                 
                 {
                     std::lock_guard<std::mutex> lk(mtx);
                     if (colorFrames.size() >= maxFrames)
                         colorFrames.pop_front();
                     colorFrames.push_back(f);
                 }
                 dataReady.notify_one();
             }
         } });
        while (!isFinish)
        {
            std::unique_lock<std::mutex> lk(mtx);
            while (!isFinish && (depthFrames.empty() || colorFrames.empty()))
            {
                dataReady.wait(lk);
            }
            while (!depthFrames.empty() && !colorFrames.empty())
            {
                if (!lk.owns_lock())
                    lk.lock();

                int64 t1 = cv::getTickCount();
                // Get a frame from the list
                Frame depthFrame = depthFrames.front();
                int64 depthT = depthFrame.timestamp;

                // Get a frame from the list
                Frame colorFrame = colorFrames.front();
                int64 colorT = colorFrame.timestamp;

                // Half of frame period is a maximum time diff between frames
                const int64 maxTdiff = int64(1000000000 / (2 * colorStream.get(cv::CAP_PROP_FPS)));
                if (depthT + maxTdiff < colorT)
                {
                    depthFrames.pop_front();
                    continue;
                }
                else if (colorT + maxTdiff < depthT)
                {
                    colorFrames.pop_front();
                    continue;
                }
                depthFrames.pop_front();
                colorFrames.pop_front();
                lk.unlock();

                cv::Mat d8, d16, dColor;

                // convert to dcolor frame(d8) and high resolution depth frame(d16)
                depthFrame.frame.convertTo(d8, CV_8U, 255.0 / 8000);
                applyColorMap(d8, dColor, cv::COLORMAP_RAINBOW);
                depthFrame.frame.convertTo(d16, CV_16U, 65535.0 / 8000); // the effective measure distance is 8000mm

                // 选取中心点
                deque<cv::Point2f> points;
                cv::Point2f center(320, 240);
                points.push_back(center);

                int center_dis = dis.disCalculate(0, d16, points);
                int current_pulse = motor.readPulse(data);

                cv::putText(colorFrame.frame, cv::format("%d", center_dis), cv::Point2i(30, 30), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(255, 255, 255), 2);
                // cout << "center_dis : " << center_dis << endl;
                cv::circle(colorFrame.frame, center, 4, cv::Scalar(255, 0, 0), -1);

                char key1 = (char)cv::waitKey(1);
                // cout << int(key1) << endl;
                int x;
                if (key1 == 'g')
                {
                    cal_points.push_back(cv::Point2f(center_dis, current_pulse));
                    cout << cal_points.back() << endl;
                }
                if (key1 == 'c')
                {
                    if (cal_points.size() >= 5)
                    {
                        float current_A = lens_param.A_1;
                        this->polyFit(cal_points, 3);
                        if (current_A != lens_param.A_1)
                        {
                            cout << "校准完毕" << endl;
                            isFinish = true;
                            break;
                        }
                    }
                    else
                    {
                        cout << "样本点过少，请继续添加" << endl;
                    }
                }

                // show dcolor frame
                imshow("Depth (colored)", dColor);

                // Show color frame
                imshow("Color", colorFrame.frame);

                // Exit on Esc key press
                if (key1 == 27) // ESC
                {
                    isFinish = true;
                    break;
                }
                // cout << "run time = " << 1000 * ((cv::getTickCount() - t1) / cv::getTickFrequency()) << " ms" << endl;
            }
        }
    }
}