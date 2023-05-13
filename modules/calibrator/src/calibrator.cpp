/**
 * @file calibrator.cpp
 * @author 赖建宇 (lai.jianyu@foxmail.com)
 * @brief
 * @version 1.0
 * @date 2023-01-06
 *
 * @copyright Copyright SCUT RobotLab(c) 2021
 *
 */

#include "calibrator.h"
#include "SteppingMotor.h"
#include "LensParam.h"
#include "param.h"

bool Calibrator::calibrate(int lens_num, dis_ptr &__dis, motor_ptr &__motor, rs2::depth_frame &depth, cv::Mat &color)
{
    // 选取中心点
    cv::Mat d8, d16, dColor;
    int DIS = 0;

    deque<cv::Point2f> points;
    cv::Point2f center(param.RS_width / 2, param.RS_height / 2);
    points.push_back(center);

    int center_dis = int(1000 * depth.get_distance(param.RS_width / 2, param.RS_height / 2));
    __dis->disCalculate(center_dis, d16, points);

    __motor->read();
    int current_pulse = __motor->readPosition();

    cv::putText(color, cv::format("%d", center_dis), cv::Point2i(30, 30), cv::FONT_HERSHEY_SIMPLEX, 2, cv::Scalar(200, 200, 255), 3);
    cv::circle(color, center, 6, cv::Scalar(0, 0, 255), -1);
    switch (cal_points.size())
    {
    case 0:
        cv::putText(color, "cal at 0.5m", cv::Point2i(param.RS_width / 2, param.RS_height - 50), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(200, 200, 255), 2);
        break;
    case 1:
        cv::putText(color, "cal at 1.5m", cv::Point2i(param.RS_width / 2, param.RS_height - 50), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(200, 200, 255), 2);
        break;
    case 2:
        cv::putText(color, "cal at 2.5m", cv::Point2i(param.RS_width / 2, param.RS_height - 50), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(200, 200, 255), 2);
        break;
    case 3:
        cv::putText(color, "cal at 4.0m", cv::Point2i(param.RS_width / 2, param.RS_height - 50), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(200, 200, 255), 2);
        break;
    case 4:
        cv::putText(color, "cal at 6.0m", cv::Point2i(param.RS_width / 2, param.RS_height - 50), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(200, 200, 255), 2);
        break;
    case 5:
        cv::putText(color, "cal at 8.0m", cv::Point2i(param.RS_width / 2, param.RS_height - 50), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(200, 200, 255), 2);
        break;
    case 6:
        cv::putText(color, "cal at infinity position", cv::Point2i(param.RS_width / 2, param.RS_height - 50), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(200, 200, 255), 2);
        break;
    case 7:
        cv::putText(color, "cal at nearest position", cv::Point2i(param.RS_width / 2, param.RS_height - 50), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(200, 200, 255), 2);
        break;
    case 8:
        cv::putText(color, "press C to load param", cv::Point2i(param.RS_width / 2, param.RS_height - 50), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(200, 200, 255), 2);
        break;

    default:
        break;
    }

    char key1 = (char)cv::waitKey(1);
    int x;
    if (key1 == 'g')
    {
        // cv::waitKey(1);
        if (!cal_points.empty())
        {
            if (current_pulse != cal_points.back().y)
            {
                cal_points.push_back(cv::Point2f(center_dis, current_pulse));
                cout << cal_points.back() << endl;
            }
        }
        else
        {
            cal_points.push_back(cv::Point2f(center_dis, current_pulse));
            cout << cal_points.back() << endl;
        }
    }
    if (key1 == 'c')
    {
        int n = 6;
        switch (-lens_num)
        {
        case 1:
            for (int i = 0; i <= n; i++)
            {
                lens_param.LENS_1.at<double>(0, i) = cal_points[i].y;
            }
            lens_param.LENS_1.at<double>(0, 7) = cal_points[7].x;
            break;
        case 2:
            for (int i = 0; i <= n; i++)
            {
                lens_param.LENS_2.at<double>(0, i) = cal_points[i].y;
            }
            lens_param.LENS_2.at<double>(0, 7) = cal_points[7].x;
            break;
        case 3:
            for (int i = 0; i <= n; i++)
            {
                lens_param.LENS_3.at<double>(0, i) = cal_points[i].y;
            }
            lens_param.LENS_3.at<double>(0, 7) = cal_points[7].x;
            break;
        case 4:
            for (int i = 0; i <= n; i++)
            {
                lens_param.LENS_4.at<double>(0, i) = cal_points[i].y;
            }
            lens_param.LENS_4.at<double>(0, 7) = cal_points[7].x;
            break;
        case 5:
            for (int i = 0; i <= n; i++)
            {
                lens_param.LENS_5.at<double>(0, i) = cal_points[i].y;
            }
            lens_param.LENS_5.at<double>(0, 7) = cal_points[7].x;
            break;
        default:
            break;
        }
        lens_param.write();
        cout << "param load success" << endl;
        return 1;
        // break;
    }
    return 0;
}

bool Calibrator::nucleusCalibrate(int lens_num, dis_ptr &__dis, motor_ptr &__motor, rs2::depth_frame &depth, cv::Mat &color)
{
    // 选取中心点
    cv::Mat d8, d16, dColor;
    int DIS = 0;

    deque<cv::Point2f> points;
    cv::Point2f center(param.RS_width / 2, param.RS_height / 2);
    points.push_back(center);

    int center_dis = int(1000 * depth.get_distance(param.RS_width / 2, param.RS_height / 2));
    __dis->disCalculate(center_dis, d16, points);
    int current_pulse = 0;
    __motor->read();
    int read_position = __motor->readPosition();
    int read_command = __motor->readCommand();
    if (read_position >= 0)
    {
        current_pulse = read_position;
        this->last_pulse = current_pulse;
        __motor->write(read_position, 0);
    }

    cv::putText(color, cv::format("%d", center_dis), cv::Point2i(50, 50), cv::FONT_HERSHEY_SIMPLEX, 2, cv::Scalar(200, 200, 255), 2);
    cv::circle(color, center, 6, cv::Scalar(0, 0, 255), -1);
    switch (cal_points.size())
    {
    case 0:
        cv::putText(color, "cal at 0.5m", cv::Point2i(param.RS_width / 2, param.RS_height - 50), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(200, 200, 255), 2);
        break;
    case 1:
        cv::putText(color, "cal at 1.5m", cv::Point2i(param.RS_width / 2, param.RS_height - 50), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(200, 200, 255), 2);
        break;
    case 2:
        cv::putText(color, "cal at 2.5m", cv::Point2i(param.RS_width / 2, param.RS_height - 50), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(200, 200, 255), 2);
        break;
    case 3:
        cv::putText(color, "cal at 4.0m", cv::Point2i(param.RS_width / 2, param.RS_height - 50), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(200, 200, 255), 2);
        break;
    case 4:
        cv::putText(color, "cal at 6.0m", cv::Point2i(param.RS_width / 2, param.RS_height - 50), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(200, 200, 255), 2);
        break;
    case 5:
        cv::putText(color, "cal at 8.0m", cv::Point2i(param.RS_width / 2, param.RS_height - 50), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(200, 200, 255), 2);
        break;
    case 6:
        cv::putText(color, "cal at infinity position", cv::Point2i(param.RS_width / 2, param.RS_height - 50), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(200, 200, 255), 2);
        break;
    case 7:
        cv::putText(color, "cal at nearest position", cv::Point2i(param.RS_width / 2, param.RS_height - 50), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(200, 200, 255), 2);
        break;
    case 8:
        cv::putText(color, "press REC to load param", cv::Point2i(param.RS_width / 2, param.RS_height - 50), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(200, 200, 255), 2);
        break;

    default:
        break;
    }

    char key1 = (char)cv::waitKey(1);
    int x;
    // if (key1 == 'g')
    if (read_command == -2 && last_pulse != 0)
    {
        // cv::waitKey(1);
        if (!cal_points.empty())
        {
            if (last_pulse > (cal_points.back().y - 5) && last_pulse < (cal_points.back().y + 5))
            {
                cout << "脉冲差值过小！" << endl;
            }
            else
            {
                cal_points.push_back(cv::Point2f(center_dis, last_pulse));
                cout << cal_points.back() << endl;
            }
        }
        else
        {
            cal_points.push_back(cv::Point2f(center_dis, last_pulse));
            cout << cal_points.back() << endl;
        }
    }

    // if (key1 == 'c')
    if (read_command == -3 || read_command == -4)
    {
        int n = 6;
        switch (-lens_num)
        {
        case 1:
            for (int i = 0; i <= n; i++)
            {
                lens_param.LENS_1.at<double>(0, i) = cal_points[i].y;
            }
            lens_param.LENS_1.at<double>(0, 7) = cal_points[7].x;
            break;
        case 2:
            for (int i = 0; i <= n; i++)
            {
                lens_param.LENS_2.at<double>(0, i) = cal_points[i].y;
            }
            lens_param.LENS_2.at<double>(0, 7) = cal_points[7].x;
            break;
        case 3:
            for (int i = 0; i <= n; i++)
            {
                lens_param.LENS_3.at<double>(0, i) = cal_points[i].y;
            }
            lens_param.LENS_3.at<double>(0, 7) = cal_points[7].x;
            break;
        case 4:
            for (int i = 0; i <= n; i++)
            {
                lens_param.LENS_4.at<double>(0, i) = cal_points[i].y;
            }
            lens_param.LENS_4.at<double>(0, 7) = cal_points[7].x;
            break;
        case 5:
            for (int i = 0; i <= n; i++)
            {
                lens_param.LENS_5.at<double>(0, i) = cal_points[i].y;
            }
            lens_param.LENS_5.at<double>(0, 7) = cal_points[7].x;
            break;
        default:
            break;
        }
        lens_param.write();
        cout << "param load success" << endl;
        return 1;
        // break;
    }
    return 0;
}

/**
 * @brief 曲线拟合函数，已弃用
 *
 * @param points
 * @param n
 * @param lens_num
 * @return cv::Mat
 */
cv::Mat Calibrator::polyFit(vector<cv::Point2f> &points, int n, int lens_num)
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
    switch (lens_num)
    {
    case 1:
        for (int i = 0; i <= n; i++)
        {
            lens_param.LENS_1.at<double>(0, i + 1) = curve.at<double>(0, n - i);
        }
        break;
    case 2:
        for (int i = 0; i <= n; i++)
        {
            lens_param.LENS_2.at<double>(0, i + 1) = curve.at<double>(0, n - i);
        }
        break;
    case 3:
        for (int i = 0; i <= n; i++)
        {
            lens_param.LENS_3.at<double>(0, i + 1) = curve.at<double>(0, n - i); // 4次（测试）
        }
        break;
    case 4:
        for (int i = 0; i <= n; i++)
        {
            lens_param.LENS_4.at<double>(0, i + 1) = curve.at<double>(0, n - i);
        }
        break;
    case 5:
        for (int i = 0; i <= n; i++)
        {
            lens_param.LENS_5.at<double>(0, i + 1) = curve.at<double>(0, n - i);
        }
        break;
    default:
        break;
    }
    lens_param.write();
    cout << "curve = " << curve << endl;
    return curve;
}

// void Calibrator::astraCalibration(int lens_num, Dis &dis, int64 &t0)
// {
// cv::VideoCapture depthStream(cv::CAP_OPENNI2_ASTRA);
// cv::VideoCapture colorStream(4, cv::CAP_V4L2);

// colorStream.set(cv::CAP_PROP_FRAME_WIDTH, 640);
// colorStream.set(cv::CAP_PROP_FRAME_HEIGHT, 480);
// depthStream.set(cv::CAP_PROP_FRAME_WIDTH, 640);
// depthStream.set(cv::CAP_PROP_FRAME_HEIGHT, 480);
// depthStream.set(cv::CAP_PROP_OPENNI2_MIRROR, 0);

// Frame frame;
// const std::size_t maxFrames = 32;
// // Synchronization objects
// std::mutex mtx;
// std::condition_variable dataReady;
// std::atomic<bool> isFinish;
// TransferData readData, writeData;
// // Motor motor;
// int round = 0;
// cout << "************** calibration initalized *****************" << endl;
// cout << "现在进入校准程序,请旋转对焦环并带动电机齿轮，将相机对焦在彩色图像中 蓝色中心点 所瞄准的目标上,随后长按 g 采集" << endl;
// cout << "至少采集5组数据校准(如0.6m、1m、2m、4m、6m),便于拟合函数,数据越多则拟合精度越高" << endl;
// cout << "采集完毕后,长按 c 进行拟合" << endl;
// cout << "按输入1并回车,开始校准" << endl;
// cout << "ps1: 请关注图片上的距离值,单位为mm,若与实际情况偏差过大请重新校准" << endl;
// cout << "ps2: 校准完毕后会自动退出,请重新启动程序;校准过程中按esc退出" << endl;
// int key0;
// while (key0 != 1)
// {
//     cin >> key0;
// }

// isFinish = false;
// while (!isFinish)
// {
//     std::thread depthReader([&]
//                             {
//      while (!isFinish)
//      {
//          // Grab and decode new frame
//          if (depthStream.grab())
//          {
//              AstraFrame f;
//              f.timestamp = cv::getTickCount();
//              depthStream.retrieve(f.frame, cv::CAP_OPENNI_DEPTH_MAP);
//              if (f.frame.empty())
//              {
//                  cerr << "ERROR: Failed to decode frame from depth stream" << endl;
//                  break;
//              }

//              {
//                  std::lock_guard<std::mutex> lk(mtx);
//                  if (frame.astraDepthFrames.size() >= maxFrames)
//                      frame.astraDepthFrames.pop_front();
//                  frame.astraDepthFrames.push_back(f);
//              }
//              dataReady.notify_one();
//          }
//      } });

//     // Start color reading thread
//     std::thread colorReader([&]
//                             {
//      while (!isFinish)
//      {
//          // Grab and decode new frame
//          if (colorStream.grab())
//          {
//              AstraFrame f;
//              f.timestamp = cv::getTickCount();
//              colorStream.retrieve(f.frame);
//              if (f.frame.empty())
//              {
//                  cerr << "ERROR: Failed to decode frame from color stream" << endl;
//                  break;
//              }

//              {
//                  std::lock_guard<std::mutex> lk(mtx);
//                  if (frame.astraColorFrames.size() >= maxFrames)
//                      frame.astraColorFrames.pop_front();
//                  frame.astraColorFrames.push_back(f);
//              }
//              dataReady.notify_one();
//          }
//      } });
//     while (!isFinish)
//     {
//         std::unique_lock<std::mutex> lk(mtx);
//         while (!isFinish && (frame.astraDepthFrames.empty() || frame.astraColorFrames.empty()))
//         {
//             dataReady.wait(lk);
//         }
//         while (!frame.astraDepthFrames.empty() && !frame.astraColorFrames.empty())
//         {
//             if (!lk.owns_lock())
//                 lk.lock();

//             int64 t1 = cv::getTickCount();
//             // Get a frame from the list
//             AstraFrame depthFrame = frame.astraDepthFrames.front();
//             int64 depthT = depthFrame.timestamp;

//             // Get a frame from the list
//             AstraFrame colorFrame = frame.astraColorFrames.front();
//             int64 colorT = colorFrame.timestamp;

//             // Half of frame period is a maximum time diff between frames
//             const int64 maxTdiff = int64(1000000000 / (2 * colorStream.get(cv::CAP_PROP_FPS)));
//             if (depthT + maxTdiff < colorT)
//             {
//                 frame.astraDepthFrames.pop_front();
//                 continue;
//             }
//             else if (colorT + maxTdiff < depthT)
//             {
//                 frame.astraColorFrames.pop_front();
//                 continue;
//             }
//             frame.astraDepthFrames.pop_front();
//             frame.astraColorFrames.pop_front();
//             lk.unlock();

//             cv::Mat d8, d16, dColor;

//             // convert to dcolor frame(d8) and high resolution depth frame(d16)
//             depthFrame.frame.convertTo(d8, CV_8U, 255.0 / 8000);
//             applyColorMap(d8, dColor, cv::COLORMAP_RAINBOW);
//             depthFrame.frame.convertTo(d16, CV_16U, 65535.0 / 8000); // the effective measure distance is 8000mm

//             // 选取中心点
//             deque<cv::Point2f> points;
//             cv::Point2f center(320, 240);
//             points.push_back(center);

//             int center_dis = dis.disCalculate(0, d16, points);
//             int current_pulse = __motor->read(data);

//             cv::putText(colorFrame.frame, cv::format("%d", center_dis), cv::Point2i(30, 30), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(255, 255, 255), 2);
//             // cout << "center_dis : " << center_dis << endl;
//             cv::circle(colorFrame.frame, center, 4, cv::Scalar(255, 0, 0), -1);

//             char key1 = (char)cv::waitKey(1);
//             // cout << int(key1) << endl;
//             int x;
//             if (key1 == 'g')
//             {
//                 if (!cal_points.empty())
//                 {
//                     if (current_pulse != cal_points.back().y)
//                     {
//                         cal_points.push_back(cv::Point2f(center_dis, current_pulse));
//                         cout << cal_points.back() << endl;
//                     }
//                 }
//                 else
//                 {
//                     cal_points.push_back(cv::Point2f(center_dis, current_pulse));
//                     cout << cal_points.back() << endl;
//                 }
//             }
//             if (key1 == 'c')
//             {
//                 if (cal_points.size() >= 5)
//                 {
//                     float current_A = lens_param.A;
//                     this->polyFit(cal_points, 5, -lens_num); // 输入的lens_num为负值
//                     if (current_A != lens_param.A)
//                     {
//                         cout << "校准完毕" << endl;
//                         isFinish = true;
//                         break;
//                     }
//                 }
//                 else
//                 {
//                     cout << "样本点过少，请继续添加" << endl;
//                 }
//             }

//             // show dcolor frame
//             imshow("Depth (colored)", dColor);

//             // Show color frame
//             imshow("Color", colorFrame.frame);

//             // Exit on Esc key press
//             if (key1 == 27) // ESC
//             {
//                 isFinish = true;
//                 break;
//             }
//             // cout << "run time = " << 1000 * ((cv::getTickCount() - t1) / cv::getTickFrequency()) << " ms" << endl;
//         }
//     }
// }
// }
