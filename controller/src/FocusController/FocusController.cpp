/**
 * @file FocusController.cpp
 * @author 赖建宇 (lai.jianyu@foxmail.com)
 * @brief
 * @version 1.0
 * @date 2023-03-12
 *
 * @copyright Copyright SCUT RobotLab(c) 2021
 *
 */

#include "FocusController.h"
#include "param.h"
#include "LensParam.h"
#include <iostream>
#include <fstream>
#include <string>

using namespace std;

int FocusController::CalInit(int64 &t0) { return 0; }

bool FocusController::FocusInit(int64 &t0, int &lens_num)
{
    Face face1;
    Dis dis1;
    TransferData writeData;
    __data = make_shared<Data>();

    // 每次循环：先读取当前电机位置
    // 随后根据目标距离解算电机脉冲目标值，求差
    // 写入串口，驱动电机
    // (waitkey后)写入串口，准备读取
    if (lens_num > 0)
    {
        writeData.command2 = 0x01;
        __data->write(3, writeData); // 打开使能
        cv::waitKey(3);
        // motor.setZero(data); // 电机置零
        if (param.cam_module == ASTRA)
        {
            astraProcessFrame(t0);
        }
        if (param.cam_module == REALSENSE)
        {
            rsProcessFrame(t0);
        }
    }
    else
    {
        cout << "lens_num_error" << endl;
    }
    return 1;
}

/**
 * @brief Realsense相机 帧处理函数
 *
 * @param face
 * @param dis
 * @param t0
 * @param data
 */
void FocusController::rsProcessFrame(int64 &t0)
{
    int key;
    int round = 0;
    int fps = param.FPS; // support 1,2,3,5,6,10,15,30
    int detect_rate = 30 / fps;
    __reader = make_shared<RsReader>();
    __motor = make_shared<SteppingMotor>();
    __dis = make_shared<Dis>();
    __face = make_shared<Face>();

    // 相机初始化
    __reader->camInit();

    // 在指定时间后再进入循环，避免相机初始化未完成导致程序崩溃
    cv::waitKey(2000);

    while (key != 27)
    {
        int64 t1 = cv::getTickCount();
        __reader->read();

        int DIS = 0;
        cv::Mat d16, dColor;
        cv::Mat color = __reader->color;
        cv::Mat depth = __reader->depth;
        if (drop_init)
        {
            drop_count = 0;
        }

        // FPS（目标检测帧率）控制
        if (detect_init)
        {
            detect_count = detect_rate - 1;
        }

        // 决策，判断该帧是否需要进行目标检测&采取的对焦策略
        DIS = Decider(color, d16, detect_count);

        // 读取当前脉冲值
        int current_pulse = __motor->readPulse();

        // 计算目标脉冲值-方案一:使用四次函数拟合，五次函数不稳定，A暂时废弃
        // int target_pulse = (lens_param.B * pow(DIS, 4) + lens_param.C * pow(DIS, 3) + lens_param.D * pow(DIS, 2) + lens_param.E * DIS + lens_param.F);

        // 计算目标脉冲值-方案二:使用插值法
        int target_pulse = disInterPolater(DIS);
        cv::putText(color, cv::format("%d", target_pulse), cv::Point2i(15, 60), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 0, 0), 2);

        // 计算差值，写入串口，同时进行异常处理，驱动镜头
        __motor->writePulse((target_pulse - current_pulse));

        // 输出彩色图和深度图
        imshow("Depth", depth * 15);
        imshow("Color", color);

        // 计算运行时间
        int key = cv::waitKey(1);
        float run_time = 1000 * ((cv::getTickCount() - t1) / cv::getTickFrequency());
        cout << "run time = " << run_time << " ms" << endl;
    }
}

// /**
//  * @brief Realsense 相机读取函数
//  *
//  * @param pipe
//  * @param frames
//  */
// void FocusController::rs_read(rs2::pipeline &pipe, rs2::frameset &frames)
// {
//     // int64 t1 = cv::getTickCount();

//     frames = pipe.wait_for_frames();
//     // Get each frame
//     rs2::frame color_frame = frames.get_color_frame();
//     rs2::depth_frame depth_frame = frames.get_depth_frame();
//     // rs2::video_frame ir_frame_left = frames.get_infrared_frame(1);
//     // rs2::video_frame ir_frame_right = frames.get_infrared_frame(2);

//     rsDepthFrames.push_back(depth_frame);
//     rsColorFrames.push_back(color_frame);
//     if (rsColorFrames.size() > 2)
//     {
//         rsColorFrames.pop_front();
//     }
//     if (rsDepthFrames.size() > 2)
//     {
//         rsDepthFrames.pop_front();
//     }
//     // cout << "run time = " << 1000 * ((cv::getTickCount() - t1) / cv::getTickFrequency()) << " ms" << endl;
// }

/**
 * @brief 掉帧（未检测到人脸）处理函数
 *
 * @param mode
 * @param dis
 * @param d16
 */
void FocusController::dropProcess(int mode, cv::Mat &d16)
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
 * @brief 判断进入掉帧模式(即中心或区域对焦模式)/物体追踪模式
 *
 * @param face
 * @param dis
 * @param color
 * @param d16
 * @param detect_count
 * @return int DIS
 */
int FocusController::Decider(cv::Mat &color, cv::Mat &d16, int &detect_count)
{
    // 复杂的判断过程(待简化)
    int DIS;
    int situation = 0;
    if (detect_count == 0)
    {
        // 进行检测的帧
        bool detected = __face->faceDetect(color, __face->detected_faces, this->drop_count);
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
        this->dropProcess(param.DROP_PROCESS_MODE, d16);
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
        this->dropProcess(param.DROP_PROCESS_MODE, d16);
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
 * @brief 内插法计算目标脉冲
 *
 * @param dis
 * @return int
 */
int FocusController::disInterPolater(int &dis)
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

void FocusController::astraProcessFrame(int64 &t0)
{
    // cv::VideoCapture depthStream(cv::CAP_OPENNI2_ASTRA);
    // cv::VideoCapture colorStream(4, cv::CAP_V4L2);

    // colorStream.set(cv::CAP_PROP_FRAME_WIDTH, param.ASTRA_width);
    // colorStream.set(cv::CAP_PROP_FRAME_HEIGHT, param.ASTRA_height);
    // depthStream.set(cv::CAP_PROP_FRAME_WIDTH, param.ASTRA_width);
    // depthStream.set(cv::CAP_PROP_FRAME_HEIGHT, param.ASTRA_height);
    // depthStream.set(cv::CAP_PROP_OPENNI2_MIRROR, 0);
    // cout << "Depth stream: "
    //      << depthStream.get(cv::CAP_PROP_FRAME_WIDTH) << "x" << depthStream.get(cv::CAP_PROP_FRAME_HEIGHT)
    //      << " @" << depthStream.get(cv::CAP_PROP_FPS) << " fps" << endl;

    // const std::size_t maxFrames = 32;
    // // Synchronization objects
    // std::mutex mtx;
    // std::condition_variable dataReady;
    // std::atomic<bool> isFinish;
    // TransferData readData, writeData;
    // // Motor motor;
    // int round = 0;

    // isFinish = false;

    // // Start depth reading thread
    // std::thread depthReader([&]
    //                         {
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
    //                  if (astraDepthFrames.size() >= maxFrames)
    //                  {
    //                     astraDepthFrames.pop_front();
    //                  }
    //                  if(param.INVERT_ON)
    //                  {
    //                     cv::flip(f.frame,f.frame,0);
    //                  }
    //                  astraDepthFrames.push_back(f);
    //              }
    //              dataReady.notify_one();
    //          }
    //      } });

    // // Start color reading thread
    // std::thread colorReader([&]
    //                         {
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
    //                  if (astraColorFrames.size() >= maxFrames)
    //                  {
    //                      astraColorFrames.pop_front();
    //                  }
    //                  if(param.INVERT_ON)
    //                  {
    //                     cv::flip(f.frame,f.frame,0);
    //                  }
    //                  astraColorFrames.push_back(f);
    //              }
    //              dataReady.notify_one();
    //          }
    //      } });
    // while (!isFinish)
    // {
    //     std::unique_lock<std::mutex> lk(mtx);
    //     while (!isFinish && (astraDepthFrames.empty() || astraColorFrames.empty()))
    //     {
    //         dataReady.wait(lk);
    //     }
    //     while (!astraDepthFrames.empty() && !astraColorFrames.empty())
    //     {
    //         if (!lk.owns_lock())
    //             lk.lock();

    //         int64 t1 = cv::getTickCount();
    //         // Get a frame from the list
    //         AstraFrame depthFrame = astraDepthFrames.front();
    //         int64 depthT = depthFrame.timestamp;

    //         // Get a frame from the list
    //         AstraFrame colorFrame = astraColorFrames.front();
    //         int64 colorT = colorFrame.timestamp;

    //         // Half of frame period is a maximum time diff between frames
    //         const int64 maxTdiff = int64(1000000000 / (2 * colorStream.get(cv::CAP_PROP_FPS)));
    //         if (depthT + maxTdiff < colorT)
    //         {
    //             astraDepthFrames.pop_front();
    //             continue;
    //         }
    //         else if (colorT + maxTdiff < depthT)
    //         {
    //             astraColorFrames.pop_front();
    //             continue;
    //         }
    //         astraDepthFrames.pop_front();
    //         astraColorFrames.pop_front();
    //         lk.unlock();

    //         cv::Mat d8, d16, dColor;
    //         int DIS = 0;
    //         // this->depth_frames = depthFrames;
    //         // this->color_frames = colorFrames;
    //         if (drop_init)
    //         {
    //             drop_count = 0;
    //         }

    //         // convert to dcolor frame(d8) and high resolution depth frame(d16)
    //         depthFrame.frame.convertTo(d8, CV_8U, 255.0 / 8000);
    //         applyColorMap(d8, dColor, cv::COLORMAP_RAINBOW);
    //         depthFrame.frame.convertTo(d16, CV_16U, 65535.0 / 8000); // the effective measure distance is 8000mm

    //         // FPS control
    //         int fps = param.FPS; // support 1,2,3,5,6,10,15,30
    //         int detect_rate = 30 / fps;
    //         if (detect_init)
    //         {
    //             detect_count = detect_rate - 1;
    //         }
    //         // cout << this->detect_count << endl;

    //         DIS = Decider(face, dis, colorFrame.frame, d16, detect_count);
    //         // 计算差值，写入串口，同时进行异常处理
    //         __motor = make_shared<SteppingMotor>();
    //         int current_pulse = __motor->readPulse(data);
    //         // 方案二:使用插值法
    //         int target_pulse = disInterPolater(DIS);
    //         cv::putText(colorFrame.frame, cv::format("%d", target_pulse), cv::Point2i(15, 60), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 0, 0), 2);

    //         __motor->writePulse((target_pulse - current_pulse), data);
    //         // int target_pulse = (lens_param.B * pow(DIS, 4) + lens_param.C * pow(DIS, 3) + lens_param.D * pow(DIS, 2) + lens_param.E * DIS + lens_param.F);
    //         // // cout << "current_pulse = " << current_pulse << endl;
    //         // // cout << "target_pulse = " << target_pulse << endl;
    //         // if (abs(target_pulse - current_pulse) < abs(lens_param.INFINIT_PULSE - lens_param.INIT_PULSE))
    //         // {
    //         //     int min_pulse = (lens_param.INFINIT_PULSE < lens_param.INIT_PULSE ? lens_param.INFINIT_PULSE : lens_param.INIT_PULSE);
    //         //     int max_pulse = (lens_param.INFINIT_PULSE > lens_param.INIT_PULSE ? lens_param.INFINIT_PULSE : lens_param.INIT_PULSE);
    //         //     if (target_pulse < max_pulse && target_pulse > min_pulse)
    //         //     {
    //         //         if (current_pulse <= max_pulse + 50 && current_pulse >= min_pulse - 50)
    //         //         {
    //         //             motor.writePulse((target_pulse - current_pulse), data);
    //         //             cout << "写入中" << endl;
    //         //         }
    //         //         else
    //         //         {
    //         //             cout << "ERROR!-当前值异常" << endl;
    //         //         }
    //         //     }
    //         //     else
    //         //     {
    //         //         cout << "ERROR!-目标值异常" << endl;
    //         //     }
    //         // }
    //         // else
    //         // {
    //         //     cout << "ERROR!-差值过大" << endl;
    //         // }

    //         // show dcolor frame
    //         imshow("Depth (colored)", dColor);

    //         // Show color frame
    //         imshow("Color", colorFrame.frame);

    //         // Exit on Esc key press
    //         int key = cv::waitKey(1);
    //         if (key == 27) // ESC
    //         {
    //             isFinish = true;
    //             // break;
    //         }
    //         cout << "run time = " << 1000 * ((cv::getTickCount() - t1) / cv::getTickFrequency()) << " ms" << endl;
    //     }
    // }
}
