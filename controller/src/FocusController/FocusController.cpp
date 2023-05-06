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

int FocusController::init(int64 &t0, int lens_num)
{
    Face face1;
    Dis dis1;
    TransferData writeData;
    __data = make_shared<Data>();
    __dis = make_shared<Dis>();
    // __motor = make_shared<SteppingMotor>();
    __motor = make_shared<NucleusN>();
    __logic = make_shared<LogicTools>();
    // __motor->init(writeData, writeData);
    // __decider = make_shared<decider>();

    // 每次循环：先读取当前电机位置
    // 随后根据目标距离解算电机脉冲目标值，求差
    // 写入串口，驱动电机
    // (waitkey后)写入串口，准备读取
    if (lens_num > 0)
    {
        // __motor->enable();
        // motor.setZero(data); // 电机置零
        if (param.cam_module == ASTRA)
        {
            // astraProcessFrame(t0);
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
 * @brief 深度图重投影至波形图
 *
 * @param depth
 * @param af_dis
 * @param mf_dis
 */
void FocusController::depthReProjection(cv::Mat &depth, int af_dis, int mf_dis)
{
    // rows:480 cols:848
    cv::Mat reproject(depth.rows, (depth.cols / 4), CV_8UC1, cv::Scalar(0)); // 单通道,大小同depth
    float scale = (float)depth.rows / (float)(8 * 1000);                     // 最远显示为8m
    cout << "scale" << scale << endl;

    // for (int i = 0; i < depth.cols; i += 4)
    // {
    //     for (int j = 0; j < depth.rows; j++)
    //     {
    //         int dis = depth.at<uint16_t>(j, i);
    //         if (dis < 8000)
    //         {
    //             // cout << "i:" << i << endl;
    //             if (reproject.at<uint8_t>((int)(scale * dis), i / 4) <= 255)
    //             {
    //                 reproject.at<uint8_t>((int)(scale * dis), i / 4) += 8;
    //             }
    //         }
    //     }
    // }

    for (int j = 0; j < depth.rows; j++)
    {
        uint16_t *row_j = depth.ptr<uint16_t>(j);
        for (int i = 0; i < depth.cols; i += 4)
        {
            int dis = row_j[i];
            if (dis < 8000)
            {
                // cout << "i:" << i << endl;
                if (reproject.at<uint8_t>((int)(scale * dis), i / 4) <= 247)
                {
                    reproject.at<uint8_t>((int)(scale * dis), i / 4) += 8;
                }
            }
        }
    }
    cv::cvtColor(reproject, reproject, cv::COLOR_GRAY2RGB);
    if (af_dis > 0 && af_dis < 8000)
    {
        int position = af_dis * scale;
        cv::Point2i start_point(0, position);
        cv::Point2i end_point(param.RS_width / 4, position);
        cv::line(reproject, start_point, end_point, cv::Scalar(0, 200, 0), 2);
    }
    if (mf_dis > 0 && mf_dis < 8000)
    {
        int position = mf_dis * scale;
        cv::Point2i start_point(0, position);
        cv::Point2i end_point(param.RS_width / 4, position);
        cv::line(reproject, start_point, end_point, cv::Scalar(0, 0, 200), 2);
    }

    cv::flip(reproject, reproject, 0);
    reprojected_depth = reproject;
    // cv::imshow("reproject", reprojected_depth);
}

void FocusController::colorDepthMix(cv::Mat &reprojected_depth, cv::Mat &color)
{
    cv::Mat resized_depth = reprojected_depth;
    cv::Mat resized_color = color;
    cv::namedWindow("mix_output", 0);
    cv::setWindowProperty("mix_output", cv::WND_PROP_FULLSCREEN, cv::WINDOW_FULLSCREEN);
    int delta_cols = resized_color.cols - resized_depth.cols;
    // for (int i = delta_cols; i < resized_color.cols; i++)
    // {
    //     for (int j = 0; j < resized_color.rows; j++)
    //     {
    //         resized_color.at<cv::Vec3b>(j, i) = resized_depth.at<cv::Vec3b>(j, i - delta_cols) * 1;
    //     }
    // }

    for (int i = 0; i < resized_color.rows; i++)
    {
        cv::Vec3b *data1 = resized_color.ptr<cv::Vec3b>(i);
        cv::Vec3b *data2 = resized_depth.ptr<cv::Vec3b>(i);
        for (int j = delta_cols; j < resized_color.cols; j++)
        {
            data1[j] = data2[j - delta_cols];
        }
    }

    cv::resize(resized_color, resized_color, cv::Size(1920, 1080));
    // cv::addWeighted(resized_depth_large, 0.3, resized_color, 0.7, 0, mix_img); // 0.5+0.5=1,0.3+0.7=1
    cv::imshow("mix_output", resized_color);
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
    int real_fps = 0;
    int fps_counter = 0;
    float runtime_counter = 0;
    __reader = make_shared<RsReader>();
    __decider = make_shared<decider>();
    // __face = make_shared<Face>();
    __face = make_shared<FaceLight>();
    __object = make_shared<ObjectLight>();

    // ROI计算
    float zoom_rate;
    cv::Rect2i ROI;
    if ((float)param.LENS_LENGTH > 24.f)
    {
        zoom_rate = (float)param.LENS_LENGTH / 24.f;
        int ROI_height = (float)param.RS_height / zoom_rate;
        int ROI_width = (float)param.RS_width / zoom_rate;
        int ROI_tl_x = (param.RS_width - ROI_width) / 2;
        int ROI_tl_y = (param.RS_height - ROI_height) / 2;
        cv::Rect2i ROI_cal(ROI_tl_x, ROI_tl_y, ROI_width, ROI_height);
        ROI = ROI_cal;
    }

    // 相机初始化
    __reader->camInit();

    // 在指定时间后再进入循环，避免相机初始化未完成导致程序崩溃
    cv::waitKey(2000);

    while (key != 27)
    {
        cout << "********** INIT **********" << endl;
        int64 t1 = cv::getTickCount();
        __reader->read();

        int DIS = 0;
        int detect_flag = 0;
        int preserve = 0;
        bool detected = 0;

        cv::Mat d16, dColor;
        cv::Mat color = __reader->color;
        cv::Mat depth = __reader->depth;
        // cv::Mat color_copy = color;
        cv::Mat color_copy;
        color.copyTo(color_copy);

        float run_time0 = 1000 * ((cv::getTickCount() - t1) / cv::getTickFrequency());
        cout << "run time before draw-box = " << run_time0 << " ms" << endl;

        // 绘制目标框
        cout << "********** DRAW-BOX **********" << endl;
        if (!MF_trigger)
        {
            cout << "vf" << __face->isValideFace() << endl;
            cout << "vo" << __object->isValideObject() << endl;
            if (__object->isValideObject())
            {
                __object->drawBox(color, depth);
                cout << "obj" << endl;
            }
            float run_time0_1 = 1000 * ((cv::getTickCount() - t1) / cv::getTickFrequency());
            cout << "run time draw-box 1 = " << run_time0_1 << " ms" << endl;
            string AF = "AF-mode";
            cv::putText(color, AF, cv::Point2i(15, 90), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 0, 0), 2);
        }
        else
        {
            string MF = "MF-mode";
            cv::putText(color, MF, cv::Point2i(15, 90), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 0, 0), 2);
        }
        cv::rectangle(color, ROI, cv::Scalar(180, 180, 180), 3);

        float run_time0_2 = 1000 * ((cv::getTickCount() - t1) / cv::getTickFrequency());
        cout << "run time draw-box 2 = " << run_time0_2 << " ms" << endl;

        cout << "********** DETECT **********" << endl;
        // if (!MF_trigger)
        // {
        // 检测器，判断该帧是否需要进行目标检测
        if (__logic->timeTrigger(t0, 10))
        {
            int64 t2 = cv::getTickCount();
            // detected = __face->detect(color_copy, depth);
            // __detector = __face;
            detected = __object->detect(color_copy, depth);
            __detector = __object;
            // detect_flag = 1;
            detect_flag = 1;
            face_trigger = 1;
            float run_time1 = 1000 * ((cv::getTickCount() - t2) / cv::getTickFrequency());
            cout << "run time obj-detect = " << run_time1 << " ms" << endl;
        }
        else
        {
            // 目标检测后搭配一次面部检测
            if (face_trigger)
            {
                detected = __face->detect(color_copy, depth);
                // detected = 0;
                __detector = __face;
                face_trigger = 0;
                detect_flag = 0;
            }
            else
            {
                detect_flag = 2;
            }
        }
        // }

        cout << "********** READ **********" << endl;
        int result = __motor->read();
        cout << "read-result:" << result << endl;
        cv::putText(color, cv::format("%d", result), cv::Point2i(15, 120), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 0, 0), 2);
        if (result == -3 && !MF_trigger)
        {
            // rec_on:触发手动模式
            MF_trigger = 1;
        }
        else if (result == -4)
        {
            // rec_off:关闭手动模式
            MF_trigger = 0;
        }
        // if (result == -3 || result == -4)
        // {
        //     // rec按键按下:切换手动和自动
        //     if (MF_trigger == 1)
        //     {
        //         MF_trigger = 0;
        //     }
        //     else
        //     {
        //         MF_trigger = 1;
        //     }
        // }
        // 暂停运行(低功耗)
        if (result == -2)
        {
            while (1)
            {
                cv::waitKey(2000);
                int result2 = __motor->read();
                if (result2 == -3 || result2 == -4)
                {
                    break;
                }
            }
        }

        cout << "********** DECIDE **********" << endl;
        int64 t3 = cv::getTickCount();
        // 简易追踪器 & 掉帧/对焦策略处理器 & 距离解算器
        DIS = __decider->decide(d16, color, __reader, __face, __object, __dis, __logic, detected, detect_flag, result);

        float run_time2 = 1000 * ((cv::getTickCount() - t3) / cv::getTickFrequency());
        cout << "run time decide = " << run_time2 << " ms" << endl;
        // 读取当前脉冲值
        // int current_pulse = __motor->read();
        int current_pulse = 0;

        // 计算目标脉冲值-方案一:使用四次函数拟合，五次函数不稳定，A暂时废弃
        // int target_pulse = (lens_param.B * pow(DIS, 4) + lens_param.C * pow(DIS, 3) + lens_param.D * pow(DIS, 2) + lens_param.E * DIS + lens_param.F);

        // 计算目标脉冲值-方案二:使用插值法
        int target_pulse = __decider->disInterPolater(DIS);
        last_target_pulse = target_pulse;
        cv::putText(color, cv::format("%d", last_target_pulse), cv::Point2i(15, 60), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 0, 0), 2);

        // 计算差值，写入串口，同时进行异常处理，驱动镜头
        // __motor->write((target_pulse - current_pulse));
        cout << "********** WRITE **********" << endl;
        int64 t4 = cv::getTickCount();
        if (MF_trigger)
        {
            if (result >= 0 && result <= 9999)
            {
                if (result > (MF_init_result + 10) || result < (MF_init_result - 10))
                {
                    __motor->write(result, 0);
                }
            }
            // 不能直接套interPolater，是反函数
            // depthReProjection(depth, __decider->pulseInterPolater(result), 0);
        }
        else
        {
            if (result >= 0 && result <= 9999)
            {
                MF_init_result = result;
            }
            __motor->write(last_target_pulse, 0);
        }
        cv::putText(color, cv::format("%d", real_fps), cv::Point2i(15, 400), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 0, 0), 2);
        depthReProjection(depth, DIS, __decider->pulseInterPolater(result));

        // 输出彩色图和深度图
        // imshow("Depth", depth * 10);
        // imshow("Color", color);
        colorDepthMix(reprojected_depth, color);
        float run_time3 = 1000 * ((cv::getTickCount() - t4) / cv::getTickFrequency());
        cout << "run time reproject = " << run_time3 << " ms" << endl;
        // imshow("color-copy", color_copy);

        // 计算运行时间
        int key = cv::waitKey(1);
        float run_time = 1000 * ((cv::getTickCount() - t1) / cv::getTickFrequency());
        fps_counter++;
        runtime_counter += run_time;
        if (runtime_counter >= 1000.)
        {
            real_fps = fps_counter;
            fps_counter = 0;
            runtime_counter = 0;
        }
        cout << "real_fps" << real_fps << endl;
        cout << "run time = " << run_time << " ms" << endl;
    }
}
