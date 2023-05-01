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
    cout << "0000" << endl;
    cout << depth.rows << endl;
    cout << depth.cols << endl;
    cv::Mat reproject(depth.rows, depth.cols, CV_8UC1, cv::Scalar(0)); // 单通道,大小同depth
    float scale = (float)depth.rows / (float)(8 * 1000);               // 最远显示为8m
    cout << "scale" << scale << endl;
    for (int i = 0; i < reproject.cols; i++)
    {
        for (int j = 0; j < reproject.rows; j++)
        {
            int dis = depth.at<uint16_t>(j, i);
            // cout << "2222" << endl;
            if (dis < 8000)
            {
                if (reproject.at<uint8_t>((int)(scale * dis), i) <= 255)
                {
                    // cout << "3333" << endl;
                    // cout << (int)(scale * dis) << endl;
                    // cout << "?!?" << (int)reproject.at<uint8_t>((int)(scale * dis), i) << endl;
                    reproject.at<uint8_t>((int)(scale * dis), i) += 4;
                }
            }
        }
    }
    cout << "TYPE1" << reproject.type() << endl;
    cv::cvtColor(reproject, reproject, cv::COLOR_GRAY2RGB);
    cout << "TYPE2" << reproject.type() << endl;
    if (af_dis > 0 && af_dis < 8000)
    {
        int position = af_dis * scale;
        cv::Point2i start_point(0, position);
        cv::Point2i end_point(param.RS_width, position);
        cv::line(reproject, start_point, end_point, cv::Scalar(0, 0, 255), 3);
    }

    cout << "4444" << endl;
    cv::flip(reproject, reproject, 0);
    reprojected_depth = reproject;
    cv::imshow("reproject", reprojected_depth);
}

void FocusController::colorDepthMix(cv::Mat &reprojected_depth, cv::Mat &color)
{
    cv::Mat resized_depth;
    cv::Mat resized_depth_large(1080, 1920, CV_8UC3, cv::Scalar(0, 0, 0));
    cv::Mat resized_color;
    cv::Mat mix_img;
    cv::namedWindow("mix_output", 0);
    cv::setWindowProperty("mix_output", cv::WND_PROP_FULLSCREEN, cv::WINDOW_FULLSCREEN);
    cv::resize(reprojected_depth, resized_depth, cv::Size(480, 1080));
    cv::resize(color, resized_color, cv::Size(1920, 1080));
    int delta_cols = resized_color.cols - resized_depth.cols;
    for (int i = delta_cols; i < resized_color.cols; i++)
    {
        for (int j = 0; j < resized_color.rows; j++)
        {
            for (int k = 0; k < 3; k++)
            {
                resized_color.at<cv::Vec3b>(j, i)[k] = resized_color.at<cv::Vec3b>(j, i)[k] * 0.5 +
                                                       resized_depth.at<cv::Vec3b>(j, i - delta_cols)[k] * 0.5;
            }
        }
    }
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
    __reader = make_shared<RsReader>();
    __decider = make_shared<decider>();
    __face = make_shared<Face>();
    __object = make_shared<ObjectLight>();

    // 相机初始化
    __reader->camInit();

    // 在指定时间后再进入循环，避免相机初始化未完成导致程序崩溃
    cv::waitKey(2000);

    while (key != 27)
    {
        int64 t1 = cv::getTickCount();
        __reader->read();

        int DIS = 0;
        int detect_flag = 0;
        int preserve = 0;
        bool detected = 0;

        cv::Mat d16, dColor;
        cv::Mat color = __reader->color;
        // cv::Mat color_copy = color;
        cv::Mat color_copy;
        color.copyTo(color_copy);
        cv::Mat depth = __reader->depth;

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
            string AF = "AF-mode";
            cv::putText(color, AF, cv::Point2i(15, 90), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 0, 0), 2);
        }
        else
        {
            string MF = "MF-mode";
            cv::putText(color, MF, cv::Point2i(15, 90), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 0, 0), 2);
        }
        float zoom_rate;
        if ((float)param.LENS_LENGTH > 24.f)
        {
            zoom_rate = (float)param.LENS_LENGTH / 24.f;
            int ROI_height = (float)param.RS_height / zoom_rate;
            int ROI_width = (float)param.RS_width / zoom_rate;
            int ROI_tl_x = (param.RS_width - ROI_width) / 2;
            int ROI_tl_y = (param.RS_height - ROI_height) / 2;
            cv::Rect2i ROI(ROI_tl_x, ROI_tl_y, ROI_width, ROI_height);
            cv::rectangle(color, ROI, cv::Scalar(0, 255, 0), 3);
        }

        cout << "********** DETECT **********" << endl;
        // if (!MF_trigger)
        // {
        // 检测器，判断该帧是否需要进行目标检测
        if (__logic->timeTrigger(t0, 10))
        {
            // detected = __face->detect(color_copy, depth);
            // __detector = __face;
            detected = __object->detect(color_copy, depth);
            __detector = __object;
            // detect_flag = 1;
            detect_flag = 1;
            face_trigger = 1;
        }
        else
        {
            // 目标检测后搭配一次面部检测
            if (face_trigger)
            {
                detected = __face->detect(color_copy, depth);
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

        cout << "********** DECIDE **********" << endl;
        // 简易追踪器 & 掉帧/对焦策略处理器 & 距离解算器
        DIS = __decider->decide(d16, color, __reader, __face, __object, __dis, __logic, detected, detect_flag, result);

        // 读取当前脉冲值
        // int current_pulse = __motor->read();
        int current_pulse = 0;

        // 计算目标脉冲值-方案一:使用四次函数拟合，五次函数不稳定，A暂时废弃
        // int target_pulse = (lens_param.B * pow(DIS, 4) + lens_param.C * pow(DIS, 3) + lens_param.D * pow(DIS, 2) + lens_param.E * DIS + lens_param.F);

        // 计算目标脉冲值-方案二:使用插值法
        int target_pulse = __decider->disInterPolater(DIS);
        last_target_pulse = target_pulse;
        cv::putText(color, cv::format("%d", last_target_pulse), cv::Point2i(15, 60), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 0, 0), 2);
        depthReProjection(depth, DIS, 0);
        // 计算差值，写入串口，同时进行异常处理，驱动镜头
        // __motor->write((target_pulse - current_pulse));
        cout << "********** WRITE **********" << endl;
        if (MF_trigger)
        {
            if (result >= 0 && result <= 9999)
            {
                if (result > (MF_init_result + 10) || result < (MF_init_result - 10))
                {
                    __motor->write(result, 0);
                }
            }
        }
        else
        {
            if (result >= 0 && result <= 9999)
            {
                MF_init_result = result;
            }
            __motor->write(last_target_pulse, 0);
        }

        // 输出彩色图和深度图
        imshow("Depth", depth * 10);
        imshow("Color", color);
        colorDepthMix(reprojected_depth, color);
        // imshow("color-copy", color_copy);

        // 计算运行时间
        int key = cv::waitKey(1);
        float run_time = 1000 * ((cv::getTickCount() - t1) / cv::getTickFrequency());
        cout << "run time = " << run_time << " ms" << endl;
    }
}
