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
    __filter = make_shared<Dis>();
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
            // data1[j] = 0.2 * data1[j] + 0.8 * data2[j - delta_cols];
        }
    }

    cv::resize(resized_color, resized_color, cv::Size(1920, 1080));
    // cv::addWeighted(resized_depth_large, 0.3, resized_color, 0.7, 0, mix_img); // 0.5+0.5=1,0.3+0.7=1
    cv::imshow("mix_output", resized_color);
}

/**
 * @brief 拉格朗日插值计算
 *
 * @param dis
 * @param n
 * @return float
 */
float FocusController::lagrange(float dis, int n, int flag)
{
    float yResult = 0.0;
    if (dis > 500)
    {
        // 参数载入
        float arrX[8] = {
            500.f,
            1500.f,
            2500.f,
            4000.f,
            6000.f,
            8000.f,
            25000.f};
        float arrY[8] = {
            float(lens_param.A),
            float(lens_param.B),
            float(lens_param.C),
            float(lens_param.D),
            float(lens_param.E),
            float(lens_param.F),
            float(lens_param.G)};

        if (flag != 0)
        {
            // 反向求解
            float temp[n];
            for (int i = 0; i < 7; i++)
            {
                temp[i] = arrX[i];
                arrX[i] = arrY[i];
                arrY[i] = temp[i];
            }
        }
        const int N = 10;
        // LValue[N]存放的是每次求解的插值基函数的通项
        float LValue[N];
        // 循环变量k,m
        int k, m;
        // 插值基函数中的上下累乘temp1,temp2
        float temp1, temp2;

        for (k = 0; k < n; k++)
        {
            temp1 = 1.0; // 分子和分母不能为0，所以初始化为1.0
            temp2 = 1.0;
            for (m = 0; m < n; m++)
            {
                if (m == k)
                {
                    continue;
                }
                temp1 *= (dis - arrX[m]);     // 插值公式的分子部分。(x-x1)(x-x2)
                temp2 *= (arrX[k] - arrX[m]); // 插值公式的分母部分(x0-x1)(x1-x2)
            }

            LValue[k] = temp1 / temp2; // 求出的一个分式
        }

        for (int i = 0; i < n; i++)
        {
            yResult += arrY[i] * LValue[i]; // 求出和
        }
    }
    else
    {
        int closest_pulse;
        if ((lens_param.B - lens_param.A) < 0)
        {
            closest_pulse = 9990;
        }
        else
        {
            closest_pulse = 0;
        }
        if (dis > lens_param.INIT_DIS)
        {
            int target_pulse = closest_pulse + (lens_param.A - closest_pulse) * (dis - lens_param.INIT_DIS) / (500 - lens_param.INIT_DIS);
            return target_pulse;
        }
        else
        {
            // return closest_pulse;
            return lens_param.A;
        }
    }

    return yResult;
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
    // __face = make_shared<Body>();
    __object = make_shared<ObjectLight>();

    // 总体识别区ROI计算
    float zoom_rate;
    cv::Rect2i ROI;
    if ((float)param.LENS_LENGTH > 26.f)
    {
        zoom_rate = (float)param.LENS_LENGTH / 26.f;
        int ROI_height = (float)param.RS_height / zoom_rate;
        int ROI_width = (float)param.RS_width / zoom_rate;
        int ROI_tl_x = (param.RS_width - ROI_width) / 2;
        int ROI_tl_y = (param.RS_height - ROI_height) / 2;
        cv::Rect2i ROI_cal(ROI_tl_x + param.width_compensate, ROI_tl_y + param.height_compensate, ROI_width, ROI_height);
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
        int final_write_position = 0;
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
            string AF = "AF";
            cv::putText(color, AF, cv::Point2i(15, 90), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 0, 0), 2);
        }
        else
        {
            string MF = "MF";
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
            cout << "********** OBJECT-DETECT **********" << endl;
            int64 t2 = cv::getTickCount();
            // __face->detect(color_copy, depth);
            detected = __object->detect(color_copy, depth);
            // detected = 0;
            __detector = __object;
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
                cout << "********** FACE-DETECT **********" << endl;
                int64 t2 = cv::getTickCount();
                detected = __face->detect(color_copy, depth);
                // detected = 0;
                __detector = __face;
                face_trigger = 0;
                detect_flag = 0;
                float run_time1 = 1000 * ((cv::getTickCount() - t2) / cv::getTickFrequency());
                cout << "run time face-detect = " << run_time1 << " ms" << endl;
            }
            else
            {
                detect_flag = 2;
            }
        }
        // }

        cout << "********** READ **********" << endl;
        // cv::waitKey(100);
        __motor->read();
        int command = __motor->readCommand();
        int position = __motor->readPosition();
        cout << "read-result:" << position << endl;
        string current_position = "pos:" + cv::format("%d", position);
        string current_command = "com:" + cv::format("%d", command);
        cv::putText(color, current_position, cv::Point2i(15, 120), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 0, 0), 2);
        cv::putText(color, current_command, cv::Point2i(15, 140), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 0, 0), 2);

        if (command == -3 || command == -4)
        {
            if (!MF_trigger)
            {
                MF_trigger = 1;
            }
            else
            {
                MF_trigger = 0;
            }
        }

        // 自动模式下，CAL控制对焦模式
        if (command == -1)
        {
            if (forced_drop_trigger)
            {
                forced_drop_trigger = 0;
            }
            else
            {
                forced_drop_trigger = 1;
            }
        }

        // 长按，校准并暂停运行(低功耗)
        if (command == -2)
        {
            __motor->write(-1, 0); // 行程校准
            while (1)
            {
                cv::waitKey(2000);
                __motor->read();
                int result2 = __motor->readCommand();
                if (result2 == -3 || result2 == -4)
                {
                    break;
                }
            }
        }

        cout << "********** DECIDE **********" << endl;
        int64 t3 = cv::getTickCount();
        // 简易追踪器 & 掉帧/对焦策略处理器 & 距离解算器

        if (!forced_drop_trigger && !MF_trigger)
        {
            // 中心区域对焦(触发强制掉帧),situation = 0;
            cv::putText(color, "center mode", cv::Point2i(60, 90), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 0, 0), 2);
            DIS = __decider->decide(depth, color, __reader, __face, __object, __dis, __logic, 0, 3, position);
        }
        else
        {
            // 智能识别对焦,situation = detected;
            if (!MF_trigger)
            {
                cv::putText(color, "AI mode", cv::Point2i(60, 90), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 0, 0), 2);
            }
            DIS = __decider->decide(depth, color, __reader, __face, __object, __dis, __logic, detected, detect_flag, position);
        }
        cv::putText(color, cv::format("%d", DIS), cv::Point2i(260, 90), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 0, 0), 2);

        float run_time2 = 1000 * ((cv::getTickCount() - t3) / cv::getTickFrequency());
        cout << "run time decide = " << run_time2 << " ms" << endl;
        // 读取当前脉冲值
        // int current_pulse = __motor->read();
        int current_pulse = 0;

        // 计算目标脉冲值-方案一:使用四次函数拟合，五次函数不稳定，A暂时废弃
        // int target_pulse = (lens_param.B * pow(DIS, 4) + lens_param.C * pow(DIS, 3) + lens_param.D * pow(DIS, 2) + lens_param.E * DIS + lens_param.F);

        // 计算目标脉冲值-方案二:使用线性插值
        // int target_pulse = __decider->disInterPolater(DIS);
        if (DIS < 500)
        {
            DIS = 500;
        }
        // 计算目标脉冲值-方案三:拉格朗日插值
        int target_pulse = this->lagrange(DIS, 7, 0);
        last_target_pulse = target_pulse;
        string target_position = "target-pos:" + cv::format("%d", last_target_pulse);
        cv::putText(color, target_position, cv::Point2i(15, 60), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 0, 0), 2);

        // 计算差值，写入串口，同时进行异常处理，驱动镜头
        // __motor->write((target_pulse - current_pulse));
        cout << "********** WRITE **********" << endl;
        int64 t4 = cv::getTickCount();
        if (MF_trigger && forced_drop_trigger)
        {
            cv::putText(color, "0-10m reprojection", cv::Point2i(60, 90), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 0, 0), 2);
            if (position >= 0 && position <= 9999)
            {
                if (position > (MF_init_result + 10) || position < (MF_init_result - 10))
                {
                    if (position > 500)
                    {
                        // __motor->write(__decider->disInterPolater(position), 0);
                        // __motor->write(this->lagrange(position, 7, 0), 0);
                        final_write_position = this->lagrange(position, 7, 0);
                    }
                    else
                    {
                        // test stratage
                        // int position = 501;
                        // int currected_position = 500 - ((500 - position) / 10);
                        // __motor->write(__decider->disInterPolater(position) - (500 - position), 0);
                        // __motor->write(this->lagrange(lens_param.A, 7, 0), 0);
                        final_write_position = lens_param.A;
                    }
                }
            }
            depthReProjection(depth, DIS, position);
        }
        else if (MF_trigger && !forced_drop_trigger)
        {
            // 该模式还有待测试
            cv::putText(color, "full scale projection", cv::Point2i(60, 90), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 0, 0), 2);
            if (position >= 0 && position <= 9999)
            {
                if (position > (MF_init_result + 10) || position < (MF_init_result - 10))
                {
                    if (position > 500)
                    {
                        // __motor->write(position, 0);
                        final_write_position = position;
                    }
                    else
                    {
                        final_write_position = 500;
                    }
                }
            }
            cv::putText(color, "test" + cv::format("%d", __decider->pulseInterPolater(position)), cv::Point2i(120, 250), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 0, 0), 2);
            depthReProjection(depth, DIS, __decider->pulseInterPolater(position));
        }
        else
        {
            if (position >= 0 && position <= 9999)
            {
                MF_init_result = position;
            }
            depthReProjection(depth, DIS, 9999 - position);
            // __motor->write(last_target_pulse, 0);
            final_write_position = last_target_pulse;
        }
        // 最终补丁
        // final_write_position = __filter->kalmanFilter(final_write_position);
        cv::putText(color, cv::format("%d", final_write_position), cv::Point2i(15, 330), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 255, 0), 2);
        if (final_write_position < lens_param.A && final_write_position > lens_param.G)
        {
            __motor->write(final_write_position, 0);
        }
        else if (final_write_position < lens_param.G && final_write_position > lens_param.A)
        {
            __motor->write(final_write_position, 0);
        }
        else
        {
            int dis1 = abs(final_write_position - lens_param.A);
            int dis2 = abs(final_write_position - lens_param.G);
            int position = (dis1 < dis2) ? lens_param.A : lens_param.G;
            __motor->write(position, 0);
        }

        // 打印FPS
        cv::putText(color, cv::format("%d", real_fps), cv::Point2i(15, 400), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 0, 0), 2);

        // 输出彩色图和深度图
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
