/**
 * @file frame_processor.cpp
 * @author 赖建宇 (lai.jianyu@foxmail.com)
 * @brief
 * @version 1.0
 * @date 2022-12-01
 *
 * @copyright Copyright SCUT RobotLab(c) 2021
 *
 */

#include "processor.h"
#include "lens_param.h"
#include "param.h"
#include <iostream>
#include <fstream>
#include <string>

void Frame::astraProcessFrame(Face &face, Dis &dis, int64 &t0, Data &data)
{
    cv::VideoCapture depthStream(cv::CAP_OPENNI2_ASTRA);
    cv::VideoCapture colorStream(4, cv::CAP_V4L2);

    colorStream.set(cv::CAP_PROP_FRAME_WIDTH, param.ASTRA_width);
    colorStream.set(cv::CAP_PROP_FRAME_HEIGHT, param.ASTRA_height);
    depthStream.set(cv::CAP_PROP_FRAME_WIDTH, param.ASTRA_width);
    depthStream.set(cv::CAP_PROP_FRAME_HEIGHT, param.ASTRA_height);
    depthStream.set(cv::CAP_PROP_OPENNI2_MIRROR, 0);
    cout << "Depth stream: "
         << depthStream.get(cv::CAP_PROP_FRAME_WIDTH) << "x" << depthStream.get(cv::CAP_PROP_FRAME_HEIGHT)
         << " @" << depthStream.get(cv::CAP_PROP_FPS) << " fps" << endl;

    const std::size_t maxFrames = 32;
    // Synchronization objects
    std::mutex mtx;
    std::condition_variable dataReady;
    std::atomic<bool> isFinish;
    TransferData readData, writeData;
    Motor motor;
    int round = 0;

    isFinish = false;

    // Start depth reading thread
    std::thread depthReader([&]
                            {
         while (!isFinish)
         {
             // Grab and decode new frame
             if (depthStream.grab())
             {
                 AstraFrame f;
                 f.timestamp = cv::getTickCount();
                 depthStream.retrieve(f.frame, cv::CAP_OPENNI_DEPTH_MAP);
                 if (f.frame.empty())
                 {
                     cerr << "ERROR: Failed to decode frame from depth stream" << endl;
                     break;
                 }

                 {
                     std::lock_guard<std::mutex> lk(mtx);
                     if (astraDepthFrames.size() >= maxFrames)
                     {
                        astraDepthFrames.pop_front();
                     }
                     if(param.INVERT_ON)
                     {
                        cv::flip(f.frame,f.frame,0);
                     }
                     astraDepthFrames.push_back(f);
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
                 AstraFrame f;
                 f.timestamp = cv::getTickCount();
                 colorStream.retrieve(f.frame);
                 if (f.frame.empty())
                 {
                     cerr << "ERROR: Failed to decode frame from color stream" << endl;
                     break;
                 }
                 
                 {
                     std::lock_guard<std::mutex> lk(mtx);
                     if (astraColorFrames.size() >= maxFrames)
                     {
                         astraColorFrames.pop_front();
                     }
                     if(param.INVERT_ON)
                     {
                        cv::flip(f.frame,f.frame,0);
                     }
                     astraColorFrames.push_back(f);
                 }
                 dataReady.notify_one();
             }
         } });
    while (!isFinish)
    {
        std::unique_lock<std::mutex> lk(mtx);
        while (!isFinish && (astraDepthFrames.empty() || astraColorFrames.empty()))
        {
            dataReady.wait(lk);
        }
        while (!astraDepthFrames.empty() && !astraColorFrames.empty())
        {
            if (!lk.owns_lock())
                lk.lock();

            int64 t1 = cv::getTickCount();
            // Get a frame from the list
            AstraFrame depthFrame = astraDepthFrames.front();
            int64 depthT = depthFrame.timestamp;

            // Get a frame from the list
            AstraFrame colorFrame = astraColorFrames.front();
            int64 colorT = colorFrame.timestamp;

            // Half of frame period is a maximum time diff between frames
            const int64 maxTdiff = int64(1000000000 / (2 * colorStream.get(cv::CAP_PROP_FPS)));
            if (depthT + maxTdiff < colorT)
            {
                astraDepthFrames.pop_front();
                continue;
            }
            else if (colorT + maxTdiff < depthT)
            {
                astraColorFrames.pop_front();
                continue;
            }
            astraDepthFrames.pop_front();
            astraColorFrames.pop_front();
            lk.unlock();

            cv::Mat d8, d16, dColor;
            int DIS = 0;
            // this->depth_frames = depthFrames;
            // this->color_frames = colorFrames;
            if (drop_init)
            {
                drop_count = 0;
            }

            // convert to dcolor frame(d8) and high resolution depth frame(d16)
            depthFrame.frame.convertTo(d8, CV_8U, 255.0 / 8000);
            applyColorMap(d8, dColor, cv::COLORMAP_RAINBOW);
            depthFrame.frame.convertTo(d16, CV_16U, 65535.0 / 8000); // the effective measure distance is 8000mm

            // FPS control
            int fps = param.FPS; // support 1,2,3,5,6,10,15,30
            int detect_rate = 30 / fps;
            if (detect_init)
            {
                detect_count = detect_rate - 1;
            }
            // cout << this->detect_count << endl;

            DIS = Decider(face, dis, colorFrame.frame, d16, detect_count);
            // 计算差值，写入串口，同时进行异常处理
            int current_pulse = motor.readPulse(data);
            // 方案二:使用插值法
            int target_pulse = disInterPolater(DIS);
            cv::putText(colorFrame.frame, cv::format("%d", target_pulse), cv::Point2i(15, 60), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 0, 0), 2);

            motor.writePulse((target_pulse - current_pulse), data);
            // int target_pulse = (lens_param.B * pow(DIS, 4) + lens_param.C * pow(DIS, 3) + lens_param.D * pow(DIS, 2) + lens_param.E * DIS + lens_param.F);
            // // cout << "current_pulse = " << current_pulse << endl;
            // // cout << "target_pulse = " << target_pulse << endl;
            // if (abs(target_pulse - current_pulse) < abs(lens_param.INFINIT_PULSE - lens_param.INIT_PULSE))
            // {
            //     int min_pulse = (lens_param.INFINIT_PULSE < lens_param.INIT_PULSE ? lens_param.INFINIT_PULSE : lens_param.INIT_PULSE);
            //     int max_pulse = (lens_param.INFINIT_PULSE > lens_param.INIT_PULSE ? lens_param.INFINIT_PULSE : lens_param.INIT_PULSE);
            //     if (target_pulse < max_pulse && target_pulse > min_pulse)
            //     {
            //         if (current_pulse <= max_pulse + 50 && current_pulse >= min_pulse - 50)
            //         {
            //             motor.writePulse((target_pulse - current_pulse), data);
            //             cout << "写入中" << endl;
            //         }
            //         else
            //         {
            //             cout << "ERROR!-当前值异常" << endl;
            //         }
            //     }
            //     else
            //     {
            //         cout << "ERROR!-目标值异常" << endl;
            //     }
            // }
            // else
            // {
            //     cout << "ERROR!-差值过大" << endl;
            // }

            // show dcolor frame
            imshow("Depth (colored)", dColor);

            // Show color frame
            imshow("Color", colorFrame.frame);

            // Exit on Esc key press
            int key = cv::waitKey(1);
            if (key == 27) // ESC
            {
                isFinish = true;
                // break;
            }
            cout << "run time = " << 1000 * ((cv::getTickCount() - t1) / cv::getTickFrequency()) << " ms" << endl;
        }
    }
}

/**
 * @brief Realsense相机 帧处理函数
 *
 * @param face
 * @param dis
 * @param t0
 * @param data
 */
void Frame::rsProcessFrame(Face &face, Dis &dis, int64 &t0, Data &data)
{
    int key;
    TransferData readData, writeData;
    Motor motor;
    int round = 0;
    // judge whether devices is exist or not
    rs2::context ctx;
    auto list = ctx.query_devices(); // Get a snapshot of currently connected devices
    if (list.size() == 0)
        throw std::runtime_error("No device detected. Is it plugged in?");
    rs2::device dev = list.front();

    rs2::frameset frames;
    // Contruct a pipeline which abstracts the device
    rs2::pipeline pipe;
    // Create a configuration for configuring the pipeline with a non default profile
    rs2::config cfg; // 创建一个以非默认配置的配置用来配置管道
    rs2::sensor sen;
    // Add desired streams to configuration
    cfg.enable_stream(RS2_STREAM_COLOR, param.RS_width, param.RS_height, RS2_FORMAT_BGR8, param.RS_fps); // 向配置添加所需的流
    cfg.enable_stream(RS2_STREAM_DEPTH, param.RS_width, param.RS_height, RS2_FORMAT_Z16, param.RS_fps);

    std::ifstream file("../test2.json");
    if (file.good())
    {
        std::string str((std::istreambuf_iterator<char>(file)), std::istreambuf_iterator<char>());

        auto prof = cfg.resolve(pipe);
        if (auto advanced = prof.get_device().as<rs400::advanced_mode>())
        {
            advanced.load_json(str);
        }
    }
    rs2::decimation_filter dec_filter; // 抽取滤波器
    rs2::spatial_filter spat_filter;   // 空间滤波器
    rs2::threshold_filter thres_filter;
    rs2::temporal_filter temp_filter; // 时间滤波器
    rs2::hole_filling_filter hf_filter;

    spat_filter.set_option(RS2_OPTION_FILTER_MAGNITUDE, 2);
    temp_filter.set_option(RS2_OPTION_FILTER_SMOOTH_ALPHA, 0.4);
    temp_filter.set_option(RS2_OPTION_FILTER_SMOOTH_DELTA, 20);
    temp_filter.set_option(RS2_OPTION_HOLES_FILL, 6);
    hf_filter.set_option(RS2_OPTION_HOLES_FILL, 1);

    // get depth scale
    // float depth_scale = get_depth_scale(profile.get_device());
    // start stream
    pipe.start(cfg); // 指示管道使用所请求的配置启动流
    sen = pipe.get_active_profile().get_device().query_sensors()[1];
    // sen.set_option(RS2_OPTION_AUTO_EXPOSURE_PRIORITY, true);

    while (key != 27)
    {
        int64 t1 = cv::getTickCount();
        rs_read(pipe, frames);

        // Creating OpenCV Matrix from a color image
        rs2::frame color_frame = rsColorFrames.back();
        rs2::frame depth_frame = rsDepthFrames.back();
        // depth_frame = spat_filter.process(depth_frame);
        depth_frame = temp_filter.process(depth_frame);
        // depth_frame = hf_filter.process(depth_frame);

        double current_color_timestamp = color_frame.get_timestamp();
        double current_depth_timestamp = depth_frame.get_timestamp();
        if (current_color_timestamp == last_color_timestamp)
        {
            continue;
        }
        if (current_depth_timestamp == last_depth_timestamp)
        {
            continue;
        }
        last_color_timestamp = current_color_timestamp;
        last_depth_timestamp = current_depth_timestamp;

        cv::Mat color(cv::Size(param.RS_width, param.RS_height), CV_8UC3, (void *)color_frame.get_data(), cv::Mat::AUTO_STEP);
        cv::Mat depth(cv::Size(param.RS_width, param.RS_height), CV_16U, (void *)depth_frame.get_data(), cv::Mat::AUTO_STEP);

        if (param.INVERT_ON)
        {
            cv::waitKey(1);
            cv::flip(color, color, 0);
            cv::flip(depth, depth, 0);
        }
        cv::Mat d8, d16, dColor;
        int DIS = 0;
        // this->depth_frames = depthFrames;
        // this->color_frames = colorFrames;
        if (drop_init)
        {
            drop_count = 0;
        }

        // FPS control
        int fps = param.FPS; // support 1,2,3,5,6,10,15,30
        int detect_rate = 30 / fps;
        if (detect_init)
        {
            detect_count = detect_rate - 1;
        }
        // cout << this->detect_count << endl;

        DIS = Decider(face, dis, color, d16, detect_count);

        // 计算差值，写入串口，同时进行异常处理
        int current_pulse = motor.readPulse(data);
        // 方案一:使用四次函数拟合，五次函数不稳定，A暂时废弃
        // int target_pulse = (lens_param.B * pow(DIS, 4) + lens_param.C * pow(DIS, 3) + lens_param.D * pow(DIS, 2) + lens_param.E * DIS + lens_param.F);
        // 方案二:使用插值法
        int target_pulse = disInterPolater(DIS);
        cv::putText(color, cv::format("%d", target_pulse), cv::Point2i(15, 60), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 0, 0), 2);

        motor.writePulse((target_pulse - current_pulse), data);
        // cout << "current_pulse = " << current_pulse << endl;
        // cout << "target_pulse = " << target_pulse << endl;
        // if (abs(target_pulse - current_pulse) < abs(lens_param.INFINIT_PULSE - lens_param.INIT_PULSE))
        // {
        //     int min_pulse = (lens_param.INFINIT_PULSE < lens_param.INIT_PULSE ? lens_param.INFINIT_PULSE : lens_param.INIT_PULSE);
        //     int max_pulse = (lens_param.INFINIT_PULSE > lens_param.INIT_PULSE ? lens_param.INFINIT_PULSE : lens_param.INIT_PULSE);
        //     cv::putText(color, cv::format("%d", min_pulse), cv::Point2i(15, 100), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 0, 0), 2);
        //     cv::putText(color, cv::format("%d", max_pulse), cv::Point2i(200, 100), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 0, 0), 2);
        //     if (target_pulse < max_pulse + 50 && target_pulse > min_pulse - 50)
        //     {
        //         if (current_pulse <= max_pulse + 100 && current_pulse >= min_pulse - 100)
        //         {
        //             motor.writePulse((target_pulse - current_pulse), data);
        //             cout << "写入中" << endl;
        //         }
        //         else
        //         {
        //             cout << "ERROR!-当前值异常" << endl;
        //         }
        //     }
        //     else
        //     {
        //         cout << "ERROR!-目标值异常" << endl;
        //     }
        // }
        // else
        // {
        //     cout << "ERROR!-差值过大" << endl;
        // }

        // show dcolor frame
        imshow("Depth", depth * 15);

        // Show color frame
        imshow("Color", color);

        // Exit on Esc key press
        int key = cv::waitKey(1);
        cout << "run time = " << 1000 * ((cv::getTickCount() - t1) / cv::getTickFrequency()) << " ms" << endl;
    }
}

/**
 * @brief Realsense 相机读取函数
 *
 * @param pipe
 * @param frames
 */
void Frame::rs_read(rs2::pipeline &pipe, rs2::frameset &frames)
{
    // int64 t1 = cv::getTickCount();

    frames = pipe.wait_for_frames();
    // Get each frame
    rs2::frame color_frame = frames.get_color_frame();
    rs2::depth_frame depth_frame = frames.get_depth_frame();
    // rs2::video_frame ir_frame_left = frames.get_infrared_frame(1);
    // rs2::video_frame ir_frame_right = frames.get_infrared_frame(2);

    rsDepthFrames.push_back(depth_frame);
    rsColorFrames.push_back(color_frame);
    if (rsColorFrames.size() > 2)
    {
        rsColorFrames.pop_front();
    }
    if (rsDepthFrames.size() > 2)
    {
        rsDepthFrames.pop_front();
    }
    // cout << "run time = " << 1000 * ((cv::getTickCount() - t1) / cv::getTickFrequency()) << " ms" << endl;
}

/**
 * @brief 掉帧（未检测到人脸）处理函数
 *
 * @param mode
 * @param dis
 * @param d16
 */
void Frame::dropProcess(int mode, Dis &dis, cv::Mat &d16)
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
            int center_dis = dis.disCalculate(0, d16, points);
        }
        if (param.cam_module == REALSENSE)
        {
            int img_width = param.RS_width;
            int img_height = param.RS_height;
            int center_dis = int(1000 * rsDepthFrames.back().get_distance((img_width + param.width_compensate) / 2, (img_height + param.height_compensate) / 2));
            cout << "CENTER_DIS:" << center_dis << endl;
            dis.disCalculate(center_dis, d16, points);
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
            int center_dis = dis.disCalculate(0, d16, points);
        }
        if (param.cam_module == REALSENSE)
        {
            // 选取1/5 * 1/5中心区域
            int img_width = param.RS_width;
            int img_height = param.RS_height;
            int current_dis = 0;
            int min_dis = int(1000 * rsDepthFrames.back().get_distance(param.RS_width / 2, param.RS_height / 2));
            for (int i = 0.4 * img_width + param.width_compensate; i <= 0.6 * img_width + param.width_compensate && i >= 0.4 * img_width + param.width_compensate; i++)
            {
                for (int j = 0.4 * img_height + param.height_compensate; j <= 0.6 * img_height + param.height_compensate && j >= 0.4 * img_height + param.height_compensate; j++)
                {
                    // cout << "i:" << i << " j:" << j << endl;
                    current_dis = int(1000 * rsDepthFrames.back().get_distance(i, j));
                    // cout << "current-dis:" << current_dis << endl;
                    if (current_dis < min_dis && current_dis != 0)
                    {
                        min_dis = current_dis;
                    }
                }
            }
            cout << "MIN-DIS: " << min_dis << endl;
            dis.disCalculate(min_dis, d16, points);
        }
        break;
    }
    default:
        break;
    }
}

int Frame::Decider(Face &face, Dis &dis, cv::Mat &color, cv::Mat &d16, int &detect_count)
{
    // 复杂的判断过程(待简化)
    int DIS;
    int situation = 0;
    if (detect_count == 0)
    {
        // 进行检测的帧
        bool detected = face.faceDetect(color, face.detected_faces, this->drop_count);
        if (detected)
        {
            if (!face.face_center.empty())
            {
                // 若检测到人脸：锁定人脸（todo：多人脸策略）
                situation = 1;
                cout << "----1----" << endl;
                drop_init = 1; // 重新初始化掉帧计算器
            }
        }
        else
        {
            if (!dis.target_dis.empty())
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
                    if (!face.face_center.empty())
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
        if (!face.face_center.empty())
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
        // situation=1:进入掉帧处理
        this->dropProcess(param.DROP_PROCESS_MODE, dis, d16);
        break;
    }
    case 1:
    {
        if (param.cam_module == REALSENSE)
        {
            // situation=1:锁定队列末尾的面部
            cout << "s1-1" << endl;
            float center_y = face.face_center.back().y;
            cout << "s1-2" << endl;
            if (param.INVERT_ON)
            {
                center_y = param.RS_height - face.face_center.back().y;
            }
            int face_dis = int(1000 * rsDepthFrames.back().get_distance(face.face_center.back().x, center_y));
            // 对realsense相机来说，discalculate并不承担计算距离的功能
            // 通过第一个int直接传入距离，函数中只是对距离进行滤波和错误处理
            DIS = dis.disCalculate(face_dis, d16, face.face_center);
            cout << "s1-3" << endl;
            break;
        }
        else
        {
            DIS = dis.disCalculate(1, d16, face.face_center);
            break;
        }
    }
    default:
        this->dropProcess(param.DROP_PROCESS_MODE, dis, d16);
        break;
    }

    if (!dis.target_dis.empty())
    {
        DIS = dis.target_dis.back();
        cout << "DIS" << DIS << endl;
        cv::putText(color, cv::format("%d", DIS), cv::Point2i(15, 30), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 0, 0), 2);
    }
    else
    {
        DIS = 1000;
        cout << "ERROR!-距离队列异常" << endl;
    }
    return DIS;
}

int Frame::disInterPolater(int &dis)
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
    else if (dis < 1500 && dis >= 500)
    {
        target_pulse = lens_param.A + (lens_param.B - lens_param.A) * (dis - 500) / 1000;
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