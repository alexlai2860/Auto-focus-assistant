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

            if (detect_count == 0)
            {
                bool detected = face.faceDetect(colorFrame.frame, face.detected_faces, this->drop_count);
                if (detected)
                {
                    dis.disCalculate(1, d16, face.face_center);
                    if (!dis.target_dis.empty())
                    {
                        // if (dis.movDecider(t0, face.face_center))
                        // {
                        //     cv::putText(dColor, "move", cv::Point2i(int(face.face_center.back().x), int(face.face_center.back().y) + 15), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 0, 0), 2);
                        // }
                        // else
                        // {
                        //     cv::putText(dColor, "static", cv::Point2i(int(face.face_center.back().x), int(face.face_center.back().y) + 15), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 0, 0), 2);
                        // }
                        cv::circle(dColor, face.face_center.back(), 2, cv::Scalar(0, 200, 200), 5);
                        cv::putText(dColor, cv::format("%d", DIS), cv::Point2i(int(face.face_center.back().x), int(face.face_center.back().y) - 15), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(255, 255, 255), 2);
                    }
                    drop_init = 1; // 重新初始化掉帧计算器
                }
                else
                {
                    if (!dis.target_dis.empty())
                    {
                        drop_count++;
                        if (drop_count >= param.MAX_DROP_FRAME)
                        {
                            this->dropProcess(param.DROP_PROCESS_MODE, dis, d16);
                        }
                    }
                    else
                    {
                        this->dropProcess(param.DROP_PROCESS_MODE, dis, d16);
                    }
                    drop_init = 0;
                }
                detect_init = 1; // 重新初始化面部识别帧率计数器
            }
            else
            {
                if (!face.face_center.empty())
                {
                    dis.disCalculate(1, d16, face.face_center);
                    if (!dis.target_dis.empty())
                    {
                        // if (dis.movDecider(t0, face.face_center))
                        // {
                        //     cv::putText(dColor, "move", cv::Point2i(int(face.face_center.back().x), int(face.face_center.back().y) + 15), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 0, 0), 2);
                        // }
                        // else
                        // {
                        //     cv::putText(dColor, "static", cv::Point2i(int(face.face_center.back().x), int(face.face_center.back().y) + 15), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 0, 0), 2);
                        // }
                        cv::circle(dColor, face.face_center.back(), 2, cv::Scalar(0, 200, 200), 5);
                        cv::putText(dColor, cv::format("%d", DIS), cv::Point2i(int(face.face_center.back().x), int(face.face_center.back().y) - 15), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(255, 255, 255), 2);
                    }
                    else
                    {
                        this->dropProcess(param.DROP_PROCESS_MODE, dis, d16);
                    }
                }
                else
                {
                    this->dropProcess(param.DROP_PROCESS_MODE, dis, d16);
                }
                detect_init = 0;
                detect_count = detect_count - 1; // 计数器递减至0
            }

            if (!dis.target_dis.empty())
            {
                DIS = dis.target_dis.back();
                cout << "DIS" << DIS << endl;
            }
            else
            {
                DIS = 1000;
                cout << "ERROR!-距离队列异常" << endl;
            }

            // 计算差值，写入串口，同时进行异常处理
            int current_pulse = motor.readPulse(data);
            int target_pulse = (lens_param.A * pow(DIS, 5) + lens_param.B * pow(DIS, 4) + lens_param.C * pow(DIS, 3) + lens_param.D * pow(DIS, 2) + lens_param.E * DIS + lens_param.F);
            // cout << "current_pulse = " << current_pulse << endl;
            // cout << "target_pulse = " << target_pulse << endl;
            if (abs(target_pulse - current_pulse) < abs(lens_param.INFINIT_PULSE - lens_param.INIT_PULSE))
            {
                int min_pulse = (lens_param.INFINIT_PULSE < lens_param.INIT_PULSE ? lens_param.INFINIT_PULSE : lens_param.INIT_PULSE);
                int max_pulse = (lens_param.INFINIT_PULSE > lens_param.INIT_PULSE ? lens_param.INFINIT_PULSE : lens_param.INIT_PULSE);
                if (target_pulse < max_pulse && target_pulse > min_pulse)
                {
                    if (current_pulse <= max_pulse + 50 && current_pulse >= min_pulse - 50)
                    {
                        // motor.writePulse((target_pulse - current_pulse), data);
                        // cout << "写入中" << endl;
                    }
                    else
                    {
                        cout << "ERROR!-当前值异常" << endl;
                    }
                }
                else
                {
                    cout << "ERROR!-目标值异常" << endl;
                }
            }
            else
            {
                cout << "ERROR!-差值过大" << endl;
            }

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
        // frames = pipe.wait_for_frames();
        // // Get each frame
        // rs2::frame c = frames.get_color_frame();
        // rs2::depth_frame d = frames.get_depth_frame();
        // // rs2::video_frame ir_frame_left = frames.get_infrared_frame(1);
        // // rs2::video_frame ir_frame_right = frames.get_infrared_frame(2);

        // rsDepthFrames.push_back(d);
        // rsColorFrames.push_back(c);
        // if (rsColorFrames.size() > 5)
        // {
        //     rsColorFrames.pop_front();
        // }
        // if (rsDepthFrames.size() > 5)
        // {
        //     rsDepthFrames.pop_front();
        // }
        // Creating OpenCV Matrix from a color image
        rs2::frame color_frame = rsColorFrames.back();
        rs2::frame depth_frame = rsDepthFrames.back();

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

        if (detect_count == 0)
        {
            bool detected = face.faceDetect(color, face.detected_faces, this->drop_count);
            if (detected)
            {
                float center_y = face.face_center.back().y;
                if (param.INVERT_ON)
                {
                    center_y = param.RS_height - face.face_center.back().y;
                }
                int face_dis = int(1000 * rsDepthFrames.back().get_distance(face.face_center.back().x, center_y));
                dis.disCalculate(face_dis, d16, face.face_center);
                if (!dis.target_dis.empty())
                {
                    // if (dis.movDecider(t0, face.face_center))
                    // {
                    //     cv::putText(dColor, "move", cv::Point2i(int(face.face_center.back().x), int(face.face_center.back().y) + 15), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 0, 0), 2);
                    // }
                    // else
                    // {
                    //     cv::putText(dColor, "static", cv::Point2i(int(face.face_center.back().x), int(face.face_center.back().y) + 15), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 0, 0), 2);
                    // }
                    cv::circle(depth, face.face_center.back(), 2, cv::Scalar(0, 200, 200), 5);
                    cv::putText(depth, cv::format("%d", DIS), cv::Point2i(int(face.face_center.back().x), int(face.face_center.back().y) - 15), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(255, 255, 255), 2);
                }
                drop_init = 1; // 重新初始化掉帧计算器
            }
            else
            {
                if (!dis.target_dis.empty())
                {
                    drop_count++;
                    if (drop_count >= param.MAX_DROP_FRAME)
                    {
                        this->dropProcess(param.DROP_PROCESS_MODE, dis, d16);
                    }
                }
                else
                {
                    this->dropProcess(param.DROP_PROCESS_MODE, dis, d16);
                }
                drop_init = 0;
            }
            detect_init = 1; // 重新初始化面部识别帧率计数器
        }
        else
        {
            if (!face.face_center.empty())
            {
                float center_y = face.face_center.back().y;
                if (param.INVERT_ON)
                {
                    center_y = param.RS_height - face.face_center.back().y;
                }
                int face_dis = int(1000 * rsDepthFrames.back().get_distance(face.face_center.back().x, center_y));
                dis.disCalculate(face_dis, d16, face.face_center);
                if (!dis.target_dis.empty())
                {
                    // if (dis.movDecider(t0, face.face_center))
                    // {
                    //     cv::putText(dColor, "move", cv::Point2i(int(face.face_center.back().x), int(face.face_center.back().y) + 15), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 0, 0), 2);
                    // }
                    // else
                    // {
                    //     cv::putText(dColor, "static", cv::Point2i(int(face.face_center.back().x), int(face.face_center.back().y) + 15), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 0, 0), 2);
                    // }
                    cv::circle(depth, face.face_center.back(), 2, cv::Scalar(0, 200, 200), 5);
                    cv::putText(depth, cv::format("%d", DIS), cv::Point2i(int(face.face_center.back().x), int(face.face_center.back().y) - 15), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(255, 255, 255), 2);
                }
                else
                {
                    this->dropProcess(param.DROP_PROCESS_MODE, dis, d16);
                }
            }
            else
            {
                this->dropProcess(param.DROP_PROCESS_MODE, dis, d16);
            }
            detect_init = 0;
            detect_count = detect_count - 1; // 计数器递减至0
        }

        if (!dis.target_dis.empty())
        {
            DIS = dis.target_dis.back();
            cout << "DIS" << DIS << endl;
        }
        else
        {
            DIS = 1000;
            cout << "ERROR!-距离队列异常" << endl;
        }

        // 计算差值，写入串口，同时进行异常处理
        int current_pulse = motor.readPulse(data);
        int target_pulse = (lens_param.A * pow(DIS, 5) + lens_param.B * pow(DIS, 4) + lens_param.C * pow(DIS, 3) + lens_param.D * pow(DIS, 2) + lens_param.E * DIS + lens_param.F);
        // cout << "current_pulse = " << current_pulse << endl;
        // cout << "target_pulse = " << target_pulse << endl;
        if (abs(target_pulse - current_pulse) < abs(lens_param.INFINIT_PULSE - lens_param.INIT_PULSE))
        {
            int min_pulse = (lens_param.INFINIT_PULSE < lens_param.INIT_PULSE ? lens_param.INFINIT_PULSE : lens_param.INIT_PULSE);
            int max_pulse = (lens_param.INFINIT_PULSE > lens_param.INIT_PULSE ? lens_param.INFINIT_PULSE : lens_param.INIT_PULSE);
            if (target_pulse < max_pulse && target_pulse > min_pulse)
            {
                if (current_pulse <= max_pulse + 50 && current_pulse >= min_pulse - 50)
                {
                    motor.writePulse((target_pulse - current_pulse), data);
                    cout << "写入中" << endl;
                }
                else
                {
                    cout << "ERROR!-当前值异常" << endl;
                }
            }
            else
            {
                cout << "ERROR!-目标值异常" << endl;
            }
        }
        else
        {
            cout << "ERROR!-差值过大" << endl;
        }

        // show dcolor frame
        imshow("Depth", depth * 15);

        // Show color frame
        imshow("Color", color);

        // Exit on Esc key press
        int key = cv::waitKey(1);
        cout << "run time = " << 1000 * ((cv::getTickCount() - t1) / cv::getTickFrequency()) << " ms" << endl;
    }
}

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

void Frame::dropProcess(int mode, Dis &dis, cv::Mat &d16)
{
    switch (mode)
    {
    case 1:
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
            int center_dis = int(1000 * rsDepthFrames.back().get_distance(param.RS_width / 2, param.RS_height / 2));
            dis.disCalculate(center_dis, d16, points);
        }
    }
    case 2:
    {
    }
    default:
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
            int center_dis = int(1000 * rsDepthFrames.back().get_distance(param.RS_width / 2, param.RS_height / 2));
            dis.disCalculate(center_dis, d16, points);
        }
        break;
    }
}