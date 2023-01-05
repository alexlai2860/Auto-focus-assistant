/**
 * @file realsense_reader.cpp
 * @author 赖建宇 (lai.jianyu@foxmail.com)
 * @brief
 * @version 1.0
 * @date 2023-01-05
 *
 * @copyright Copyright SCUT RobotLab(c) 2021
 *
 */

#include "realsense_reader.h"
#include <iostream>

using namespace std;

#define width 1280
#define height 720
#define fps 30

void RealsenseReader::rs_read()
{
    // judge whether devices is exist or not
    rs2::context ctx;
    auto list = ctx.query_devices(); // Get a snapshot of currently connected devices
    if (list.size() == 0)
        throw std::runtime_error("No device detected. Is it plugged in?");
    rs2::device dev = list.front();

    //
    rs2::frameset frames;
    // Contruct a pipeline which abstracts the device
    rs2::pipeline pipe;
    // Create a configuration for configuring the pipeline with a non default profile
    rs2::config cfg; // 创建一个以非默认配置的配置用来配置管道
    // Add desired streams to configuration
    cfg.enable_stream(RS2_STREAM_COLOR, width, height, RS2_FORMAT_BGR8, fps); // 向配置添加所需的流
    cfg.enable_stream(RS2_STREAM_DEPTH, width, height, RS2_FORMAT_Z16, fps);

    // get depth scale
    // float depth_scale = get_depth_scale(profile.get_device());

    // start stream
    pipe.start(cfg); // 指示管道使用所请求的配置启动流

    while (1)
    {
        frames = pipe.wait_for_frames(); // 等待所有配置的流生成框架

        // Align to depth
        rs2::align align_to_color(RS2_STREAM_COLOR);
        frames = align_to_color.process(frames);

        // Get imu data
        // if (rs2::motion_frame accel_frame = frames.first_or_default(RS2_STREAM_ACCEL))
        // {
        //     rs2_vector accel_sample = accel_frame.get_motion_data();
        //     std::cout << "Accel:" << accel_sample.x << ", " << accel_sample.y << ", " << accel_sample.z << std::endl;
        // }
        // if (rs2::motion_frame gyro_frame = frames.first_or_default(RS2_STREAM_GYRO))
        // {
        //     rs2_vector gyro_sample = gyro_frame.get_motion_data();
        //     std::cout << "Gyro:" << gyro_sample.x << ", " << gyro_sample.y << ", " << gyro_sample.z << std::endl;
        // }

        // Get each frame
        rs2::frame color_frame = frames.get_color_frame();
        rs2::depth_frame depth_frame = frames.get_depth_frame();
        // rs2::video_frame ir_frame_left = frames.get_infrared_frame(1);
        // rs2::video_frame ir_frame_right = frames.get_infrared_frame(2);

        // Creating OpenCV Matrix from a color image
        cv::Mat color(cv::Size(width, height), CV_8UC3, (void *)color_frame.get_data(), cv::Mat::AUTO_STEP);
        cv::Mat pic_depth(cv::Size(width, height), CV_16U, (void *)depth_frame.get_data(), cv::Mat::AUTO_STEP);

        float dis_to_center = depth_frame.get_distance(width / 2, height / 2);
        cout << "distance : " << dis_to_center << endl;

        // Display in a GUI
        cv::namedWindow("Display Image", cv::WINDOW_AUTOSIZE);
        cv::imshow("Display Image", color);
        cv::waitKey(1);
        cv::imshow("Display depth", pic_depth * 15);
        cv::waitKey(1);
        // cv::imshow("Display pic_left", pic_left);
        // cv::waitKey(1);
        // cv::imshow("Display pic_right",pic_right);
        // cv::waitKey(1);
    }
}
