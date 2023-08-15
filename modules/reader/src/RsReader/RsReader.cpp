/**
 * @file RsReader.cpp
 * @author 赖建宇 (lai.jianyu@foxmail.com)
 * @brief
 * @version 1.0
 * @date 2023-03-11
 *
 * @copyright Copyright SCUT RobotLab(c) 2021
 *
 */

#include "RsReader.h"

using namespace std;

bool RsReader::camInit()
{
    // judge whether devices is exist or not
    rs2::context ctx;
    auto list = ctx.query_devices(); // Get a snapshot of currently connected devices
    if (list.size() == 0)
        throw std::runtime_error("No device detected. Is it plugged in?");
    rs2::device dev = list.front();

    // Add desired streams to configuration
    cfg.enable_stream(RS2_STREAM_COLOR, param.RS_width, param.RS_height, RS2_FORMAT_BGR8, param.RS_fps); // 向配置添加所需的流
    cfg.enable_stream(RS2_STREAM_DEPTH, param.RS_width, param.RS_height, RS2_FORMAT_Z16, param.RS_fps);

    std::ifstream file("../param/test3.json");
    if (file.good())
    {
        std::string str((std::istreambuf_iterator<char>(file)), std::istreambuf_iterator<char>());

        auto prof = cfg.resolve(pipe);
        if (auto advanced = prof.get_device().as<rs400::advanced_mode>())
        {
            advanced.load_json(str);
            // cout << "json load success 1" << endl;
        }
        cout << "json load success 2" << endl;
    }
    else
    {
        cout << "json load fail " << endl;
    }
    spat_filter.set_option(RS2_OPTION_FILTER_MAGNITUDE, 2);
    temp_filter.set_option(RS2_OPTION_FILTER_SMOOTH_ALPHA, 0.4);
    temp_filter.set_option(RS2_OPTION_FILTER_SMOOTH_DELTA, 20);
    temp_filter.set_option(RS2_OPTION_HOLES_FILL, 6);
    hf_filter.set_option(RS2_OPTION_HOLES_FILL, 1);

    pipe.start(cfg); // 指示管道使用所请求的配置启动流
    sen = pipe.get_active_profile().get_device().query_sensors()[1];
    return 1;
}

/**
 * @brief 从pipe中读取frame
 *
 * @return true
 * @return false 时间戳overlap
 */
bool RsReader::read()
{
    frames = pipe.wait_for_frames();
    // Get each frame
    rs2::frame color_f = frames.get_color_frame();
    rs2::depth_frame depth_f = frames.get_depth_frame();
    // rs2::video_frame ir_frame_left = frames.get_infrared_frame(1);
    // rs2::video_frame ir_frame_right = frames.get_infrared_frame(2);

    rsDepthFrames.push_back(depth_f);
    rsColorFrames.push_back(color_f);
    if (rsColorFrames.size() > 2)
    {
        rsColorFrames.pop_front();
    }
    if (rsDepthFrames.size() > 2)
    {
        rsDepthFrames.pop_front();
    }

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
        return 0;
    }
    if (current_depth_timestamp == last_depth_timestamp)
    {
        return 0;
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

    this->color = color;
    this->depth = depth;

    return 1;
}