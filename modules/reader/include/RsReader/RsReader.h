/**
 * @file RsReader.h
 * @author 赖建宇 (lai.jianyu@foxmail.com)
 * @brief
 * @version 1.0
 * @date 2023-03-11
 *
 * @copyright Copyright SCUT RobotLab(c) 2021
 *
 */

#include "reader.h"

class RsReader : public reader
{
private:
    rs2::frame color_frame;
    rs2::frame depth_frame;
    rs2::frameset frames;
    rs2::pipeline pipe;

    // double last_color_timestamp;
    // double last_depth_timestamp;
    // list<rs2::frame> rsColorFrames;
    // list<rs2::depth_frame> rsDepthFrames;

    rs2::decimation_filter dec_filter;  // 抽取滤波器
    rs2::spatial_filter spat_filter;    // 空间滤波器
    rs2::threshold_filter thres_filter; // 阈值滤波器
    rs2::temporal_filter temp_filter;   // 时间滤波器
    rs2::hole_filling_filter hf_filter;

public:
    RsReader() { cout << "rsreader_constructor" << endl; };
    ~RsReader() { cout << "rsreader_distructor" << endl; };
    bool camInit() override;
    bool read() override;
};