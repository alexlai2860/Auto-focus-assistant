/**
 * @file main.cpp
 * @author 赖建宇 (lai.jianyu@foxmail.com)
 * @brief
 * @version 1.0
 * @date 2022-11-29
 *
 * @copyright Copyright SCUT RobotLab(c) 2021
 *
 */

#include "processor.h"
#include <iostream>
#include <string>

using namespace std;

int main()
{
    cv::VideoCapture depthStream(cv::CAP_OPENNI2_ASTRA);
    cv::VideoCapture colorStream(4, cv::CAP_V4L2);
    int64 t0 = cv::getTickCount();

    colorStream.set(cv::CAP_PROP_FRAME_WIDTH, 640);
    colorStream.set(cv::CAP_PROP_FRAME_HEIGHT, 480);
    depthStream.set(cv::CAP_PROP_FRAME_WIDTH, 640);
    depthStream.set(cv::CAP_PROP_FRAME_HEIGHT, 480);
    depthStream.set(cv::CAP_PROP_OPENNI2_MIRROR, 0);
    cout << "Depth stream: "
         << depthStream.get(cv::CAP_PROP_FRAME_WIDTH) << "x" << depthStream.get(cv::CAP_PROP_FRAME_HEIGHT)
         << " @" << depthStream.get(cv::CAP_PROP_FPS) << " fps" << endl;

    Face face1;
    Dis dis1;
    Frame cam1;

    Data data1;
    cam1.processFrame(colorStream, depthStream, face1, dis1, t0, data1);

    return 0;
}