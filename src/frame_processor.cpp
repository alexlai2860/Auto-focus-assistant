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
#include "param.h"

void Frame::processFrame(cv::VideoCapture &colorStream, cv::VideoCapture &depthStream, Face &face, Dis &dis, int64 &t0, Data &data)
{
    std::list<Frame> depthFrames, colorFrames;
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
                 Frame f;
                 f.timestamp = cv::getTickCount();
                 depthStream.retrieve(f.frame, cv::CAP_OPENNI_DEPTH_MAP);
                 if (f.frame.empty())
                 {
                     cerr << "ERROR: Failed to decode frame from depth stream" << endl;
                     break;
                 }

                 {
                     std::lock_guard<std::mutex> lk(mtx);
                     if (depthFrames.size() >= maxFrames)
                         depthFrames.pop_front();
                     depthFrames.push_back(f);
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
                 Frame f;
                 f.timestamp = cv::getTickCount();
                 colorStream.retrieve(f.frame);
                 if (f.frame.empty())
                 {
                     cerr << "ERROR: Failed to decode frame from color stream" << endl;
                     break;
                 }
                 
                 {
                     std::lock_guard<std::mutex> lk(mtx);
                     if (colorFrames.size() >= maxFrames)
                         colorFrames.pop_front();
                     colorFrames.push_back(f);
                 }
                 dataReady.notify_one();
             }
         } });
    while (!isFinish)
    {
        std::unique_lock<std::mutex> lk(mtx);
        while (!isFinish && (depthFrames.empty() || colorFrames.empty()))
        {
            dataReady.wait(lk);
        }
        while (!depthFrames.empty() && !colorFrames.empty())
        {
            if (!lk.owns_lock())
                lk.lock();

            int64 t1 = cv::getTickCount();
            // Get a frame from the list
            Frame depthFrame = depthFrames.front();
            int64 depthT = depthFrame.timestamp;

            // Get a frame from the list
            Frame colorFrame = colorFrames.front();
            int64 colorT = colorFrame.timestamp;

            // Half of frame period is a maximum time diff between frames
            const int64 maxTdiff = int64(1000000000 / (2 * colorStream.get(cv::CAP_PROP_FPS)));
            if (depthT + maxTdiff < colorT)
            {
                depthFrames.pop_front();
                continue;
            }
            else if (colorT + maxTdiff < depthT)
            {
                colorFrames.pop_front();
                continue;
            }
            depthFrames.pop_front();
            colorFrames.pop_front();
            lk.unlock();

            cv::Mat d8, d16, dColor;
            this->depth_frames = depthFrames;
            this->color_frames = colorFrames;
            this->drop_count = 0;

            // convert to dcolor frame(d8) and high resolution depth frame(d16)
            depthFrame.frame.convertTo(d8, CV_8U, 255.0 / 8000);
            applyColorMap(d8, dColor, cv::COLORMAP_RAINBOW);
            depthFrame.frame.convertTo(d16, CV_16U, 65535.0 / 8000); // the effective measure distance is 8000mm

            // FPS control
            int fps = param.FPS; // support 1,2,3,5,6,10,15,30
            int detect_rate = 30 / fps;
            if (this->detect_init)
            {
                this->detect_count = detect_rate - 1;
            }
            // cout << this->detect_count << endl;

            if (this->detect_count == 0)
            {
                bool detected = face.faceDetect(colorFrame.frame, face.detected_faces, this->drop_count);
                if (detected)
                {
                    dis.disCalculate(1, d16, face.face_center);
                    if (!dis.face_dis.empty())
                    {
                        if (dis.movDecider(t0, face.face_center))
                        {
                            cv::putText(dColor, "move", cv::Point2i(int(face.face_center.back().x), int(face.face_center.back().y) + 15), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 0, 0), 2);
                        }
                        else
                        {
                            cv::putText(dColor, "static", cv::Point2i(int(face.face_center.back().x), int(face.face_center.back().y) + 15), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 0, 0), 2);
                        }
                        int DIS = dis.face_dis.back();
                        cv::circle(dColor, face.face_center.back(), 2, cv::Scalar(0, 200, 200), 5);
                        cv::putText(dColor, cv::format("%d", DIS), cv::Point2i(int(face.face_center.back().x), int(face.face_center.back().y) - 15), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(255, 255, 255), 2);
                    }
                }
                this->detect_init = 1;
            }
            else
            {
                if (!face.face_center.empty())
                {
                    dis.disCalculate(1, d16, face.face_center);
                    if (!dis.face_dis.empty())
                    {
                        if (dis.movDecider(t0, face.face_center))
                        {
                            cv::putText(dColor, "move", cv::Point2i(int(face.face_center.back().x), int(face.face_center.back().y) + 15), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 0, 0), 2);
                        }
                        else
                        {
                            cv::putText(dColor, "static", cv::Point2i(int(face.face_center.back().x), int(face.face_center.back().y) + 15), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 0, 0), 2);
                        }
                        int DIS = dis.face_dis.back();
                        cv::circle(dColor, face.face_center.back(), 2, cv::Scalar(0, 200, 200), 5);
                        cv::putText(dColor, cv::format("%d", DIS), cv::Point2i(int(face.face_center.back().x), int(face.face_center.back().y) - 15), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(255, 255, 255), 2);
                    }
                }
                this->detect_init = 0;
                this->detect_count = this->detect_count - 1;
                // continue;
            }
            TransferData move_test;
            move_test.direction_and_speed1 = 0x04;
            move_test.direction_and_speed2 = 0xFF;
            move_test.pulse_h = 0x00;
            move_test.pulse_m = 0x00;
            move_test.pulse_l = 0x64;
            data.write(4, move_test);

            // show dcolor frame
            imshow("Depth (colored)", dColor);

            // Show color frame
            imshow("Color", colorFrame.frame);

            // Exit on Esc key press
            int key = cv::waitKey(1);
            if (key == 27) // ESC
            {
                isFinish = true;
                break;
            }
            cout << "run time = " << 1000 * ((cv::getTickCount() - t1) / cv::getTickFrequency()) << " ms" << endl;
        }
    }
}
