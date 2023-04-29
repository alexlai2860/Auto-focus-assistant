/**
 * @file ObjectProcessorLight.cpp
 * @author 赖建宇 (lai.jianyu@foxmail.com)
 * @brief 采用yolo_fastestv2部署的目标检测器
 * @version 1.0
 * @date 2023-04-08
 *
 * @copyright Copyright SCUT RobotLab(c) 2021
 *
 */
#include "ObjectLight.h"
#include "param.h"

using namespace cv;
using namespace dnn;
using namespace std;

bool ObjectLight::detect(cv::Mat &color_frame, cv::Mat &depth_frame)
{
    if (!init)
    {
        yolo.init("../onnx/yolo_fastestv2.onnx", 0.45, 0.45, 0.4);
        init = 1;
    }
    if (!target.empty())
    {
        cout << "target- back -size(before detect)" << target.back().size() << endl;
    }
    if (yolo.detect(color_frame, depth_frame))
    {
        cout << "detect-over" << endl;
        // target = yolo.yolo_target;
        target.push_back(yolo.yolo_target.back());
        cout << "target_size:" << target.size() << endl;
        draw_object_box = 1;
        if (target.size() > 1)
        {
            cout << "target- back-1 -size(after detect)" << target.at(target.size() - 2).size() << endl;
        }
        if (target.size() > 10)
        {
            target.pop_front();
        }
        return 1;
    }
    else
    {
        return 0;
    }
}

bool ObjectLight::drawBox(cv::Mat &color_frame, cv::Mat &depth_frame)
{
    // cout<<"target-size"<<target.size()<<endl;
    // cout<<"target-back-size"<<target.back().size()<<endl;

    const vector<SingleObject> object_vector = target.back();
    for (int i = 0; i < object_vector.size(); i++)
    {
        if (object_vector.at(i).init_trigger == -1)
        {
            cv::Rect target_box = object_vector.at(i).single_object_box;
            // Draw a rectangle displaying the bounding box
            rectangle(color_frame, target_box.tl(), cv::Point2i(target_box.x + target_box.width, target_box.y + target_box.height), Scalar(0, 0, 255), 3);

            // Get the label for the class name and its confidence
            string label = format("%.2f", object_vector.at(i).conf);
            label = format("%.2f", object_vector.at(i).cam_dis) + ":" + label;

            // Display the label at the top of the bounding box
            int baseLine;
            Size labelSize = getTextSize(label, FONT_HERSHEY_SIMPLEX, 0.5, 1, &baseLine);
            int top = target_box.tl().y;
            top = max(top, labelSize.height);
            // rectangle(frame, Point(left, top - int(1.5 * labelSize.height)), Point(left + int(1.5 * labelSize.width), top + baseLine), Scalar(0, 255, 0), FILLED);
            putText(color_frame, label, target_box.tl(), FONT_HERSHEY_SIMPLEX, 1, Scalar(0, 255, 0), 1);

            if (!object_vector.at(i).single_face_in_object.empty())
            {
                cv::Rect2i face_box = object_vector.at(i).single_face_in_object;
                rectangle(color_frame, face_box.tl(), cv::Point2i(face_box.x + face_box.width, face_box.y + face_box.height), Scalar(0, 255, 255), 3);
            }

            cout << "draw-objects-vector-" << i << ":" << target_box << endl;
        }
    }

    // yolo.drawPred(color_frame, depth_frame);
    return 1;
}

void yolo_fast::simpleNMS(vector<SingleObject> &current_object)
{
    int overlap_threshold = 0.3;
    sort(current_object.begin(), current_object.end(), [&](SingleObject s1, SingleObject s2)
         { return s1.conf > s2.conf; });
    // 2.先求出所有bbox自己的大小
    std::vector<float> area(current_object.size());
    for (int i = 0; i < current_object.size(); ++i)
    {
        area[i] = current_object.at(i).single_object_box.area();
    }
    // 3.循环
    for (int i = 0; i < current_object.size(); ++i)
    {
        for (int j = i + 1; j < current_object.size();)
        {
            cv::Rect object_i = current_object[i].single_object_box;
            cv::Rect object_j = current_object[j].single_object_box;

            float left = std::max(object_i.x, object_j.x);
            float right = std::min(object_i.x + object_i.width, object_j.x + object_j.width);
            float top = std::max(object_i.y, object_j.y);
            float bottom = std::min(object_i.y + object_i.height, object_j.y + object_j.height);
            float width = std::max(right - left + 1, 0.f);
            float height = std::max(bottom - top + 1, 0.f);
            float u_area = height * width;
            float iou = (u_area) / (area[i] + area[j] - u_area);
            if (iou >= overlap_threshold)
            {
                current_object.erase(current_object.begin() + j);
                area.erase(area.begin() + j);
            }
            else
            {
                ++j;
            }
        }
    }
}

bool yolo_fast::init(string modelpath, float obj_Threshold, float conf_Threshold, float nms_Threshold)
{
    this->objThreshold = obj_Threshold;
    this->confThreshold = conf_Threshold;
    this->nmsThreshold = nms_Threshold;

    ifstream ifs(this->classesFile.c_str());
    string line;
    while (getline(ifs, line))
        this->classes.push_back(line);
    this->num_class = this->classes.size();
    this->net = readNet(modelpath);
    return 1;
}

void yolo_fast::drawPred(cv::Mat &frame, const cv::Mat &depth_frame)
{
    // Perform non maximum suppression to eliminate redundant overlapping boxes with
    // lower confidences
    vector<int> indices;
    NMSBoxes(boxes, confidences, this->confThreshold, this->nmsThreshold, indices);
    // cout << "size" << indices.size() << endl;
    for (size_t i = 0; i < indices.size(); ++i)
    {
        int idx = indices[i];
        Rect box = boxes[idx];
        this->drawSinglePred(classIds[idx], confidences[idx], box.x, box.y,
                             box.x + box.width, box.y + box.height, frame, depth_frame);
    }
}

void yolo_fast::drawSinglePred(int classId, float conf, int left, int top, int right, int bottom, Mat &frame, const Mat &depth_frame) // Draw the predicted bounding box
{
    // Draw a rectangle displaying the bounding box
    rectangle(frame, Point(left, top), Point(right, bottom), Scalar(0, 0, 255), 3);

    // Get the label for the class name and its confidence
    string label = format("%.2f", conf);
    label = this->classes[classId] + ":" + label;

    // Display the label at the top of the bounding box
    int baseLine;
    Size labelSize = getTextSize(label, FONT_HERSHEY_SIMPLEX, 0.5, 1, &baseLine);
    top = max(top, labelSize.height);
    // rectangle(frame, Point(left, top - int(1.5 * labelSize.height)), Point(left + int(1.5 * labelSize.width), top + baseLine), Scalar(0, 255, 0), FILLED);
    putText(frame, label, Point(left, top), FONT_HERSHEY_SIMPLEX, 1, Scalar(0, 255, 0), 1);
}

bool yolo_fast::detect(Mat &frame, const cv::Mat &depth_frame)
{
    Mat blob;
    blobFromImage(frame, blob, 1 / 255.0, Size(this->inpWidth, this->inpHeight));
    this->net.setInput(blob);
    vector<Mat> outs;
    this->net.forward(outs, this->net.getUnconnectedOutLayersNames());

    boxes.clear();
    confidences.clear();
    classIds.clear();

    cout << "detect-start" << endl;

    // generate proposals
    float ratioh = (float)frame.rows / this->inpHeight, ratiow = (float)frame.cols / this->inpWidth;
    int n = 0, q = 0, i = 0, j = 0, nout = this->anchor_num * 5 + this->classes.size(), row_ind = 0;
    float *pdata = (float *)outs[0].data;
    vector<SingleObject> current_objects;
    for (n = 0; n < this->num_stage; n++) // stage
    {
        int num_grid_x = (int)(this->inpWidth / this->stride[n]);
        int num_grid_y = (int)(this->inpHeight / this->stride[n]);
        for (i = 0; i < num_grid_y; i++)
        {
            for (j = 0; j < num_grid_x; j++)
            {
                Mat scores = outs[0].row(row_ind).colRange(this->anchor_num * 5, outs[0].cols);
                Point classIdPoint;
                double max_class_socre;
                // Get the value and location of the maximum score
                minMaxLoc(scores, 0, &max_class_socre, 0, &classIdPoint);
                for (q = 0; q < this->anchor_num; q++) /// anchor
                {
                    const float anchor_w = this->anchors[n][q * 2];
                    const float anchor_h = this->anchors[n][q * 2 + 1];
                    float box_score = pdata[4 * this->anchor_num + q];
                    if (box_score > this->objThreshold && max_class_socre > this->confThreshold)
                    {
                        float cx = (pdata[4 * q] * 2.f - 0.5f + j) * this->stride[n];     /// cx
                        float cy = (pdata[4 * q + 1] * 2.f - 0.5f + i) * this->stride[n]; /// cy
                        float w = powf(pdata[4 * q + 2] * 2.f, 2.f) * anchor_w;           /// w
                        float h = powf(pdata[4 * q + 3] * 2.f, 2.f) * anchor_h;           /// h

                        int left = MAX((cx - 0.5 * w) * ratiow, 0);
                        int top = MAX((cy - 0.5 * h) * ratioh, 0); // 防止小于0报错

                        classIds.push_back(classIdPoint.x);
                        confidences.push_back(box_score * max_class_socre);
                        boxes.push_back(Rect(left, top, (int)(w * ratiow), (int)(h * ratioh)));

                        cout << "param-DBM" << param.DRAW_BOX_MIN << endl;
                        if (boxes.back().area() > param.DRAW_BOX_MIN * param.DRAW_BOX_MIN)
                        {
                            if (classIdPoint.x == 0 && box_score * max_class_socre > 0.4)
                            {
                                // 强制过滤目标(临时)
                                cout << "new-box--" << i << endl;
                                SingleObject single_object;
                                single_object.id = classIdPoint.x;
                                single_object.conf = box_score * max_class_socre;
                                single_object.single_object_box = cv::Rect(left, top, (int)(w * ratiow), (int)(h * ratioh));
                                single_object.cam_dis = depth.getTargetDepth(depth_frame, single_object.single_object_box, classIdPoint.x);
                                cout << "half-done" << endl;
                                single_object.center = cv::Point2i(left + (int)(w * ratiow) / 2, top + (int)(h * ratioh) / 2);
                                single_object.detected = 1;
                                current_objects.push_back(single_object);
                                cout << "new-box-added" << endl;
                            }
                        }
                    }
                }
                row_ind++;
                pdata += nout;
            }
        }
    }
    cout << "detect-done" << endl;
    if (!current_objects.empty())
    {
        this->simpleNMS(current_objects);
        yolo_target.push_back(current_objects);
    }
    cout << "new-box-detect-size:" << current_objects.size() << endl;
    cout << "yolo_target_size:" << yolo_target.size() << endl;
    cout << "new-box-add-done" << endl;
    if (yolo_target.size() >= param.FACE_DEQUE)
    {
        // 暂时用FACE_DEQUE判断
        yolo_target.pop_front();
    }
    if (!current_objects.empty())
    {
        return 1;
    }
    else
    {
        return 0;
    }
    // return 1;
}