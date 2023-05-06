#include <fstream>
#include <sstream>
#include <iostream>
#include <opencv2/dnn.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include "FaceLight.h"

using namespace cv;
using namespace dnn;
using namespace std;

bool FaceLight::detect(cv::Mat &color_frame, cv::Mat &depth_frame)
{
	// init
	if (!init)
	{
		scrfd.init(this->default_cfg);
		init = 1;
	}
	// detect
	if (scrfd.detect(color_frame, depth_frame))
	{
		cout << "face-detect-over" << endl;
		face.push_back(scrfd.scrfd_face.back());
		cout << "face-size " << face.size() << endl;
		if (face.size() >= param.FACE_DEQUE)
		{
			face.pop_front();
		}
		return 1;
	}
	else
	{
		return 0;
	}
}

bool FaceLight::drawBox(cv::Mat &color_frame, cv::Mat &depth_frame)
{
	return 0;
}

bool SCRFD::init(Net_config config)
{
	this->confThreshold = config.confThreshold;
	this->nmsThreshold = config.nmsThreshold;
	this->net = readNet(config.modelfile);
	return 1;
}

Mat SCRFD::resize_image(Mat srcimg, int *newh, int *neww, int *top, int *left)
{
	int srch = srcimg.rows, srcw = srcimg.cols;
	*newh = this->inpHeight;
	*neww = this->inpWidth;
	Mat dstimg;
	if (this->keep_ratio && srch != srcw)
	{
		float hw_scale = (float)srch / srcw;
		if (hw_scale > 1)
		{
			*newh = this->inpHeight;
			*neww = int(this->inpWidth / hw_scale);
			resize(srcimg, dstimg, Size(*neww, *newh), INTER_AREA);
			*left = int((this->inpWidth - *neww) * 0.5);
			copyMakeBorder(dstimg, dstimg, 0, 0, *left, this->inpWidth - *neww - *left, BORDER_CONSTANT, 0);
		}
		else
		{
			*newh = (int)this->inpHeight * hw_scale;
			*neww = this->inpWidth;
			resize(srcimg, dstimg, Size(*neww, *newh), INTER_AREA);
			*top = (int)(this->inpHeight - *newh) * 0.5;
			copyMakeBorder(dstimg, dstimg, *top, this->inpHeight - *newh - *top, 0, 0, BORDER_CONSTANT, 0);
		}
	}
	else
	{
		resize(srcimg, dstimg, Size(*neww, *newh), INTER_AREA);
	}
	return dstimg;
}

bool SCRFD::detect(Mat &frame, Mat &depth_frame)
{
	int newh = 0, neww = 0, padh = 0, padw = 0;
	Mat img = this->resize_image(frame, &newh, &neww, &padh, &padw);
	Mat blob;
	blobFromImage(img, blob, 1 / 128.0, Size(this->inpWidth, this->inpHeight), Scalar(127.5, 127.5, 127.5), true, false);
	this->net.setInput(blob);
	vector<Mat> outs;
	this->net.forward(outs, this->net.getUnconnectedOutLayersNames());

	/////generate proposals
	vector<float> confidences;
	vector<Rect> boxes;
	vector<vector<int>> landmarks;
	float ratioh = (float)frame.rows / newh, ratiow = (float)frame.cols / neww;
	int n = 0, i = 0, j = 0, k = 0, l = 0;
	for (n = 0; n < 3; n++)
	{
		int num_grid_x = (int)(this->inpWidth / this->stride[n]);
		int num_grid_y = (int)(this->inpHeight / this->stride[n]);
		float *pdata_score = (float *)outs[n * 3].data;	   /// score
		float *pdata_bbox = (float *)outs[n * 3 + 1].data; /// bounding box
		float *pdata_kps = (float *)outs[n * 3 + 2].data;  /// face landmark
		for (i = 0; i < num_grid_y; i++)
		{
			for (j = 0; j < num_grid_x; j++)
			{
				for (k = 0; k < 2; k++)
				{
					if (pdata_score[0] > this->confThreshold)
					{
						const int xmin = (int)(((j - pdata_bbox[0]) * this->stride[n] - padw) * ratiow);
						const int ymin = (int)(((i - pdata_bbox[1]) * this->stride[n] - padh) * ratioh);
						const int width = (int)((pdata_bbox[2] + pdata_bbox[0]) * this->stride[n] * ratiow);
						const int height = (int)((pdata_bbox[3] + pdata_bbox[1]) * this->stride[n] * ratioh);
						confidences.push_back(pdata_score[0]);
						boxes.push_back(Rect(xmin, ymin, width, height));
						vector<int> landmark(10, 0);
						for (l = 0; l < 10; l += 2)
						{
							landmark[l] = (int)(((j + pdata_kps[l]) * this->stride[n] - padw) * ratiow);
							landmark[l + 1] = (int)(((i + pdata_kps[l + 1]) * this->stride[n] - padh) * ratioh);
						}
						landmarks.push_back(landmark);
					}
					pdata_score++;
					pdata_bbox += 4;
					pdata_kps += 10;
				}
			}
		}
	}

	// Perform non maximum suppression to eliminate redundant overlapping boxes with
	// lower confidences
	vector<int> indices;
	vector<SingleFace> current_faces;
	dnn::NMSBoxes(boxes, confidences, this->confThreshold, this->nmsThreshold, indices);
	for (i = 0; i < indices.size(); ++i)
	{
		int idx = indices[i];
		Rect box = boxes[idx];
		rectangle(frame, Point(box.x, box.y), Point(box.x + box.width, box.y + box.height), Scalar(0, 0, 255), 2);
		for (k = 0; k < 10; k += 2)
		{
			circle(frame, Point(landmarks[idx][k], landmarks[idx][k + 1]), 1, Scalar(0, 255, 0), -1);
		}

		SingleFace single_face;
		cv::Point2i center(box.x + box.width / 2, box.y + box.height / 2);
		single_face.center = center;
		// single_face.single_face = selected_face;
		single_face.cam_dis = depth.getPointDepth(depth_frame, center);
		single_face.face_rect = box;
		single_face.detected = 1;
		current_faces.push_back(single_face);

		// Get the label for the class name and its confidence
		string label = format("%.2f", confidences[idx]);
		// Display the label at the top of the bounding box
		int baseLine;
		Size labelSize = getTextSize(label, FONT_HERSHEY_SIMPLEX, 0.5, 1, &baseLine);
		int top = max(box.y, labelSize.height);
		// rectangle(frame, Point(left, top - int(1.5 * labelSize.height)), Point(left + int(1.5 * labelSize.width), top + baseLine), Scalar(0, 255, 0), FILLED);
		putText(frame, label, Point(box.x, top), FONT_HERSHEY_SIMPLEX, 0.75, Scalar(0, 255, 0), 1);
	}
	this->scrfd_face.push_back(current_faces);
	return 1;
}

/**
 * @brief 面部识别结果筛选函数（异常结果/超出roi）
 *
 * @param faces 储存的面部数据
 * @param i 第i个面部
 * @return true valid_face
 * @return false invalid_face
 */
bool SCRFD::isValidFace(SingleFace single_face)
{
	// 筛除面积过大/面积过小/处于ROI区域外的faces
	int frame_area = param.RS_height * param.RS_width;
	cv::Point2f center = single_face.center;
	float zoom_rate = (float)param.LENS_LENGTH / 24.f;
	cv::Point2i ROI(param.RS_width / zoom_rate, param.RS_height / zoom_rate);
	cv::Rect2i face_box = single_face.face_rect;
	int border_x = (param.RS_width - ROI.x) / 2;
	int border_y = (param.RS_height - ROI.y) / 2;
	if (face_box.area()> 0.25 * frame_area)
	{
		return 0;
	}
	else if (center.x < param.RS_width - border_x - ROI.x || center.x > param.RS_width - border_x)
	{
		return 0;
	}
	else if (center.y < param.RS_height - border_y - ROI.y || center.y > param.RS_height - border_y)
	{
		return 0;
	}
	else if (face_box.area() < param.DRAW_BOX_MIN * (param.DRAW_BOX_MIN + 10))
	{
		return 0;
	}
	return 1;
}
