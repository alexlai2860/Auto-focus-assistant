#include "body.h"

using namespace cv;
using namespace std;
using namespace Ort;

const int connect_list[36] = {0, 1, 0, 2, 1, 3, 2, 4, 3, 5, 4, 6, 5, 6, 5, 7, 7, 9, 6, 8, 8, 10, 5, 11, 6, 12, 11, 12, 11, 13, 13, 15, 12, 14, 14, 16};

bool Body::detect(cv::Mat &color_frame, cv::Mat &depth_frame)
{
	// init
	if (!init)
	{
		e2pose.init("../onnx/E2pose_weights/e2epose_mbnv2_1x3x320x320.onnx", 0.4);
		init = 1;
	}
	// detect
	if (e2pose.detect(color_frame))
	{
		// cout << "face-detect-over" << endl;
		// face.push_back(scrfd.scrfd_face.back());
		// cout << "face-size " << face.size() << endl;
		// if (face.size() >= param.FACE_DEQUE)
		// {
		// 	face.pop_front();
		// }
		return 1;
	}
	else
	{
		return 0;
	}
}

bool Body::drawBox(cv::Mat &color_frame, cv::Mat &depth_frame)
{
	return 0;
}

bool E2Pose::init(string model_path, float confThreshold)
{
	// 标准ort初始化
	// OrtStatus* status = OrtSessionOptionsAppendExecutionProvider_CUDA(sessionOptions, 0);
	sessionOptions.SetGraphOptimizationLevel(ORT_ENABLE_BASIC);
	const wchar_t *path = L"PATH";
	ort_session = new Session(env, model_path.data(), sessionOptions);
	size_t numInputNodes = ort_session->GetInputCount();
	size_t numOutputNodes = ort_session->GetOutputCount();
	AllocatorWithDefaultOptions allocator;
	for (int i = 0; i < numInputNodes; i++)
	{
		input_names.push_back(ort_session->GetInputName(i, allocator));
		Ort::TypeInfo input_type_info = ort_session->GetInputTypeInfo(i);
		auto input_tensor_info = input_type_info.GetTensorTypeAndShapeInfo();
		auto input_dims = input_tensor_info.GetShape();
		input_node_dims.push_back(input_dims);
	}
	for (int i = 0; i < numOutputNodes; i++)
	{
		output_names.push_back(ort_session->GetOutputName(i, allocator));
		Ort::TypeInfo output_type_info = ort_session->GetOutputTypeInfo(i);
		auto output_tensor_info = output_type_info.GetTensorTypeAndShapeInfo();
		auto output_dims = output_tensor_info.GetShape();
		output_node_dims.push_back(output_dims);
	}
	// ort可以直接读取输入长宽数据
	this->inpHeight = input_node_dims[0][2];
	this->inpWidth = input_node_dims[0][3];
	this->confThreshold = confThreshold;
	return 1;
}

void E2Pose::normalize_(Mat img)
{
	// 标准normalize(将图像数据转为vector<float>储存)
	// img.convertTo(img, CV_32F);
	int row = img.rows;
	int col = img.cols;
	this->input_image_.resize(row * col * img.channels());
	for (int c = 0; c < 3; c++)
	{
		for (int i = 0; i < row; i++)
		{
			for (int j = 0; j < col; j++)
			{
				float pix = img.ptr<uchar>(i)[j * 3 + c];
				this->input_image_[c * row * col + i * col + j] = pix;
			}
		}
	}
}

bool E2Pose::detect(Mat &srcimg)
{
	// resize方面，有些模型需要进行边缘填充(注释部分)
	Mat dstimg;
	cvtColor(srcimg, dstimg, COLOR_BGR2RGB);
	resize(dstimg, dstimg, Size(this->inpWidth, this->inpHeight));

	// const float ratio = std::min(float(this->inpHeight) / float(srcimg.rows), float(this->inpWidth) / float(srcimg.cols));
	// const int neww = int(srcimg.cols * ratio);
	// const int newh = int(srcimg.rows * ratio);

	// Mat dstimg;
	// resize(srcimg, dstimg, Size(neww, newh));
	// copyMakeBorder(dstimg, dstimg, 0, this->inpHeight - newh, 0, this->inpWidth - neww, BORDER_CONSTANT, 114);

	// 标准ort推理
	this->normalize_(dstimg);
	array<int64_t, 4> input_shape_{1, 3, this->inpHeight, this->inpWidth};

	auto allocator_info = MemoryInfo::CreateCpu(OrtDeviceAllocator, OrtMemTypeCPU);
	Value input_tensor_ = Value::CreateTensor<float>(allocator_info, input_image_.data(), input_image_.size(), input_shape_.data(), input_shape_.size());

	vector<Value> ort_outputs = ort_session->Run(RunOptions{nullptr}, &input_names[0], &input_tensor_, 1, output_names.data(), output_names.size());

	// 视不同模型确定
	const float *kpt = ort_outputs[0].GetTensorMutableData<float>();
	const float *pv = ort_outputs[1].GetTensorMutableData<float>();
	const int num_proposal = ort_outputs.at(0).GetTensorTypeAndShapeInfo().GetShape().at(1);
	const int num_pts = ort_outputs.at(0).GetTensorTypeAndShapeInfo().GetShape().at(2);
	const int len = num_pts * 3;

	// 后处理大体上和opencv-dnn一致
	vector<vector<int>> results;
	for (int i = 0; i < num_proposal; i++)
	{
		if (pv[i] >= this->confThreshold)
		{
			vector<int> human_pts(num_pts * 2, 0);
			for (int j = 0; j < num_pts; j++)
			{
				const float score = kpt[j * 3] * 2;
				if (score >= this->confThreshold)
				{
					const float x = kpt[j * 3 + 1] * srcimg.cols;
					const float y = kpt[j * 3 + 2] * srcimg.rows;
					human_pts[j * 2] = int(x);
					human_pts[j * 2 + 1] = int(y);
				}
			}
			results.push_back(human_pts);
		}
		kpt += len;
	}

	for (size_t i = 0; i < results.size(); ++i)
	{
		for (int j = 0; j < num_pts; j++)
		{
			const int cx = results[i][j * 2];
			const int cy = results[i][j * 2 + 1];
			if (cx > 0 && cy > 0)
			{
				circle(srcimg, Point(cx, cy), 3, Scalar(0, 0, 255), -1, LINE_AA);
			}

			const int start_x = results[i][connect_list[j * 2] * 2];
			const int start_y = results[i][connect_list[j * 2] * 2 + 1];
			const int end_x = results[i][connect_list[j * 2 + 1] * 2];
			const int end_y = results[i][connect_list[j * 2 + 1] * 2 + 1];
			if (start_x > 0 && start_y > 0 && end_x > 0 && end_y > 0)
			{
				line(srcimg, Point(start_x, start_y), Point(end_x, end_y), Scalar(0, 255, 0), 2, LINE_AA);
			}
		}
		const int start_x = results[i][connect_list[num_pts * 2] * 2];
		const int start_y = results[i][connect_list[num_pts * 2] * 2 + 1];
		const int end_x = results[i][connect_list[num_pts * 2 + 1] * 2];
		const int end_y = results[i][connect_list[num_pts * 2 + 1] * 2 + 1];
		if (start_x > 0 && start_y > 0 && end_x > 0 && end_y > 0)
		{
			line(srcimg, Point(start_x, start_y), Point(end_x, end_y), Scalar(0, 255, 0), 2, LINE_AA);
		}
	}
	imshow("E2POSE", srcimg);
	return 1;
}