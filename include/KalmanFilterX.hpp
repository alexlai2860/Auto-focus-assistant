/**
 * @file KalmanFilterX.hpp
 * @author 杨泽霖 (scut.bigeyoung@qq.com)
 * @brief 使用轻量型 Matx 的卡尔曼滤波器
 * @details
 * 考虑到 OpenCV 中提供的 cv::KalmanFilter 是基于 cv::Mat 实现的，而
 * cv::Mat 的内存操作是运行时在堆上开辟，因此会消耗较多的时间，现对
 * 卡尔曼滤波器的功能使用 cv::Matx 进行复现，并简化了部分功能的实现，
 * 以实现更加方便的使用。
 * @version 1.0
 * @date 2021-01-11
 *
 * @copyright Copyright SCUT RobotLab(c) 2021
 *
 */

#pragma once

#include <opencv2/core.hpp>

template <uint16_t StateDim, uint16_t MeasureDim, uint16_t ControlDim = 0, typename Enabled = void>
class KalmanFilterX;

template <uint16_t StateDim, uint16_t MeasureDim, uint16_t ControlDim>
class KalmanFilterX<StateDim, MeasureDim, ControlDim, std::enable_if_t<ControlDim == 0>>
{
public:
	KalmanFilterX(float processErr = 1e-2, float measureErr = 1e-2)
	{
		this->A = this->A.eye();
		this->AT = this->A.eye();
		this->Q = this->Q.eye() * processErr;
		this->R = this->R.eye() * measureErr;
		// 默认构造假定初始状态不可知，故给予极大的协方差矩阵
		this->P = this->P.eye() * 1e7;
	}

	void init(const cv::Matx<float, StateDim, 1> &state, float error)
	{
		this->xhat = state;
		this->xhatminus = this->xhat;
		this->P = this->P.eye() * error;
		this->Pminus = this->P;
	}

	void setA(const cv::Matx<float, StateDim, StateDim> &_A)
	{
		A = _A;
		AT = A.t();
	};

	void setH(const cv::Matx<float, MeasureDim, MeasureDim> &_H)
	{
		H = _H;
		HT = H.t();
	};

	auto predict()
	{
		// 1. 预测状态
		xhatminus = A * xhat;

		// 2. 预测误差协方差
		Pminus = A * P * AT + Q;

		return xhatminus;
	}

	auto correct(const cv::Matx<float, MeasureDim, 1> &measurement)
	{
		z = measurement;

		// 3. 计算卡尔曼增益
		K = Pminus * HT * (H * Pminus * HT + R).inv();

		// 4. 用测量值更新估计
		xhat = xhatminus + K * (z - H * xhatminus);

		// 5. 更新误差协方差
		P = (Q - K * H) * Pminus;

		return xhat;
	};

	cv::Matx<float, StateDim, 1> xhat;			// 后验状态估计（当前估计值）
	cv::Matx<float, StateDim, 1> xhatminus;		// 先验状态估计（未来预测值）
	cv::Matx<float, StateDim, StateDim> Q;		// 过程噪声协方差（由环境因素引入的噪声）
	cv::Matx<float, MeasureDim, MeasureDim> R;	// 测量噪声协方差（传感器误差）
	cv::Matx<float, StateDim, StateDim> P;		// 后验误差协方差（当前估计误差）
	cv::Matx<float, StateDim, StateDim> Pminus; // 先验误差协方差（未来预测误差）
	cv::Matx<float, StateDim, MeasureDim> K;	// 卡尔曼增益
	cv::Matx<float, MeasureDim, 1> z;			// 测量值（传感器数据）

protected:
	cv::Matx<float, StateDim, StateDim> A;	  // 状态转换模型（转移矩阵）
	cv::Matx<float, MeasureDim, StateDim> H;  // 状态变量到测量值的转换矩阵（测量矩阵）
	cv::Matx<float, StateDim, StateDim> AT;	  // A.t
	cv::Matx<float, StateDim, MeasureDim> HT; // H.t
};

template <uint16_t StateDim, uint16_t MeasureDim, uint16_t ControlDim>
class KalmanFilterX<StateDim, MeasureDim, ControlDim, std::enable_if_t<ControlDim != 0>> : public KalmanFilterX<StateDim, MeasureDim, 0>
{
public:
	KalmanFilterX(float processErr = 1e-2, float measureErr = 1e-2)
		: KalmanFilterX<StateDim, MeasureDim, 0>(processErr, measureErr) {}

	std::array<float, StateDim> predict(const std::array<float, ControlDim> &control)
	{
		this->u = control;

		// 1. 预测状态
		this->xhatminus = this->A * this->xhat + B * u;

		// 2. 预测误差协方差
		this->Pminus = this->A * this->P * this->AT + this->Q;

		return this->xhatminus.getData();
	}
	cv::Matx<float, StateDim, ControlDim> B; // 控制转换模型（控制矩阵）
	cv::Matx<float, ControlDim, 1> u;		 // 控制量（控制器数据）
};

using KalmanFilter11 = KalmanFilterX<1U, 1U>;
using KalmanFilter22 = KalmanFilterX<2U, 2U>;
using KalmanFilter33 = KalmanFilterX<3U, 3U>;
using KalmanFilter44 = KalmanFilterX<4U, 4U>;
using KalmanFilter66 = KalmanFilterX<6U, 6U>;
