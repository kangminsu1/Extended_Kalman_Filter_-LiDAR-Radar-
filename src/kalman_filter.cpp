#include "kalman_filter.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;


KalmanFilter::KalmanFilter() = default; // 생성자 (객체 생성시 호출)

KalmanFilter::~KalmanFilter() = default; // 소멸자 (객체 소멸시 호출, 메모리 해제)

MatrixXd KalmanFilter::I_ = MatrixXd::Identity(4, 4);
Tools KalmanFilter::tools_ = Tools();

// 초기 설정 
void KalmanFilter::Init(VectorXd &x_in,       MatrixXd &P_in,       MatrixXd &F_in, MatrixXd &H_in,
		MatrixXd &R_lidar_in, MatrixXd &R_radar_in, MatrixXd &Q_in) {
	x_       = x_in;
	P_       = P_in;
	F_       = F_in;
	H_       = H_in;
	R_lidar_ = R_lidar_in;
	R_radar_ = R_radar_in;
	Q_       = Q_in;
}

// 센서에서 수신된 State Vector 예측 부
void KalmanFilter::Predict() {
	x_ = F_ * x_;
	MatrixXd Ft = F_.transpose();
	P_ = F_ * P_ * Ft + Q_;
}

// 센서에서 수신된 State Vector 업데이트 부
void KalmanFilter::Update(const VectorXd &z) {
	UpdateCommon(z, H_, R_lidar_);
}

// 업데이트 부 (상세)
void KalmanFilter::UpdateCommon(const VectorXd &z, const MatrixXd &H, const MatrixXd &R) {
	VectorXd y; 
	
	y = z - H * x_; // 측정치 - (H + State vector)

	MatrixXd Ht = H.transpose(); // Residual Data
	MatrixXd S = H * P_ * Ht + R; // HPH^T + 시스템 노이즈
	MatrixXd Si = S.inverse(); // Residual Data (Cov)
	MatrixXd K =  P_ * Ht * Si; // Kalman Gain

	// new state (t + 1)
	x_ = x_ + (K * y);
	P_ = (I_ - K * H) * P_;
}
