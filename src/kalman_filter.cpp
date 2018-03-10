#include "kalman_filter.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

// Please note that the Eigen library does not initialize 
// VectorXd or MatrixXd objects with zeros upon creation.

KalmanFilter::KalmanFilter() {}

KalmanFilter::~KalmanFilter() {}

void KalmanFilter::Init(VectorXd &x_in, MatrixXd &P_in, MatrixXd &F_in,
                        MatrixXd &H_in, MatrixXd &R_in, MatrixXd &Q_in) {
  x_ = x_in;
  P_ = P_in;
  F_ = F_in;
  H_ = H_in;
  R_ = R_in;
  Q_ = Q_in;
}

void KalmanFilter::Predict() {
  // Equations from lecture
  // xnew = Fx + u
  // In out case u is not required as we are not considering external factors (it is 0 mean).
  x_ = F_ * x_;

  // Pnew = F * P * Ft + Q
  MatrixXd Ft = F_.transpose();
  P_ = F_ * P_ * Ft + Q_;
}

void KalmanFilter::Update(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Kalman Filter equations
  */
  VectorXd z_pred = H_ * x_;
  VectorXd y = z - z_pred;
  MatrixXd PHt = P_ * H_.transpose();
  MatrixXd S = H_ * PHt + R_;
  MatrixXd K = PHt * S.inverse();

  //new estimate
  x_ = x_ + (K * y);
  int x_size = x_.size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size);
  P_ = (I - K * H_) * P_;
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Extended Kalman Filter equations
  */
  // Need to convert the state into polar coordinates

  float px = x_(0);
  float py = x_(1);
  float vx = x_(2);
  float vy = x_(3);

  float rho = sqrt(px * px + py * py);
  float phi = atan2(py, px);
  float rho_dot;

  // Ensure that we are not dividing by zerio
  if (rho > 0.0001 ) {
      rho_dot = (px * vx + py * vy) / rho;
  } else {
      rho_dot = (px * vx + py * vy) / 0.0001;
  }
  VectorXd z_pred(3);
  z_pred << rho, phi, rho_dot;

  VectorXd y = z - z_pred;
  long measured_phi = y(1);
  long two_pi = 2 * M_PI;
  long to_add = (measured_phi < 0 ? two_pi : -two_pi);

  while (fabs(measured_phi) > M_PI) {
    measured_phi += to_add;
  }
  y(1) = measured_phi;

  MatrixXd Ht = H_.transpose();
  MatrixXd S = H_ * P_ * Ht + R_;
  MatrixXd Si = S.inverse();
  MatrixXd PHt = P_ * Ht;
  MatrixXd K = PHt * Si;

  //new estimate
  x_ = x_ + (K * y);
  long x_size = x_.size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size);
  P_ = (I - K * H_) * P_;
}
