#include "kalman_filter.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::max;

/*
 * Please note that the Eigen library does not initialize
 *   VectorXd or MatrixXd objects with zeros upon creation.
 */

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
  /**
   * TODO: predict the state
   */
  x_ = F_ * x_; //update the coordinates and velocities with the transition matrix
  MatrixXd Ft = F_.transpose();
  P_ = F_ * P_ * Ft + Q_; // update the state uncertainty
}

// Kalman update steps common to laser and radar
void KalmanFilter::SharedUpdate(const VectorXd &y) {
  MatrixXd Ht = H_.transpose();
  MatrixXd S = H_ * P_ * Ht + R_;
  MatrixXd Si = S.inverse();
  MatrixXd PHt = P_ * Ht;
  MatrixXd K = PHt * Si; //Kalman gain

  //new estimate
  x_ = x_ + (K * y); //update the coordinates and velocities with the Kalman gain and the difference between the measurement and predicted measurement
  long x_size = x_.size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size);
  P_ = (I - K * H_) * P_;
}

void KalmanFilter::Update(const VectorXd &z) {
  /**
   * TODO: update the state by using Kalman Filter equations
   */
  VectorXd z_pred = H_ * x_; // predicted measurment
  VectorXd y = z - z_pred; //difference between the measurement and predicted measurement
  SharedUpdate(y);
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /**
   * TODO: update the state by using Extended Kalman Filter equations
   */

  double px = x_[0];
  double py = x_[1];
  double vx = x_[2];
  double vy = x_[3];
  double px2 = px*px;
  double py2 = py*py;

  double ro = sqrt(px2+py2);

  // avoid zero division for ro_dot calculation
  if (fabs(ro) < 0.0001) ro=0.0001;
  double phi = atan2(py,px);
  double ro_dot = (px*vx + py*vy)/ro;

  VectorXd z_pred = VectorXd(3);
  z_pred << ro, phi, ro_dot;

  VectorXd y = z - z_pred;

  // adjust angle value so it lays between -Pi and Pi
  while (y(1)> M_PI) y(1)-=2.*M_PI;
  while (y(1)<-M_PI) y(1)+=2.*M_PI;

  SharedUpdate(y);
}
