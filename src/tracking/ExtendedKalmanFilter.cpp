#include "tracking/ExtendedKalmanFilter.hpp"

#include <cassert>

#include "sensors/Sensor.hpp"

namespace sfsim {

namespace { // Private helpers

using Mat36 = Eigen::Matrix<double, 3, 6>;

// Catches the easy mistake of pairing a measurement with the wrong sensor.
inline void checkPairing(const Measurement& m, const Sensor& sensor) {
  assert(m.sensor == sensor.config().id && "measurement paired with the wrong sensor");
  assert(m.type == sensor.model().type() && "measurement type does not match sensor model");
  (void)m;
  (void)sensor;
}

// Predicted measurement h(x), in the sensor's measurement space.
// Uses the pose stored on the measurement (what the sensor reported at measurement time).
Vec3 h(const Vec6& state, const Measurement& m, const Sensor& sensor) {
  return sensor.model().World2Meas(m.pose.position, m.pose.orientation, state.head<3>());
}

// Measurement Jacobian dh/dx. The model gives d(meas)/d(world position);
// the velocity block is zero because none of the models observe velocity.
Mat36 H(const Vec6& state, const Measurement& m, const Sensor& sensor) {
  Mat36 H_mat = Mat36::Zero();
  H_mat.leftCols<3>() =
      sensor.model().World2MeasJacobian(m.pose.position, m.pose.orientation, state.head<3>());
  return H_mat;
}

// z - h(x), with angular components wrapped by the model's own residual function.
Vec3 innovation(const Vec6& state, const Measurement& m, const Sensor& sensor) {
  return sensor.model().Residual(m.z, h(state, m, sensor));
}

}  // namespace

void EKF::predict(double dt, double q) {
  Mat6 F = Mat6::Identity();
  F.topRightCorner<3, 3>() = Mat3::Identity() * dt;
  // Discrete white-noise acceleration model
  Mat6 Q = Mat6::Zero();
  const double dt2 = dt * dt, dt3 = dt2 * dt, dt4 = dt3 * dt;
  Q.topLeftCorner<3, 3>() = Mat3::Identity() * (dt4 / 4.0 * q);
  Q.topRightCorner<3, 3>() = Mat3::Identity() * (dt3 / 2.0 * q);
  Q.bottomLeftCorner<3, 3>() = Mat3::Identity() * (dt3 / 2.0 * q);
  Q.bottomRightCorner<3, 3>() = Mat3::Identity() * (dt2 * q);
  x = F * x;
  P = F * P * F.transpose() + Q;
}

double EKF::gateDistance(const Measurement& m, const Sensor& sensor) const {
  checkPairing(m, sensor);
  const Mat36 H_jac = H(x, m, sensor);
  const Vec3 y = innovation(x, m, sensor);
  const Mat3 S = H_jac * P * H_jac.transpose() + m.R;
  return y.dot(S.ldlt().solve(y));
}

void EKF::update(const Measurement& m, const Sensor& sensor) {
  checkPairing(m, sensor);
  const Mat36 H_jac = H(x, m, sensor);
  const Vec3 y = innovation(x, m, sensor);
  const Mat3 S = H_jac * P * H_jac.transpose() + m.R;
  const Eigen::Matrix<double, 6, 3> K = P * H_jac.transpose() * S.ldlt().solve(Mat3::Identity());
  x += K * y;
  const Mat6 I_KH = Mat6::Identity() - K * H_jac;
  P = I_KH * P * I_KH.transpose() + K * m.R * K.transpose();  // Joseph form
}

}  // namespace sfsim
