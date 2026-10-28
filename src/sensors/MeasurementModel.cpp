#include "sensors/MeasurementModel.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

#include "core/maths.hpp"  // wrapAngle

namespace sfsim {

inline Mat3 rot(const Eigen::Quaterniond& q) {
    return q.toRotationMatrix();
}

namespace {

constexpr double kEps = 1e-9;

// ---------------------------------------------------------------- Cartesian
// z = position of the target in the sensor frame (x, y, z).
// With an identity pose this is just world position plus the sensor offset.

Vec3 cartMeas2World(const Vec3& p, const Eigen::Quaterniond& q, const Vec3& z) {
    return p + rot(q) * z.head<3>();
}

Vec3 cartWorld2Meas(const Vec3& p, const Eigen::Quaterniond& q, const Vec3& z_world) {
    return rot(q).transpose() * (z_world - p);
}

Mat3 cartMeas2WorldJac(const Vec3& p, const Eigen::Quaterniond& q, const Vec3& z) { return rot(q); }

Mat3 cartWorld2MeasJac(const Vec3& p, const Eigen::Quaterniond& q, const Vec3& z_world) { return rot(q).transpose(); }

Vec3 cartResidual(const Vec3& z, const Vec3& zPred) { return z - zPred; }

// --------------------------------------------------------------- RangeAzEl
// z = [range, azimuth, elevation] in the sensor frame.
//   x_s = r cos(el) cos(az)
//   y_s = r cos(el) sin(az)
//   z_s = r sin(el)

Vec3 raeMeas2World(const Vec3& p, const Eigen::Quaterniond& q, const Vec3& z) {
    const double r = z(0), az = z(1), el = z(2);
    Eigen::Vector3d p_s(r * std::cos(el) * std::cos(az),
                        r * std::cos(el) * std::sin(az),
                        r * std::sin(el));
    return p + rot(q) * p_s;
}

Vec3 raeWorld2Meas(const Vec3& p, const Eigen::Quaterniond& q, const Vec3& z_world) {
    const Eigen::Vector3d d = rot(q).transpose() * (z_world - p);
    const double rho = std::hypot(d.x(), d.y());
    Vec3 z;
    z(0) = d.norm();
    z(1) = std::atan2(d.y(), d.x());
    z(2) = std::atan2(d.z(), rho);
    return z;
}

// d(world)/d(r, az, el) = R_ws * d(p_s)/d(r, az, el)
Mat3 raeMeas2WorldJac(const Vec3& p, const Eigen::Quaterniond& q, const Vec3& z) {
    const double r = z(0), az = z(1), el = z(2);
    const double ca = std::cos(az), sa = std::sin(az);
    const double ce = std::cos(el), se = std::sin(el);

    Mat3 J;
    J << ce * ca, -r * ce * sa, -r * se * ca,
         ce * sa,  r * ce * ca, -r * se * sa,
         se,       0.0,          r * ce;
    return rot(q) * J;
}

// d(r, az, el)/d(world) = d(r, az, el)/d(p_s) * R_ws^T
Mat3 raeWorld2MeasJac(const Vec3& p, const Eigen::Quaterniond& q, const Vec3& z_world) {
    const Eigen::Vector3d d = rot(q).transpose() * (z_world - p);
    const double x = d.x(), y = d.y(), z = d.z();
    const double rho2 = std::max(x * x + y * y, kEps);
    const double rho = std::sqrt(rho2);
    const double r2 = rho2 + z * z;
    const double r = std::sqrt(r2);

    Mat3 J;
    J << x / r,            y / r,            z / r,
        -y / rho2,         x / rho2,         0.0,
        -x * z / (r2 * rho), -y * z / (r2 * rho), rho / r2;
    return J * rot(q).transpose();
}

Vec3 raeResidual(const Vec3& z, const Vec3& zPred) {
    Vec3 e = z - zPred;
    e(1) = wrapAngle(e(1));  // azimuth wraps; elevation stays in [-pi/2, pi/2]
    return e;
}

// ------------------------------------------------------------------ Library
// To add a measurement type: write the five functions above, add a MeasType
// enumerator, and add one row here.

const FunctionSet kCartesian = {
    3, cartMeas2World, cartWorld2Meas, cartMeas2WorldJac, cartWorld2MeasJac, cartResidual};

const FunctionSet kRangeAzEl = {
    3, raeMeas2World, raeWorld2Meas, raeMeas2WorldJac, raeWorld2MeasJac, raeResidual};

}  // namespace

const FunctionSet& getFnSet(MeasType type) {
    switch (type) {
        case MeasType::Cartesian: return kCartesian;
        case MeasType::RangeAzEl: return kRangeAzEl;
    }
    throw std::invalid_argument("getFnSet: unregistered MeasType");
}

MeasurementModel::MeasurementModel(MeasType type)
    : type_(type), fns_(&getFnSet(type)) {}

}  // namespace sfsim