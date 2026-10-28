#pragma once
// MeasurementModel: per-sensor object that converts between sensor measurements
// and world-frame positions. Measurement itself stays a pure data struct.
//
// The concrete maths lives in a library of function sets (one per MeasType).
// A MeasurementModel picks its function set once, at construction, from the
// sensor's meas_type, so callers never branch on the measurement type.

#include <Eigen/Dense>
#include "sensors/Measurement.hpp"  // Measurement, MeasType

namespace sfsim {

// One entry per measurement type. Plain function pointers: no allocation,
// trivially copyable, and a model can be rebuilt cheaply.
struct FunctionSet {
    int dim;  // measurement dimension

    // z (sensor measurement)  ->  world position (3)
    Vec3 (*meas2world)(const Vec3& p, const Eigen::Quaterniond& q, const Vec3& z);
    // world position (3)      ->  z (predicted measurement)
    Vec3 (*world2meas)(const Vec3& p, const Eigen::Quaterniond& q, const Vec3& z_world);
    // d(world position)/dz   (3 x dim)  e.g. for mapping R into the world frame
    Mat3 (*meas2worldJacobian)(const Vec3& p, const Eigen::Quaterniond& q, const Vec3& z);
    // dz/d(world position)   (dim x 3)  e.g. the position block of the EKF H
    Mat3 (*world2measJacobian)(const Vec3& p, const Eigen::Quaterniond& q, const Vec3& z_world);
    // z - zPred with any angular components wrapped
    Vec3 (*residual)(const Vec3& z, const Vec3& zPred);
};

// Library lookup. Throws std::invalid_argument for an unregistered MeasType.
const FunctionSet& getFnSet(MeasType type);

class MeasurementModel {
public:
    // Mandatory: a model cannot exist without a type.
    MeasurementModel(MeasType type);
    MeasurementModel() = delete;

    MeasType type() const { return type_; }
    int dim() const { return fns_->dim; }

    Vec3 Meas2World(const Vec3& p, const Eigen::Quaterniond& q, const Vec3& z) const { 
        return fns_->meas2world(p, q, z); }
    Vec3 World2Meas(const Vec3& p, const Eigen::Quaterniond& q, const Vec3& z_world) const { 
        return fns_->world2meas(p, q, z_world); }
    Mat3 Meas2WorldJacobian(const Vec3& p, const Eigen::Quaterniond& q, const Vec3& z) const {
        return fns_->meas2worldJacobian(p, q, z);
    }
    Mat3 World2MeasJacobian(const Vec3& p, const Eigen::Quaterniond& q, const Vec3& z_world) const {
        return fns_->world2measJacobian(p, q, z_world);
    }
    Vec3 Residual(const Vec3& z, const Vec3& zPred) const {
        return fns_->residual(z, zPred);
    }

private:
    MeasType type_;
    const FunctionSet* fns_;  // points into the static library table
};

}  // namespace sfsim
