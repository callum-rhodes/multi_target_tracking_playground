#pragma once
#include <Eigen/Dense>
#include <cstdint>

namespace sfsim {
using Vec3 = Eigen::Vector3d;
using Vec6 = Eigen::Matrix<double, 6, 1>;
using Mat3 = Eigen::Matrix3d;
using Mat6 = Eigen::Matrix<double, 6, 6>;
using TargetId = uint32_t;
using SensorId = uint32_t;
using TrackerId = uint32_t;

struct Pose {            // Generic pose in 3D space
    Vec3 position = Vec3::Zero();
    Eigen::Quaterniond orientation = Eigen::Quaterniond::Identity();  // boresight = +x in sensor frame
  };
}  // namespace sfsim
