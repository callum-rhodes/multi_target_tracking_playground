// Framework-free tests: returns non-zero on failure. Hook into CTest with add_test().
#include "sensors/MeasurementModels.hpp"
#include "core/maths.hpp"
#include <cmath>
#include <cstdio>
#include <cstdlib>

using namespace sfsim;

static int failures = 0;
#define CHECK(cond)                                                      \
  do {                                                                   \
    if (!(cond)) { std::printf("FAIL %s:%d  %s\n", __FILE__, __LINE__, #cond); ++failures; } \
  } while (0)

static SensorPose makePose(const Vec3& pos, double yaw, double pitch, double roll) {
  SensorPose p;
  p.position = pos;
  p.orientation = Eigen::AngleAxisd(yaw, Vec3::UnitZ()) *
                  Eigen::AngleAxisd(pitch, Vec3::UnitY()) *
                  Eigen::AngleAxisd(roll, Vec3::UnitX());
  return p;
}

int main() {
  const SensorPose poses[] = {
      makePose({0, 0, 0}, 0, 0, 0),
      makePose({100, -50, 10}, 0.7, 0.2, -0.3),
      makePose({-20, 30, 5}, 3.0, -0.4, 1.0),  // yaw near +-pi exercises wrapping
  };
  const Vec3 targets[] = {{500, 300, 120}, {-400, 250, -80}, {50, -700, 300}};

  for (const auto& pose : poses) {
    for (const Vec3& p : targets) {
      const Vec3 z = rae::fromCartesian(p, pose);

      // 1. round trip
      CHECK((rae::toCartesian(z, pose) - p).norm() < 1e-9);

      // 2. the two Jacobians are inverses
      const Mat3 prod = rae::toCartesianJacobian(z, pose) * rae::fromCartesianJacobian(p, pose);
      CHECK((prod - Mat3::Identity()).norm() < 1e-9);

      // 3. finite-difference check of fromCartesianJacobian
      const Mat3 J = rae::fromCartesianJacobian(p, pose);
      Mat3 Jn;
      const double eps = 1e-5;
      for (int k = 0; k < 3; ++k) {
        Vec3 dp = Vec3::Zero();
        dp(k) = eps;
        Vec3 d = rae::fromCartesian(p + dp, pose) - rae::fromCartesian(p - dp, pose);
        d(1) = wrapAngle(d(1));
        Jn.col(k) = d / (2 * eps);
      }
      CHECK((J - Jn).norm() < 1e-6);

      // 4. dispatch layer: H has zero velocity columns and matches h
      Vec6 x = Vec6::Zero();
      x.head<3>() = p;
      Measurement m;
      m.type = MeasType::RangeAzEl;
      m.pose = pose;
      m.z = z;
      m.R = Mat3::Identity();
      CHECK(residual(x, m).norm() < 1e-12);
      CHECK(measurementJacobian(x, m).rightCols<3>().norm() == 0.0);
    }
  }

  // 5. azimuth residual wraps across +-pi
  {
    Vec6 x = Vec6::Zero();
    x << -100, -1e-3, 0, 0, 0, 0;  // just below the -x axis, az ~ -pi
    Measurement m;
    m.type = MeasType::RangeAzEl;
    m.pose = makePose({0, 0, 0}, 0, 0, 0);
    m.z = Vec3(100.0, M_PI - 1e-5, 0.0);  // measured just on the other side
    CHECK(std::abs(residual(x, m)(1)) < 1e-3);
  }

  // 6. singularity: target straight above the sensor
  {
    const SensorPose pose = makePose({0, 0, 0}, 0, 0, 0);
    const Vec3 above(0, 0, 100);
    const Mat3 J = rae::fromCartesianJacobian(above, pose);
    CHECK(J.allFinite());
    CHECK(rae::nearSingularity(above, pose));
    CHECK(!rae::nearSingularity(Vec3(100, 0, 0), pose));
  }

  // 7. Cartesian type is identity
  {
    Vec6 x = Vec6::Zero();
    x << 1, 2, 3, 4, 5, 6;
    Measurement m;
    m.type = MeasType::Cartesian;
    m.z = Vec3(1, 2, 3);
    m.R = Mat3::Identity();
    CHECK(residual(x, m).norm() < 1e-12);
    const auto H = measurementJacobian(x, m);
    CHECK(H.leftCols<3>().isApprox(Mat3::Identity()));
  }

  std::printf(failures ? "%d check(s) failed\n" : "all measurement model tests passed\n", failures);
  return failures ? 1 : 0;
}
