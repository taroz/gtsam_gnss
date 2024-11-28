/**
 * @file AngularRateFactor_VVBB.h
 * @brief Angular rate factor with 3D velocities (V) and angular rate biases (B)
 * @author Taro Suzuki
 */

#pragma once
#include <gtsam/nonlinear/NonlinearFactor.h>

namespace gtsam_gnss {

/**
 * @brief Angular rate factor: Estimate 3D velocity and angular rate bias
 */
class AngularRateFactor_VVBB : public gtsam::NoiseModelFactorN<gtsam::Vector, gtsam::Vector, gtsam::Vector, gtsam::Vector> {

private:
  double axang_;
  typedef gtsam::NoiseModelFactorN<gtsam::Vector, gtsam::Vector, gtsam::Vector, gtsam::Vector> Base;

public:
  /**
   * @brief Constructor
   * @param keyV1   3D velocity (V) key at time t1, V has 3 dimension
   * @param keyV2   3D velocity (V) key at time t2, V has 3 dimension
   * @param keyB1   Receiver clock drift (D) key at time t1, D has 1 dimension
   * @param keyB2   Receiver clock drift (D) key at time t2, D has 1 dimension
   * @param axang   Angle change computed from angular rate (1 dimension)
   */
  AngularRateFactor_VVBB(
    gtsam::Key keyV1, gtsam::Key keyV2, gtsam::Key keyB1, gtsam::Key keyB2, const double& axang, const gtsam::SharedNoiseModel& model)
  : Base(model, keyV1, keyV2, keyB1, keyB2), axang_(axang) {};

  ~AngularRateFactor_VVBB() override {}

  // Error function
  using Base::evaluateError;
  gtsam::Vector evaluateError(const gtsam::Vector& v1,
                              const gtsam::Vector& v2,
                              const gtsam::Vector& b1,
                              const gtsam::Vector& b2,
                              gtsam::OptionalMatrixType Hv1,
                              gtsam::OptionalMatrixType Hv2,
                              gtsam::OptionalMatrixType Hb1,
                              gtsam::OptionalMatrixType Hb2) const override {
    gtsam::Vector1 error;
    double v1norm = v1.norm();
    double v2norm = v2.norm();
    double v1dot = v1.squaredNorm();
    double v2dot = v2.squaredNorm();
    double v1v2dot = gtsam::dot(v1, v2);
    double vang = acos(v1v2dot / (v1norm * v2norm));
    double vx1 = v1[0];
    double vy1 = v1[1];
    double vz1 = v1[2];
    double vx2 = v2[0];
    double vy2 = v2[1];
    double vz2 = v2[2];

    // error
    error << vang - (axang_ - (b1[0] + b2[0]) / 2);

    // Jacobian
    double dvx1 =
      -(vx2 / (v1norm * v2norm) - (vx1 * v1v2dot) / (sqrt(v1dot * v1dot * v1dot) * v2norm)) / sqrt(1 - v1v2dot * v1v2dot / (v1dot * v2dot));
    double dvy1 =
      -(vy2 / (v1norm * v2norm) - (vy1 * v1v2dot) / (sqrt(v1dot * v1dot * v1dot) * v2norm)) / sqrt(1 - v1v2dot * v1v2dot / (v1dot * v2dot));
    double dvz1 =
      -(vz2 / (v1norm * v2norm) - (vz1 * v1v2dot) / (sqrt(v1dot * v1dot * v1dot) * v2norm)) / sqrt(1 - v1v2dot * v1v2dot / (v1dot * v2dot));
    if (Hv1) *Hv1 = (gtsam::Matrix(1, 3) << dvx1, dvy1, dvz1).finished();

    double dvx2 =
      -(vx1 / (v1norm * v2norm) - (vx2 * v1v2dot) / (sqrt(v2dot * v2dot * v2dot) * v1norm)) / sqrt(1 - v1v2dot * v1v2dot / (v1dot * v2dot));
    double dvy2 =
      -(vy1 / (v1norm * v2norm) - (vy2 * v1v2dot) / (sqrt(v2dot * v2dot * v2dot) * v1norm)) / sqrt(1 - v1v2dot * v1v2dot / (v1dot * v2dot));
    double dvz2 =
      -(vz1 / (v1norm * v2norm) - (vz2 * v1v2dot) / (sqrt(v2dot * v2dot * v2dot) * v1norm)) / sqrt(1 - v1v2dot * v1v2dot / (v1dot * v2dot));
    if (Hv2) *Hv2 = (gtsam::Matrix(1, 3) << dvx2, dvy2, dvz2).finished();

    if (Hb1) *Hb1 = (gtsam::Matrix(1, 1) << 0.5).finished();
    if (Hb2) *Hb2 = (gtsam::Matrix(1, 1) << 0.5).finished();

    return error;
  }

  // Print contents
  void print(const std::string& s = "", const gtsam::KeyFormatter& keyFormatter = gtsam::DefaultKeyFormatter) const override {
    std::cout << (s.empty() ? "" : s + " ") << "AngularRateFactor_VVBB" << std::endl;
    std::cout << "  axang: " << axang_ << std::endl;
    Base::print("", keyFormatter);
  }

  // Measurement
  inline const double& measurementIn() const { return axang_; }
};

}  // namespace gtsam_gnss