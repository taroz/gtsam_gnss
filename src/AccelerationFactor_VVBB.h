/**
 * @file AccelerationFactor_VVBB.h
 * @brief Acceleration factor with 3D velocities (V) and acceleration biases (B)
 * @author Taro Suzuki
 */

#pragma once
#include <gtsam/nonlinear/NonlinearFactor.h>

namespace gtsam_gnss {

/**
 * @brief Acceleration factor: Estimate 3D velocity and acceleration bias
 */
class AccelerationFactor_VVBB : public gtsam::NoiseModelFactorN<gtsam::Vector, gtsam::Vector, gtsam::Vector, gtsam::Vector> {

private:
  double g_;
  double dt_;
  double acc3d_;
  typedef gtsam::NoiseModelFactorN<gtsam::Vector, gtsam::Vector, gtsam::Vector, gtsam::Vector> Base;

public:
  /**
   * @brief Constructor
   * @param keyV1   3D velocity (V) key at time t1, V has 3 dimension
   * @param keyV2   3D velocity (V) key at time t2, V has 3 dimension
   * @param keyB1   Receiver clock drift (D) key at time t1, D has 1 dimension
   * @param keyB2   Receiver clock drift (D) key at time t2, D has 1 dimension
   * @param g       Gravity acceleration (1 dimension)
   * @param dt      Time interval (t2-t1)
   * @param acc3d   Magnitude of average acceleration between time t1 and t2 (1 dimension)
   */
  AccelerationFactor_VVBB(gtsam::Key keyV1,
                          gtsam::Key keyV2,
                          gtsam::Key keyB1,
                          gtsam::Key keyB2,
                          const double& g,
                          const double& dt,
                          const double& acc3d,
                          const gtsam::SharedNoiseModel& model)
  : Base(model, keyV1, keyV2, keyB1, keyB2), g_(g), dt_(dt), acc3d_(acc3d) {};

  ~AccelerationFactor_VVBB() override {}

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
    double vx1 = v1[0];
    double vy1 = v1[1];
    double vz1 = v1[2];
    double vx2 = v2[0];
    double vy2 = v2[1];
    double vz2 = v2[2];
    double vx12 = vx1 - vx2;
    double vy12 = vy1 - vy2;
    double vz12 = vz1 - vz2;
    double dt2 = dt_ * dt_;
    gtsam::Vector3 gvec;
    gvec << 0, 0, g_;

    // error
    gtsam::Vector accvec = (v2 - v1) / dt_ + gvec;
    error << accvec.norm() - (acc3d_ - (b1[0] + b2[0]) / 2);

    // Jacobian
    double dvx1 = (2 * vx12) / (2 * dt2 * sqrt(vx12 * vx12 / dt2 + vy12 * vy12 / dt2 + (g_ - vz12 / dt_) * (g_ - vz12 / dt_)));
    double dvy1 = (2 * vy12) / (2 * dt2 * sqrt(vx12 * vx12 / dt2 + vy12 * vy12 / dt2 + (g_ - vz12 / dt_) * (g_ - vz12 / dt_)));
    double dvz1 = -(g_ - vz12 / dt_) / (dt_ * sqrt(vx12 * vx12 / dt2 + vy12 * vy12 / dt2 + (g_ - vz12 / dt_) * (g_ - vz12 / dt_)));
    if (Hv1) *Hv1 = (gtsam::Matrix(1, 3) << dvx1, dvy1, dvz1).finished();

    double dvx2 = -(2 * vx12) / (2 * dt2 * sqrt(vx12 * vx12 / dt2 + vy12 * vy12 / dt2 + (g_ - vz12 / dt_) * (g_ - vz12 / dt_)));
    double dvy2 = -(2 * vy12) / (2 * dt2 * sqrt(vx12 * vx12 / dt2 + vy12 * vy12 / dt2 + (g_ - vz12 / dt_) * (g_ - vz12 / dt_)));
    double dvz2 = (g_ - vz12 / dt_) / (dt_ * sqrt(vx12 * vx12 / dt2 + vy12 * vy12 / dt2 + (g_ - vz12 / dt_) * (g_ - vz12 / dt_)));
    if (Hv2) *Hv2 = (gtsam::Matrix(1, 3) << dvx2, dvy2, dvz2).finished();

    if (Hb1) *Hb1 = (gtsam::Matrix(1, 1) << 0.5).finished();
    if (Hb2) *Hb2 = (gtsam::Matrix(1, 1) << 0.5).finished();

    return error;
  }

  // Print contents
  void print(const std::string& s = "", const gtsam::KeyFormatter& keyFormatter = gtsam::DefaultKeyFormatter) const override {
    std::cout << (s.empty() ? "" : s + " ") << "AccelerationFactor_VVBB" << std::endl;
    std::cout << "  acc3d: " << acc3d_ << std::endl;
    Base::print("", keyFormatter);
  }

  // Measurement
  inline const double& measurementIn() const { return acc3d_; }
};

}  // namespace gtsam_gnss