/**
 * @file MotionFactor_XXVV.h
 * @brief Motion factor with 3D positions (X) and 3D velocities (V)
 * @note This factor is used to relate 3D position and 3D velocity
 * @author Taro Suzuki
 */

#pragma once
#include <gtsam/nonlinear/NonlinearFactor.h>

namespace gtsam_gnss {

/**
 * @brief Motion factor: Relate 3D position and 3D velocity
 */
class MotionFactor_XXVV : public gtsam::NoiseModelFactorN<gtsam::Vector, gtsam::Vector, gtsam::Vector, gtsam::Vector> {

private:
  double dt_;
  typedef gtsam::NoiseModelFactorN<gtsam::Vector, gtsam::Vector, gtsam::Vector, gtsam::Vector> Base;

public:
  /**
   * @brief Constructor
   * @param keyX1   3D position (X) key at time t1, X has 3 dimension
   * @param keyX2   3D position (X) key at time t2, X has 3 dimension
   * @param keyV1   3D velocity (V) key at time t1, V has 3 dimension
   * @param keyV2   3D velocity (V) key at time t2, V has 3 dimension
   * @param dt      Time interval (t2-t1)
   * @param model   Gaussian noise model (3 dimension)
   */
  MotionFactor_XXVV(gtsam::Key keyX1,
                    gtsam::Key keyX2,
                    gtsam::Key keyV1,
                    gtsam::Key keyV2,
                    const double& dt,
                    const gtsam::SharedNoiseModel& model)
  : Base(model, keyX1, keyX2, keyV1, keyV2), dt_(dt) {};

  ~MotionFactor_XXVV() override {}

  // Error function
  using Base::evaluateError;
  gtsam::Vector evaluateError(const gtsam::Vector& x1,
                              const gtsam::Vector& x2,
                              const gtsam::Vector& v1,
                              const gtsam::Vector& v2,
                              gtsam::OptionalMatrixType Hx1,
                              gtsam::OptionalMatrixType Hx2,
                              gtsam::OptionalMatrixType Hv1,
                              gtsam::OptionalMatrixType Hv2) const override {
    // Compute error
    gtsam::Vector3 error;
    error << (x2 - x1) - (v1 + v2) * dt_ / 2;

    // Jacobian
    if (Hx1) *Hx1 = -gtsam::I_3x3;
    if (Hx2) *Hx2 =  gtsam::I_3x3;
    if (Hv1) *Hv1 = -dt_ / 2 * gtsam::I_3x3;
    if (Hv2) *Hv2 = -dt_ / 2 * gtsam::I_3x3;

    return error;
  }

  // Print contents
  void print(const std::string& s = "", const gtsam::KeyFormatter& keyFormatter = gtsam::DefaultKeyFormatter) const override {
    std::cout << (s.empty() ? "" : s + " ") << "MotionFactor_XXVV" << std::endl;
    std::cout << "  dt: " << dt_ << std::endl;
    Base::print("", keyFormatter);
  }
};

}  // namespace gtsam_gnss
