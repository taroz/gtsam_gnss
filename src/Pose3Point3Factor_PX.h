/**
 * @file Pose3Point3Factor_PX.h
 * @brief Pose3Point3 factor with 3D pose (P) and 3D position (X)
 * @note This factor is used to relate 3D pose and 3D position
 * @author Taro Suzuki
 */

#pragma once
#include <gtsam/geometry/Pose3.h>
#include <gtsam/nonlinear/NonlinearFactor.h>

namespace gtsam_gnss {

/**
 * @brief Pose3Point3 factor: Relate 3D pose and 3D position
 */
class Pose3Point3Factor_PX : public gtsam::NoiseModelFactorN<gtsam::Pose3, gtsam::Vector> {

private:
  typedef gtsam::NoiseModelFactorN<gtsam::Pose3, gtsam::Vector> Base;

public:
  /**
   * @brief Constructor
   * @param keyP   3D pose (P), P is gtsam::Pose3
   * @param keyX   3D position (X), X has 3 dimension
   * @param model  Gaussian noise model (3 dimension)
   */
  Pose3Point3Factor_PX(gtsam::Key keyP, gtsam::Key keyX, const gtsam::SharedNoiseModel& model) : Base(model, keyP, keyX) {};

  ~Pose3Point3Factor_PX() override {}

  // Error function
  using Base::evaluateError;
  gtsam::Vector evaluateError(const gtsam::Pose3& p,
                              const gtsam::Vector& x,
                              gtsam::OptionalMatrixType Hp,
                              gtsam::OptionalMatrixType Hx) const override {
    // Compute error
    gtsam::Matrix Hpose_P;
    gtsam::Vector3 error = p.translation(Hpose_P) - x;

    // Jacobian
    if (Hp) *Hp = Hpose_P;
    if (Hx) *Hx = -gtsam::I_3x3;

    return error;
  }

  // Print contents
  void print(const std::string& s = "", const gtsam::KeyFormatter& keyFormatter = gtsam::DefaultKeyFormatter) const override {
    std::cout << (s.empty() ? "" : s + " ") << "Pose3Point3Factor_PX" << std::endl;
    Base::print("", keyFormatter);
  }
};

}  // namespace gtsam_gnss
