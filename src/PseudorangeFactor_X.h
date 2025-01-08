/**
 * @file PseudorangeFactor_X.h
 * @brief Pseudorange factor with 3D position (X)
 * @note This factor is applied to range observations that do not include receiver clock errors, such as DD pseudorange
 * @author Taro Suzuki
 */

#pragma once
#include <gtsam/nonlinear/NonlinearFactor.h>

namespace gtsam_gnss {

/**
 * @brief Pseudorange factor: Estimate 3D position
 */
class PseudorangeFactor_X : public gtsam::NoiseModelFactorN<gtsam::Vector> {

private:
  gtsam::Vector losvec_;
  double pr_;
  gtsam::Vector inix_;
  typedef gtsam::NoiseModelFactorN<gtsam::Vector> Base;

public:
  /**
   * @brief Constructor
   * @param keyX    3D position (X) key, X has 3 dimension
   * @param losvec  Line-of-Sight vector (3 dimension)
   * @param pr      Pseudorange residual at initial 3D position (1 dimension)
   * @param inix    Initial 3D position when calculating residual (3 dimension)
   * @param model   Gaussian noise model (1 dimension)
   */
    PseudorangeFactor_X(gtsam::Key keyX,
                        const gtsam::Vector& losvec,
                        double& pr,
                        const gtsam::Vector& inix,
                        const gtsam::SharedNoiseModel& model)
  : Base(model, keyX), losvec_(losvec), pr_(pr), inix_(inix) {};

  ~PseudorangeFactor_X() override {}

  // Error function
  using Base::evaluateError;
  gtsam::Vector evaluateError(const gtsam::Vector& x, gtsam::OptionalMatrixType Hx) const override {
    // Compute error
    gtsam::Vector1 error;
    error << (losvec_.transpose() * (x - inix_)).value() - pr_;

    // Jacobian
    if (Hx) *Hx = (gtsam::Matrix(1, 3) << losvec_.transpose()).finished();

    return error;
  }

  // Print contents
  void print(const std::string& s = "", const gtsam::KeyFormatter& keyFormatter = gtsam::DefaultKeyFormatter) const override {
    std::cout << (s.empty() ? "" : s + " ") << "PseudorangeFactor_X" << std::endl;
    std::cout << "  pseudorange residual: " << pr_ << std::endl;
    Base::print("", keyFormatter);
  }

  // Measurement
  inline const double& measurementIn() const { return pr_; }
};

}  // namespace gtsam_gnss
