/**
 * @file SC_PseudorangeFactor_X.h
 * @brief Pseudorange factor with 3D position (X) and Switchable Constraint (S)
 * @note This factor is applied to range observations that do not include receiver clock errors, such as DD pseudorange
 * @author Taro Suzuki
 */

#pragma once
#include <gtsam/nonlinear/NonlinearFactor.h>
#include "SwitchVariable.h"

namespace gtsam_gnss {

/**
 * @brief Pseudorange factor: Estimate 3D position
 */
class SC_PseudorangeFactor_X : public gtsam::NoiseModelFactorN<gtsam::Vector, SwitchVariable> {

private:
  gtsam::Vector losvec_;
  double pr_;
  gtsam::Vector inix_;
  typedef gtsam::NoiseModelFactorN<gtsam::Vector, SwitchVariable> Base;

public:
  /**
   * @brief Constructor
   * @param keyX    3D position (X) key, X has 3 dimension
   * @param keyS    Switchable Constraint (S) key, S has 1 dimension
   * @param losvec  Line-of-Sight vector (3 dimension)
   * @param pr      Pseudorange residual at initial 3D position (1 dimension)
   * @param inix    Initial 3D position when calculating residual (3 dimension)
   * @param model   Gaussian noise model (1 dimension)
   */
    SC_PseudorangeFactor_X(gtsam::Key keyX,
                        gtsam::Key keyS,
                        const gtsam::Vector& losvec,
                        const double& pr,
                        const gtsam::Vector& inix,
                        const gtsam::SharedNoiseModel& model)
  : Base(model, keyX, keyS), losvec_(losvec), pr_(pr), inix_(inix) {};

  ~SC_PseudorangeFactor_X() override {}

  // Error function
  using Base::evaluateError;
  gtsam::Vector evaluateError(const gtsam::Vector& x,
                              const SwitchVariable& s,
                              gtsam::OptionalMatrixType Hx,
                              gtsam::OptionalMatrixType Hs) const override {
    // Compute error
    gtsam::Vector1 error;
    error << (losvec_.transpose() * (x - inix_)).value() - pr_;

    // Jacobian
    if (Hx) *Hx = (gtsam::Matrix(1, 3) << s.value()*losvec_.transpose()).finished();
    if (Hs) *Hs = error;

    return s.value()*error;
  }

  // Print contents
  void print(const std::string& str = "", const gtsam::KeyFormatter& keyFormatter = gtsam::DefaultKeyFormatter) const override {
    std::cout << (str.empty() ? "" : str + " ") << "SC_PseudorangeFactor_X" << std::endl;
    std::cout << "  pseudorange residual: " << pr_ << std::endl;
    Base::print("", keyFormatter);
  }

  // Measurement
  inline const double& measurementIn() const { return pr_; }
};

}  // namespace gtsam_gnss
