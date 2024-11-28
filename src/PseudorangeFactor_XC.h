/**
 * @file PseudorangeFactor_XC.h
 * @brief Pseudorange factor with 3D position (X) and receiver clock (C)
 * @author Taro Suzuki
 */

#pragma once
#include <gtsam/nonlinear/NonlinearFactor.h>

namespace gtsam_gnss {

/**
 * @brief Pseudorange factor: Estimate 3D position and receiver clock and inter system bias (ISB)
 */
class PseudorangeFactor_XC : public gtsam::NoiseModelFactorN<gtsam::Vector, gtsam::Vector> {

private:
  gtsam::Vector losvec_;
  double pr_;
  int sysidx_;
  gtsam::Vector inix_;
  typedef gtsam::NoiseModelFactorN<gtsam::Vector, gtsam::Vector> Base;

public:
  /**
   * @brief Constructor
   * @param keyX    3D position (X) key, X has 3 dimension
   * @param keyC    Receiver clock (C) key, C has 7 dimension
   * @param losvec  Line-of-Sight vector (3 dimension)
   * @param pr      Pseudorange residual at initial 3D position (1 dimension)
   * @param sysidx  GNSS system index for estimating receiver clock and ISB, starting from 0 up to 6 (e.g. GPS:0, GLONASS:1, Galileo:2)
   * @param inix    Initial 3D position when calculating residual (3 dimension)
   * @param model   Gaussian noise model (3 dimension)
   */
  PseudorangeFactor_XC(gtsam::Key keyX,
                       gtsam::Key keyC,
                       const gtsam::Vector& losvec,
                       const double& pr,
                       const int& sysidx,
                       const gtsam::Vector& inix,
                       const gtsam::SharedNoiseModel& model)
  : Base(model, keyX, keyC), losvec_(losvec), pr_(pr), sysidx_(sysidx), inix_(inix) {};

  ~PseudorangeFactor_XC() override {}

  // Error function
  using Base::evaluateError;
  gtsam::Vector evaluateError(const gtsam::Vector& x,
                              const gtsam::Vector& c,
                              gtsam::OptionalMatrixType Hx,
                              gtsam::OptionalMatrixType Hc) const override {
    // Estimate receiver clock + inter system bias(ISB)
    gtsam::Vector7 hc;
    hc << 1, 0, 0, 0, 0, 0, 0;
    hc(sysidx_) = 1;

    // Compute error
    gtsam::Vector1 error;
    error << (losvec_.transpose() * (x - inix_) + hc.transpose() * c).value() - pr_;

    // Jacobian
    if (Hx) *Hx = (gtsam::Matrix(1, 3) << losvec_.transpose()).finished();
    if (Hc) *Hc = (gtsam::Matrix(1, 7) << hc.transpose()).finished();

    return error;
  }

  // Print contents
  void print(const std::string& s = "", const gtsam::KeyFormatter& keyFormatter = gtsam::DefaultKeyFormatter) const override {
    std::cout << (s.empty() ? "" : s + " ") << "PseudorangeFactor_XC" << std::endl;
    std::cout << "  pseudorange residual: " << pr_ << ", system index: " << sysidx_ << std::endl;
    Base::print("", keyFormatter);
  }

  // Measurement
  inline const double& measurementIn() const { return pr_; }
};

}  // namespace gtsam_gnss
