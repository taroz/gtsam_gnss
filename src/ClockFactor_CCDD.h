/**
 * @file ClockFactor_CCDD.h
 * @brief Clock factor with receiver clocks (C) and receiver clock drifts (D)
 * @note This factor is used to relate receiver clock and receiver clock drift
 * @author Taro Suzuki
 */

#pragma once
#include <gtsam/nonlinear/NonlinearFactor.h>

namespace gtsam_gnss {

/**
 * @brief Clock factor: Relate receiver clock and receiver clock drift
 */
class ClockFactor_CCDD : public gtsam::NoiseModelFactorN<gtsam::Vector, gtsam::Vector, gtsam::Vector, gtsam::Vector> {

private:
  double dt_;
  typedef gtsam::NoiseModelFactorN<gtsam::Vector, gtsam::Vector, gtsam::Vector, gtsam::Vector> Base;

public:
  /**
   * @brief Constructor
   * @param keyC1   Receiver clock (C) key at time t1, C has 7 dimension
   * @param keyC2   Receiver clock (C) key at time t2, C has 7 dimension
   * @param keyD1   Receiver clock drift (D) key at time t1, D has 1 dimension
   * @param keyD2   Receiver clock drift (D) key at time t2, D has 1 dimension
   * @param dt      Time interval (t2-t1)
   * @param model   Gaussian noise model (3 dimension)
   */
  ClockFactor_CCDD(gtsam::Key keyC1, gtsam::Key keyC2, gtsam::Key keyD1, gtsam::Key keyD2, const double& dt, const gtsam::SharedNoiseModel& model)
  : Base(model, keyC1, keyC2, keyD1, keyD2), dt_(dt) {};

  ~ClockFactor_CCDD() override {}

  // Error function
  using Base::evaluateError;
  gtsam::Vector evaluateError(const gtsam::Vector& c1,
                              const gtsam::Vector& c2,
                              const gtsam::Vector& d1,
                              const gtsam::Vector& d2,
                              gtsam::OptionalMatrixType Hc1,
                              gtsam::OptionalMatrixType Hc2,
                              gtsam::OptionalMatrixType Hd1,
                              gtsam::OptionalMatrixType Hd2) const override {
    // Compute error
    gtsam::Vector1 error;
    error << (c2[0] - c1[0]) - ((d1[0] + d2[0]) * dt_ / 2);

    // Jacobian
    if (Hc1) *Hc1 = (gtsam::Matrix(1, 7) << 1, 0, 0, 0, 0, 0, 0).finished();
    if (Hc2) *Hc2 = (gtsam::Matrix(1, 7) << -1, 0, 0, 0, 0, 0, 0).finished();
    if (Hd1) *Hd1 = (gtsam::Matrix(1, 1) << -dt_ / 2).finished();
    if (Hd2) *Hd2 = (gtsam::Matrix(1, 1) << -dt_ / 2).finished();

    return error;
  }

  // Print contents
  void print(const std::string& s = "", const gtsam::KeyFormatter& keyFormatter = gtsam::DefaultKeyFormatter) const override {
    std::cout << (s.empty() ? "" : s + " ") << "ClockFactor_CCDD" << std::endl;
    std::cout << "  dt: " << dt_ << std::endl;
    Base::print("", keyFormatter);
  }
};

}  // namespace gtsam_gnss
