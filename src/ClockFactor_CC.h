/**
 * @file ClockFactor_CC.h
 * @brief Clock factor with receiver clocks (C)
 * @note This factor is used to constrain changes in the receiver clock
 * @author Taro Suzuki
 */

#pragma once
#include <gtsam/nonlinear/NonlinearFactor.h>

namespace gtsam_gnss {

/**
 * @brief Clock factor: Constrain changes in the receiver clock
 */
class ClockFactor_CC : public gtsam::NoiseModelFactorN<gtsam::Vector, gtsam::Vector> {

private:
  typedef gtsam::NoiseModelFactorN<gtsam::Vector, gtsam::Vector> Base;

public:
  /**
   * @brief Constructor
   * @param keyC1   Receiver clock (C) key at time t1, C has Nc dimension
   * @param keyC2   Receiver clock (C) key at time t2, C has Nc dimension
   * @param model   Gaussian noise model (Nc dimension)
   */
  ClockFactor_CC(gtsam::Key keyC1, gtsam::Key keyC2, const gtsam::SharedNoiseModel& model)
  : Base(model, keyC1, keyC2) {};

  ~ClockFactor_CC() override {}

  // Error function
  using Base::evaluateError;
  gtsam::Vector evaluateError(const gtsam::Vector& c1,
                              const gtsam::Vector& c2,
                              gtsam::OptionalMatrixType Hc1,
                              gtsam::OptionalMatrixType Hc2) const override {
    // Compute error
    size_t nc = c1.size();

    gtsam::Vector error = gtsam::Vector::Zero(nc);
    error << c2 - c1;
    
    // Jacobian
    if (Hc1) *Hc1 = -gtsam::Matrix::Identity(nc, nc);
    if (Hc2) *Hc2 =  gtsam::Matrix::Identity(nc, nc);

    return error;
  }

  // Print contents
  void print(const std::string& s = "", const gtsam::KeyFormatter& keyFormatter = gtsam::DefaultKeyFormatter) const override {
    std::cout << (s.empty() ? "" : s + " ") << "ClockFactor_CC" << std::endl;
    Base::print("", keyFormatter);
  }
};

}  // namespace gtsam_gnss
