/**
 * @file SC_DopplerFactor_V.h
 * @brief Doppler factor with 3D velocity (V) and Switchable Constraint (S)
 * @note This factor is applied to Doppler observations that do not include receiver clock drift
 * @author Taro Suzuki
 */

#pragma once
#include <gtsam/nonlinear/NonlinearFactor.h>
#include "SwitchVariable.h"

namespace gtsam_gnss {

/**
 * @brief Doppler factor: Estimate 3D velocity
 */
class SC_DopplerFactor_V : public gtsam::NoiseModelFactorN<gtsam::Vector, SwitchVariable> {

private:
  gtsam::Vector losvec_;
  double prr_;
  gtsam::Vector iniv_;
  typedef gtsam::NoiseModelFactorN<gtsam::Vector, SwitchVariable> Base;

public:
  /**
   * @brief Constructor
   * @param keyV    3D velocity (V) key, V has 3 dimension
   * @param keyS    Switchable Constraint (S) key, S has 1 dimension
   * @param losvec  Line-of-Sight vector (3 dimension)
   * @param prr     Doppler (pseudorange rate) residual at initial 3D velocity (1 dimension)
   * @param iniv    Initial 3D velocity when calculating residual (3 dimension)
   * @param model   Gaussian noise model (1 dimension)
   */
    SC_DopplerFactor_V(gtsam::Key keyV,
                       gtsam::Key keyS,
                       const gtsam::Vector& losvec,
                       const double& prr,
                       const gtsam::Vector& iniv,
                       const gtsam::SharedNoiseModel& model)
  : Base(model, keyV, keyS), losvec_(losvec), prr_(prr), iniv_(iniv) {};

  ~SC_DopplerFactor_V() override {}

  // Error function
  using Base::evaluateError;
  gtsam::Vector evaluateError(const gtsam::Vector& v,
                              const SwitchVariable& s,
                              gtsam::OptionalMatrixType Hv,
                              gtsam::OptionalMatrixType Hs) const override {
    // Compute error
    gtsam::Vector1 error;
    error << (losvec_.transpose() * (v - iniv_)).value() - prr_;

    // Jacobian
    if (Hv) *Hv = (gtsam::Matrix(1, 3) << s.value()*losvec_.transpose()).finished();
    if (Hs) *Hs = error;

    return s.value()*error;
  }

  // Print contents
  void print(const std::string& s = "", const gtsam::KeyFormatter& keyFormatter = gtsam::DefaultKeyFormatter) const override {
    std::cout << (s.empty() ? "" : s + " ") << "SC_DopplerFactor_V" << std::endl;
    std::cout << "  Doppler residual: " << prr_ << std::endl;
    Base::print("", keyFormatter);
  }

  // Measurement
  inline const double& measurementIn() const { return prr_; }
};

}  // namespace gtsam_gnss
