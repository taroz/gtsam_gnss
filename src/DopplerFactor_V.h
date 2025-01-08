/**
 * @file DopplerFactor_V.h
 * @brief Doppler factor with 3D velocity (V)
 * @note This factor is applied to Doppler observations that do not include receiver clock drift
 * @author Taro Suzuki
 */

#pragma once
#include <gtsam/nonlinear/NonlinearFactor.h>

namespace gtsam_gnss {

/**
 * @brief Doppler factor: Estimate 3D velocity
 */
class DopplerFactor_V : public gtsam::NoiseModelFactorN<gtsam::Vector> {

private:
  gtsam::Vector losvec_;
  double prr_;
  gtsam::Vector iniv_;
  typedef gtsam::NoiseModelFactorN<gtsam::Vector> Base;

public:
  /**
   * @brief Constructor
   * @param keyV    3D velocity (V) key, V has 3 dimension
   * @param losvec  Line-of-Sight vector (3 dimension)
   * @param prr     Doppler (pseudorange rate) residual at initial 3D velocity (1 dimension)
   * @param iniv    Initial 3D velocity when calculating residual (3 dimension)
   * @param model   Gaussian noise model (1 dimension)
   */
    DopplerFactor_V(gtsam::Key keyV,
                    const gtsam::Vector& losvec,
                    const double& prr,
                    const gtsam::Vector& iniv,
                    const gtsam::SharedNoiseModel& model)
  : Base(model, keyV), losvec_(losvec), prr_(prr), iniv_(iniv) {};

  ~DopplerFactor_V() override {}

  // Error function
  using Base::evaluateError;
  gtsam::Vector evaluateError(const gtsam::Vector& v, gtsam::OptionalMatrixType Hv) const override {
    // Compute error
    gtsam::Vector1 error;
    error << (losvec_.transpose() * (v - iniv_)).value() - prr_;

    // Jacobian
    if (Hv) *Hv = (gtsam::Matrix(1, 3) << losvec_.transpose()).finished();

    return error;
  }

  // Print contents
  void print(const std::string& s = "", const gtsam::KeyFormatter& keyFormatter = gtsam::DefaultKeyFormatter) const override {
    std::cout << (s.empty() ? "" : s + " ") << "DopplerFactor_V" << std::endl;
    std::cout << "  Doppler residual: " << prr_ << std::endl;
    Base::print("", keyFormatter);
  }

  // Measurement
  inline const double& measurementIn() const { return prr_; }
};

}  // namespace gtsam_gnss
