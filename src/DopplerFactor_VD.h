/**
 * @file DopplerFactor_VD.h
 * @brief Doppler factor with 3D velocity (V) and receiver clock drift (D)
 * @author Taro Suzuki
 */

#pragma once
#include <gtsam/nonlinear/NonlinearFactor.h>

namespace gtsam_gnss {

/**
 * @brief Doppler factor: Estimate 3D velocity and receiver clock drift
 */
class DopplerFactor_VD : public gtsam::NoiseModelFactorN<gtsam::Vector, gtsam::Vector> {

private:
  gtsam::Vector losvec_;
  double prr_;
  gtsam::Vector iniv_;
  typedef gtsam::NoiseModelFactorN<gtsam::Vector, gtsam::Vector> Base;

public:
  /**
   * @brief Constructor
   * @param keyV    3D velocity (V) key, V has 3 dimension
   * @param keyD    Receiver clock drift (D) key, D has 1 dimension
   * @param losvec  Line-of-Sight vector (3 dimension)
   * @param prr     Doppler (pseudorange rate) residual at initial 3D velocity (1 dimension)
   * @param iniv    Initial 3D velocity (3 dimension)
   * @param model   Gaussian noise model (3 dimension)
   */
  DopplerFactor_VD(gtsam::Key keyV,
                   gtsam::Key keyD,
                   const gtsam::Vector& losvec,
                   const double& prr,
                   const gtsam::Vector& iniv,
                   const gtsam::SharedNoiseModel& model)
  : Base(model, keyV, keyD), losvec_(losvec), prr_(prr), iniv_(iniv) {};

  ~DopplerFactor_VD() override {}

  // Error function
  using Base::evaluateError;
  gtsam::Vector evaluateError(const gtsam::Vector& v,
                              const gtsam::Vector& d,
                              gtsam::OptionalMatrixType Hv,
                              gtsam::OptionalMatrixType Hd) const override {
    // Compute error
    gtsam::Vector1 error;
    error << (losvec_.transpose() * (v - iniv_) + d).value() - prr_;

    if (Hv) *Hv = (gtsam::Matrix(1, 3) << losvec_.transpose()).finished();
    if (Hd) *Hd = (gtsam::Matrix(1, 1) << 1.0).finished();

    return error;
  }

  // Print contents
  void print(const std::string& s = "", const gtsam::KeyFormatter& keyFormatter = gtsam::DefaultKeyFormatter) const override {
    std::cout << (s.empty() ? "" : s + " ") << "DopplerFactor_VD" << std::endl;
    std::cout << "  Doppler residual: " << prr_ << std::endl;
    Base::print("", keyFormatter);
  }

  // Measurement
  inline const double& measurementIn() const { return prr_; }
};

}  // namespace gtsam_gnss
