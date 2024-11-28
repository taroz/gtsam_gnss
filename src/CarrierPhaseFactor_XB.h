/**
 * @file CarrierPhaseFactor_XB.h
 * @brief Carrier phase factor with 3D position (X) and carrier phase bias (B)
 * @note This factor is usually used for DD carrier phase observation
 * @author Taro Suzuki
 */

#pragma once
#include <gtsam/nonlinear/NonlinearFactor.h>

namespace gtsam_gnss {

/**
 * @brief Carrier phase factor: Estimate 3D position and carrier phase bias
 */
class CarrierPhaseFactor_XB : public gtsam::NoiseModelFactorN<gtsam::Vector, gtsam::Vector> {

private:
  gtsam::Vector losvec_;
  double cp_;
  int biasidx_;
  int refbiasidx_;
  double lam_;
  gtsam::Vector inix_;
  typedef gtsam::NoiseModelFactorN<gtsam::Vector, gtsam::Vector> Base;

public:
  /**
   * @brief Constructor for double differenced (DD) carrier phase bias
   * @param keyX    3D position (X) key, X has 3 dimension
   * @param keyB    Carrier phase bias (B) key, B has Nbias dimension
   * @param losvec  Line-of-Sight vector (3 dimension)
   * @param cp      Carrier phase residual at initial 3D position (1 dimension)
   * @param biasidx Carrier phase bias index from 0 up to Nbias (1 dimension)
   * @param lam     Wave length (1 dimension)
   * @param inix    Initial 3D position when calculating residual (3 dimension)
   * @param model   Gaussian noise model (3 dimension)
   */
  CarrierPhaseFactor_XB(gtsam::Key keyX,
                        gtsam::Key keyB,
                        const gtsam::Vector& losvec,
                        const double& cp,
                        const int& biasidx,
                        const double& lam,
                        const gtsam::Vector& inix,
                        const gtsam::SharedNoiseModel& model)
  : Base(model, keyX, keyB), losvec_(losvec), cp_(cp), biasidx_(biasidx), refbiasidx_(-1), lam_(lam), inix_(inix) {};

  /**
   * @brief Constructor for single differenced (SD) carrier phase bias
   * @param keyX    3D position (X) key, X has 3 dimension
   * @param keyB    Carrier phase bias (B) key, B has Nbias dimension
   * @param losvec  Line-of-Sight vector (3 dimension)
   * @param cp      Carrier phase residual at initial 3D position (1 dimension)
   * @param biasidx Carrier phase bias index from 0 up to Nbias (1 dimension)
   * @param refbiasidx Carrier phase bias index of reference satellite (1 dimension)
   * @param lam     Wave length (1 dimension)
   * @param inix    Initial 3D position when calculating residual (3 dimension)
   * @param model   Gaussian noise model (3 dimension)
   */
  CarrierPhaseFactor_XB(gtsam::Key keyX,
                        gtsam::Key keyB,
                        const gtsam::Vector& losvec,
                        const double& cp,
                        const int& biasidx,
                        const int& refbiasidx,
                        const double& lam,
                        const gtsam::Vector& inix,
                        const gtsam::SharedNoiseModel& model)
  : Base(model, keyX, keyB), losvec_(losvec), cp_(cp), biasidx_(biasidx), refbiasidx_(refbiasidx), lam_(lam), inix_(inix) {};

  ~CarrierPhaseFactor_XB() override {}

  // Error function
  using Base::evaluateError;
  gtsam::Vector evaluateError(const gtsam::Vector& x,
                              const gtsam::Vector& b,
                              gtsam::OptionalMatrixType Hx,
                              gtsam::OptionalMatrixType Hb) const override {
    gtsam::Vector1 error;
    if (refbiasidx_ >= 0) {
      // Estimate single differenced carrier phase bias
      error << losvec_.transpose() * (x - inix_) - (cp_ - lam_ * (b[biasidx_] - b[refbiasidx_]));
    } else {
      // Estimate double differenced carrier phase bias
      error << losvec_.transpose() * (x - inix_) - (cp_ - lam_ * b[biasidx_]);
    }

    gtsam::Matrix Hb_ = gtsam::Matrix::Zero(1, b.size());
    Hb_(biasidx_) = lam_;
    if (refbiasidx_ >= 0) Hb_(refbiasidx_) = -lam_;

    if (Hx) *Hx = (gtsam::Matrix(1, 3) << losvec_.transpose()).finished();
    if (Hb) *Hb = Hb_;

    return error;
  }

  // Print contents
  void print(const std::string& s = "", const gtsam::KeyFormatter& keyFormatter = gtsam::DefaultKeyFormatter) const override {
    std::cout << (s.empty() ? "" : s + " ") << "CarrierPhaseFactor_XB" << std::endl;
    std::cout << "  carrier phase residual: " << cp_ << ", bias index: " << biasidx_ << std::endl;
    Base::print("", keyFormatter);
  }

  // Measurement
  inline const double& measurementIn() const { return cp_; }
};

}  // namespace gtsam_gnss
