/**
 * @file TDCPFactor_VVDD.h
 * @brief Time-difference Carrier Phase (TDCP) factor with 3D velocities (V) and receiver clock drifts (D)
 * @author Taro Suzuki
 */

#pragma once
#include <gtsam/nonlinear/NonlinearFactor.h>

namespace gtsam_gnss {

/**
 * @brief TDCP factor: Estimate 3D velocity and receiver clock drift
 */
class TDCPFactor_VVDD : public gtsam::NoiseModelFactorN<gtsam::Vector, gtsam::Vector, gtsam::Vector, gtsam::Vector> {

private:
  gtsam::Vector losvec_;
  double tdcp_;
  double dt_;
  gtsam::Vector iniv1_;
  gtsam::Vector iniv2_;
  typedef gtsam::NoiseModelFactorN<gtsam::Vector, gtsam::Vector, gtsam::Vector, gtsam::Vector> Base;

public:
  /**
   * @brief Constructor
   * @param keyV1   3D velocity (V) key at time t1, V has 3 dimension
   * @param keyV2   3D velocity (V) key at time t2, V has 3 dimension
   * @param keyD1   Receiver clock drift (D) key at time t1, D has 1 dimension
   * @param keyD2   Receiver clock drift (D) key at time t2, D has 1 dimension
   * @param losvec  Line-of-Sight vector (3 dimension)
   * @param tdcp    TDCP measurement (meter) (1 dimension)
   * @param dt      Time interval (t2-t1)
   * @param iniv1   Initial 3D velocity at time t1 when calculating residual (3 dimension)
   * @param iniv2   Initial 3D velocity at time t2 when calculating residual (3 dimension)
   * @param model   Gaussian noise model (1 dimension)
   */
  TDCPFactor_VVDD(gtsam::Key keyV1,
                  gtsam::Key keyV2,
                  gtsam::Key keyD1,
                  gtsam::Key keyD2,
                  const gtsam::Vector& losvec,
                  const double& tdcp,
                  const double& dt,
                  const gtsam::Vector& iniv1,
                  const gtsam::Vector& iniv2,
                  const gtsam::SharedNoiseModel& model)
  : Base(model, keyV1, keyV2, keyD1, keyD2), losvec_(losvec), tdcp_(tdcp), dt_(dt), iniv1_(iniv1), iniv2_(iniv2) {};

  ~TDCPFactor_VVDD() override {}

  // Error function
  using Base::evaluateError;
  gtsam::Vector evaluateError(const gtsam::Vector& v1,
                              const gtsam::Vector& v2,
                              const gtsam::Vector& d1,
                              const gtsam::Vector& d2,
                              gtsam::OptionalMatrixType Hv1,
                              gtsam::OptionalMatrixType Hv2,
                              gtsam::OptionalMatrixType Hd1,
                              gtsam::OptionalMatrixType Hd2) const override {

    // Compute error
    gtsam::Vector dv = dt_ * ((v1 - iniv1_) + (v2 - iniv2_)) / 2;
    gtsam::Vector dc = dt_ * (d1 + d2) / 2;

    gtsam::Vector1 error;
    error << (losvec_.transpose() * dv + dc).value() - tdcp_;

    if (Hv1) *Hv1 = (gtsam::Matrix(1, 3) << losvec_.transpose() * dt_ / 2).finished();
    if (Hv2) *Hv2 = (gtsam::Matrix(1, 3) << losvec_.transpose() * dt_ / 2).finished();
    if (Hd1) *Hd1 = (gtsam::Matrix(1, 1) << 0.5).finished();
    if (Hd2) *Hd2 = (gtsam::Matrix(1, 1) << 0.5).finished();

    return error;
  }

  // Print contents
  void print(const std::string& s = "", const gtsam::KeyFormatter& keyFormatter = gtsam::DefaultKeyFormatter) const override {
    std::cout << (s.empty() ? "" : s + " ") << "TDCPFactor_VVDD" << std::endl;
    std::cout << "  TDCP residual: " << tdcp_ << std::endl;
    Base::print("", keyFormatter);
  }

  // Measurement
  inline const double& measurementIn() const { return tdcp_; }
};

}  // namespace gtsam_gnss
