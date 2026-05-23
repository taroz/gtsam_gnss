/**
 * @file BetweenFactorSwitchVariable.h
 * @brief Between Factor of Switch Variable
 * @author Taro Suzuki
 */

#pragma once
#include <gtsam/nonlinear/NonlinearFactor.h>
#include "SwitchVariable.h"

namespace gtsam_gnss {

/**
 * @brief Between Factor of Switchable Constraint
 */
class BetweenFactorSwitchVariable : public gtsam::NoiseModelFactorN<SwitchVariable, SwitchVariable> {

private:
  double measured_;
  typedef gtsam::NoiseModelFactorN<SwitchVariable, SwitchVariable> Base;

public:
  /**
   * @brief Constructor
   * @param keyS1     Variable key
   * @param keyS2     Variable key
   * @param measured   Between measurement
   * @param model   Gaussian noise model
   */
  BetweenFactorSwitchVariable(gtsam::Key keyS1,
                                    gtsam::Key keyS2,
                                    const double& measured,
                                    const gtsam::SharedNoiseModel& model = nullptr)
   : Base(model, keyS1, keyS2), measured_(measured) {};

  ~BetweenFactorSwitchVariable() override {}

  // Error function
  using Base::evaluateError;
  gtsam::Vector evaluateError(const SwitchVariable& s1,
                              const SwitchVariable& s2,
                              gtsam::OptionalMatrixType Hs1,
                              gtsam::OptionalMatrixType Hs2) const override {
    if (Hs1) (*Hs1) = -gtsam::Matrix::Identity(1,1);
    if (Hs2) (*Hs2) =  gtsam::Matrix::Identity(1,1);
    return gtsam::Vector1((s2.value() - s1.value())- measured_);
  }

  // Print contents
  void print(const std::string& str = "", const gtsam::KeyFormatter& keyFormatter = gtsam::DefaultKeyFormatter) const override {
    std::cout << str << "BetweenFactor("
          << keyFormatter(this->key1()) << ","
          << keyFormatter(this->key2()) << ")" << std::endl;
      std::cout << "  measured: " << measured_ << std::endl;
      if (this->noiseModel_)
        this->noiseModel_->print("  noise model: ");
      else
        std::cout << "no noise model" << std::endl;
    }

  // Measurement
  inline const double& measured() const { return measured_; }
};

}  // namespace gtsam_gnss
