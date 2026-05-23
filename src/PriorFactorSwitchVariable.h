/**
 * @file PriorFactorSwitchVariable.h
 * @brief Prior Factor of Switch Variable
 * @author Taro Suzuki
 */

#pragma once
#include <gtsam/nonlinear/NonlinearFactor.h>
#include "SwitchVariable.h"

namespace gtsam_gnss {

/**
 * @brief Prior Factor of Switchable Constraint
 */
class PriorFactorSwitchVariable : public gtsam::NoiseModelFactorN<SwitchVariable> {

private:
  SwitchVariable prior_;
  typedef gtsam::NoiseModelFactorN<SwitchVariable> Base;

public:
  /**
   * @brief Constructor
   * @param key     Variable key
   * @param prior   Prior measurement
   * @param model   Gaussian noise model
   */
  PriorFactorSwitchVariable(gtsam::Key key,
                                    const SwitchVariable& prior,
                                    const gtsam::SharedNoiseModel& model = nullptr)
   : Base(model, key), prior_(prior) {};

  ~PriorFactorSwitchVariable() override {}

  // Error function
  using Base::evaluateError;
  gtsam::Vector evaluateError(const SwitchVariable& s,
                              gtsam::OptionalMatrixType Hs) const override {
    if (Hs) (*Hs) = gtsam::Matrix::Identity(1,1);
      return gtsam::Vector1(s.value() - prior_.value());
  }

  // Print contents
  void print(const std::string& str = "", const gtsam::KeyFormatter& keyFormatter = gtsam::DefaultKeyFormatter) const override {
    std::cout << str << "PriorFactor on " << keyFormatter(this->key()) << std::endl;
      gtsam::traits<SwitchVariable>::Print(prior_, "  prior mean: ");
      if (this->noiseModel_)
        this->noiseModel_->print("  noise model: ");
      else
        std::cout << "no noise model" << std::endl;
    }

  // Measurement
  const SwitchVariable& prior() const { return prior_; }
};

}  // namespace gtsam_gnss
