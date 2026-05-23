/**
 * @file SwitchVariable.h
 * @brief Switch Variable class for Switchable Constraints
 * @author Taro Suzuki
 */

#pragma once

namespace gtsam_gnss {

/**
 * @brief Switch Variable class
 */
class SwitchVariable {
  double s_;

public:
  inline constexpr static auto dimension = 1;

  /// Default Constructor
  SwitchVariable() : s_(1.0) {}

  /// Constructor from initial switch value
  SwitchVariable(double s) : s_(s) {}

  double value() const { return s_; }

  /* print with optional string */
  void print(const std::string& s = "") const { std::cout << s << "switch value: " << s_ << std::endl; }

  /* equals with an tolerance */
  bool equals(const SwitchVariable& other, double tol = 1e-9) const { return std::abs(s_ - other.s_) < tol; }

  /// Updates a with tangent space delta
  inline SwitchVariable retract(const gtsam::Vector v) const {
    double s = s_ + v(0);
    if (s > 1.0)
      s = 1.0;
    else if (s < 0.0)
      s = 0.0;
    return SwitchVariable(s);
  }

  /// Returns inverse retraction
  inline Vector localCoordinates(const SwitchVariable& q) const { return gtsam::Vector1(q.s_ - s_); }
};

// Values utility functions for Switch Variable
void insertSwitchVariable(gtsam::Key key, const SwitchVariable& s, gtsam::Values& values) {
  values.insert(key, s);
}

// Values utility functions for Switch Variable
SwitchVariable atSwitchVariable(gtsam::Key key, const gtsam::Values& values) {
  return values.at<SwitchVariable>(key);
}

}  // namespace gtsam_gnss

namespace gtsam {

// Define GTSAM traits
template <>
struct traits<gtsam_gnss::SwitchVariable> : public internal::Manifold<gtsam_gnss::SwitchVariable> {};

}  // namespace gtsam
