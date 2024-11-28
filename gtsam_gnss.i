namespace gtsam_gnss {

// Pseudorange factor with 3D position (X) and receiver clock (C)
#include <PseudorangeFactor_XC.h>
virtual class PseudorangeFactor_XC : gtsam::NoiseModelFactor {
  PseudorangeFactor_XC(size_t keyX, size_t keyC, const gtsam::Vector& losvec, const double& pr, const int& sysidx, const gtsam::Vector& inix, gtsam::noiseModel::Base* model);
  gtsam::Vector evaluateError(const gtsam::Vector& x, const gtsam::Vector& c) const;
  void print(string s = "", const gtsam::KeyFormatter& keyFormatter = gtsam::DefaultKeyFormatter) const;
  const double& measurementIn() const;
};

// Pseudorange factor with 3D position (X)
#include <PseudorangeFactor_X.h>
virtual class PseudorangeFactor_X : gtsam::NoiseModelFactor {
  PseudorangeFactor_X(size_t keyX, const gtsam::Vector& losvec, const double& pr, const gtsam::Vector& inix, gtsam::noiseModel::Base* model);
  gtsam::Vector evaluateError(const gtsam::Vector& x) const;
  void print(string s = "", const gtsam::KeyFormatter& keyFormatter = gtsam::DefaultKeyFormatter) const;
  const double& measurementIn() const;
};

// Doppler factor with 3D velocity (V) and receiver clock drift (D)
#include <DopplerFactor_VD.h>
virtual class DopplerFactor_VD : gtsam::NoiseModelFactor {
  DopplerFactor_VD(size_t keyV, size_t keyD, const gtsam::Vector& losvec, const double& prr, const gtsam::Vector& iniv, gtsam::noiseModel::Base* model);
  gtsam::Vector evaluateError(const gtsam::Vector& v, const gtsam::Vector& d) const;
  void print(string s = "", const gtsam::KeyFormatter& keyFormatter = gtsam::DefaultKeyFormatter) const;
  const double& measurementIn() const;
};

// Doppler factor with 3D velocity (V)
#include <DopplerFactor_V.h>
virtual class DopplerFactor_V : gtsam::NoiseModelFactor {
  DopplerFactor_V(size_t keyV, const gtsam::Vector& losvec, const double& prr, const gtsam::Vector& iniv, gtsam::noiseModel::Base* model);
  gtsam::Vector evaluateError(const gtsam::Vector& v) const;
  void print(string s = "", const gtsam::KeyFormatter& keyFormatter = gtsam::DefaultKeyFormatter) const;
  const double& measurementIn() const;
};

// Doppler factor with 3D positions (X) and receiver clocks (C)
#include <DopplerFactor_XXCC.h>
virtual class DopplerFactor_XXCC : gtsam::NoiseModelFactor {
  DopplerFactor_XXCC(size_t keyX1, size_t keyX2, size_t keyC1, size_t keyC2, const gtsam::Vector& losvec, const double& prr, const double& dt, const gtsam::Vector& inix1, const gtsam::Vector& inix2, gtsam::noiseModel::Base* model);
  gtsam::Vector evaluateError(const gtsam::Vector& x1, const gtsam::Vector& x2, const gtsam::Vector& c1, const gtsam::Vector& c2) const;
  void print(string s = "", const gtsam::KeyFormatter& keyFormatter = gtsam::DefaultKeyFormatter) const;
  const double& measurementIn() const;
};

// Carrier phase factor with 3D position (X) and carrier phase bias (B)
#include <CarrierPhaseFactor_XB.h>
virtual class CarrierPhaseFactor_XB : gtsam::NoiseModelFactor {
  CarrierPhaseFactor_XB(size_t keyX, size_t keyB, const gtsam::Vector& losvec, const double& cp, const int& biasidx, const double& lam, const gtsam::Vector& inix, gtsam::noiseModel::Base* model);
  CarrierPhaseFactor_XB(size_t keyX, size_t keyB, const gtsam::Vector& losvec, const double& cp, const int& biasidx, const int& refbiasidx, const double& lam, const gtsam::Vector& inix, gtsam::noiseModel::Base* model);
  gtsam::Vector evaluateError(const gtsam::Vector& x, const gtsam::Vector& b) const;
  void print(string s = "", const gtsam::KeyFormatter& keyFormatter = gtsam::DefaultKeyFormatter) const;
  const double& measurementIn() const;
};

// Time-difference Carrier Phase (TDCP) factor with 3D positions (X) and receiver clocks (C)
#include <TDCPFactor_XXCC.h>
virtual class TDCPFactor_XXCC : gtsam::NoiseModelFactor {
  TDCPFactor_XXCC(size_t keyX1, size_t keyX2, size_t keyC1, size_t keyC2, const gtsam::Vector& losvec, const double& tdcp, const gtsam::Vector& inix1, const gtsam::Vector& inix2, gtsam::noiseModel::Base* model);
  gtsam::Vector evaluateError(const gtsam::Vector& x1, const gtsam::Vector& x2, const gtsam::Vector& c1, const gtsam::Vector& c2) const;
  void print(string s = "", const gtsam::KeyFormatter& keyFormatter = gtsam::DefaultKeyFormatter) const;
  const double& measurementIn() const;
};

// Time-difference Carrier Phase (TDCP) factor with 3D velocities (V) and receiver clock drifts (D)
#include <TDCPFactor_VVDD.h>
virtual class TDCPFactor_VVDD : gtsam::NoiseModelFactor {
  TDCPFactor_VVDD(size_t keyV1, size_t keyV2, size_t keyD1, size_t keyD2, const gtsam::Vector& losvec, const double& tdcp, const double& dt, const gtsam::Vector& iniv1, const gtsam::Vector& iniv2, gtsam::noiseModel::Base* model);
  gtsam::Vector evaluateError(const gtsam::Vector& v1, const gtsam::Vector& v2, const gtsam::Vector& d1, const gtsam::Vector& d2) const;
  void print(string s = "", const gtsam::KeyFormatter& keyFormatter = gtsam::DefaultKeyFormatter) const;
  const double& measurementIn() const;
};

// Time-difference Carrier Phase (TDCP) factor with 3D positions (X)
#include <TDCPFactor_XX.h>
virtual class TDCPFactor_XX : gtsam::NoiseModelFactor {
  TDCPFactor_XX(size_t keyX1, size_t keyX2, const gtsam::Vector& losvec, const double& tdcp, const gtsam::Vector& inix1, const gtsam::Vector& inix2, gtsam::noiseModel::Base* model);
  gtsam::Vector evaluateError(const gtsam::Vector& x1, const gtsam::Vector& x2) const;
  void print(string s = "", const gtsam::KeyFormatter& keyFormatter = gtsam::DefaultKeyFormatter) const;
  const double& measurementIn() const;
};

// Time-difference Carrier Phase (TDCP) factor with 3D positions (X) and receiver clock drifts (D)
#include <TDCPFactor_XXDD.h>
virtual class TDCPFactor_XXDD : gtsam::NoiseModelFactor {
  TDCPFactor_XXDD(size_t keyX1, size_t keyX2, size_t keyD1, size_t keyD2, const gtsam::Vector& losvec, const double& tdcp, const gtsam::Vector& inix1, const gtsam::Vector& inix2, gtsam::noiseModel::Base* model);
  gtsam::Vector evaluateError(const gtsam::Vector& x1, const gtsam::Vector& x2, const gtsam::Vector& d1, const gtsam::Vector& d2) const;
  void print(string s = "", const gtsam::KeyFormatter& keyFormatter = gtsam::DefaultKeyFormatter) const;
  const double& measurementIn() const;
};

// Motion factor with 3D positions (X) and 3D velocities (V)
#include <MotionFactor_XXVV.h>
virtual class MotionFactor_XXVV : gtsam::NoiseModelFactor {
  MotionFactor_XXVV(size_t keyX1, size_t keyX2, size_t keyV1, size_t keyV2, const double& dt, gtsam::noiseModel::Base* model);
  gtsam::Vector evaluateError(const gtsam::Vector& x1, const gtsam::Vector& x2, const gtsam::Vector& v1, const gtsam::Vector& v2) const;
  void print(string s = "", const gtsam::KeyFormatter& keyFormatter = gtsam::DefaultKeyFormatter) const;
};

// Clock factor with receiver clocks (C) and receiver clock drifts (D)
#include <ClockFactor_CCDD.h>
virtual class ClockFactor_CCDD : gtsam::NoiseModelFactor {
  ClockFactor_CCDD(size_t keyC1, size_t keyC2, size_t keyD1, size_t keyD2, const double& dt, gtsam::noiseModel::Base* model);
  gtsam::Vector evaluateError(const gtsam::Vector& c1, const gtsam::Vector& c2, const gtsam::Vector& d1, const gtsam::Vector& d2) const;
  void print(string s = "", const gtsam::KeyFormatter& keyFormatter = gtsam::DefaultKeyFormatter) const;
};

// Pose3Point3 factor with 3D pose (P) and 3D position (X)
#include <Pose3Point3Factor_PX.h>
virtual class Pose3Point3Factor_PX : gtsam::NoiseModelFactor {
  Pose3Point3Factor_PX(size_t keyP, size_t keyX, gtsam::noiseModel::Base* model);
  gtsam::Vector evaluateError(const gtsam::Pose3& p, const gtsam::Vector& x) const;
  void print(string s = "", const gtsam::KeyFormatter& keyFormatter = gtsam::DefaultKeyFormatter) const;
};

// Acceleration factor: Estimate 3D velocity and acceleration bias
#include <AccelerationFactor_VVBB.h>
virtual class AccelerationFactor_VVBB : gtsam::NoiseModelFactor {
  AccelerationFactor_VVBB(size_t keyV1, size_t keyV2, size_t keyB1, size_t keyB2, const double& g, const double& dt, const double& acc3d, gtsam::noiseModel::Base* model);
  gtsam::Vector evaluateError(const gtsam::Vector& v1, const gtsam::Vector& v2, const gtsam::Vector& b1, const gtsam::Vector& b2) const;
  void print(string s = "", const gtsam::KeyFormatter& keyFormatter = gtsam::DefaultKeyFormatter) const;
  const double& measurementIn() const;
};

// Angular rate factor: Estimate 3D velocity and angular rate bias
#include <AngularRateFactor_VVBB.h>
virtual class AngularRateFactor_VVBB : gtsam::NoiseModelFactor {
  AngularRateFactor_VVBB(size_t keyV1, size_t keyV2, size_t keyB1, size_t keyB2, const double& axang, gtsam::noiseModel::Base* model);
  gtsam::Vector evaluateError(const gtsam::Vector& v1, const gtsam::Vector& v2, const gtsam::Vector& b1, const gtsam::Vector& b2) const;
  void print(string s = "", const gtsam::KeyFormatter& keyFormatter = gtsam::DefaultKeyFormatter) const;
  const double& measurementIn() const;
};

}
