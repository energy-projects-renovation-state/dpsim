// SPDX-FileCopyrightText: 2026 Institute for Automation of Complex Power Systems, EONERC, RWTH Aachen University
// SPDX-License-Identifier: MPL-2.0

#pragma once

#include <dpsim-models/MNASimPowerComp.h>
#include <dpsim-models/Solver/MNAInterface.h>

namespace CPS {
namespace EMT {

/// \brief Abstract base class for EMT state-space nodal (SSN) components.
///
/// This class provides the generic state-space and scheduling logic shared by
/// all SSN component types:
/// - storage of state-space matrices A, B, C, D
/// - calculation of trapezoidal discrete-time matrices
/// - steady-state initialization helpers
/// - history-vector calculation
/// - state update
/// - generic pre-step and post-step dependency registration
///
/// The exact interpretation of input and output quantities is defined in
/// derived classes.
class SSNComp : public MNASimPowerComp<Real> {
private:
  const Int mInputSize;
  const Int mOutputSize;

protected:
  Matrix mW;
  Matrix mYHist;

  Matrix mA;
  Matrix mB;
  Matrix mC;
  Matrix mD;

  Matrix mdA;
  Matrix mdB;

  const CPS::Attribute<Matrix>::Ptr mX;

  SSNComp(String uid, String name, Int inputSize, Int outputSize,
          Logger::Level logLevel = Logger::Level::off);

  Matrix calculateHistoryVector() const;

  MatrixComp calculateSteadyStateStateFromInput(const MatrixComp &u,
                                                Real frequency) const;
  MatrixComp calculateSteadyStateOutputFromInput(const MatrixComp &x,
                                                 const MatrixComp &u) const;

  void updateState(const Matrix &uOld, const Matrix &uNew);

  virtual Attribute<Matrix>::Ptr inputAttribute() const = 0;
  virtual Attribute<Matrix>::Ptr outputAttribute() const = 0;

public:
  void setParameters(const Matrix &A, const Matrix &B, const Matrix &C,
                     const Matrix &D);

  void mnaCompInitialize(Real omega, Real timeStep,
                         Attribute<Matrix>::Ptr leftVector) final;

  void
  mnaCompAddPreStepDependencies(AttributeBase::List &prevStepDependencies,
                                AttributeBase::List &attributeDependencies,
                                AttributeBase::List &modifiedAttributes) final;

  void mnaCompPreStep(Real time, Int timeStepCount) final;

  void
  mnaCompAddPostStepDependencies(AttributeBase::List &prevStepDependencies,
                                 AttributeBase::List &attributeDependencies,
                                 AttributeBase::List &modifiedAttributes,
                                 Attribute<Matrix>::Ptr &leftVector) final;
};

} // namespace EMT
} // namespace CPS
