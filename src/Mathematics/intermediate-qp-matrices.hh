/*
 * Copyright 2011,
 *
 * Olivier  Stasse
 * Andrei   Herdt
 *
 * JRL, CNRS/AIST
 *
 * This file is part of walkGenJrl.
 * walkGenJrl is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * walkGenJrl is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Lesser Public License for more details.
 * You should have received a copy of the GNU Lesser General Public License
 * along with walkGenJrl.  If not, see <http://www.gnu.org/licenses/>.
 *
 *  Research carried out within the scope of the
 *  Joint Japanese-French Robotics Laboratory (JRL)
 */

#ifndef INTERMEDQPMAT_HH_
#define INTERMEDQPMAT_HH_

#include <privatepgtypes.hh>

#include <fstream>
#include <iostream>

#include <PreviewControl/rigid-body-system.hh>

namespace PatternGeneratorJRL {
/// \brief Custom (value based) container providing intermediate elements
/// for the construction of a QP.
class IntermedQPMat {
  //
  // Public types
  //
public:
  /// \name QP elements that are objective independent
  /// \{
  struct state_variant_s {
    /// \brief Objective independent QP elements

    /// \brief Reference
    reference_t Ref;

    /// \brief State of the Center of Mass
    com_t CoM;

    /// \brief TrunkState
    trunk_t Trunk;

    /// \brief Selection matrix for the previewed feet positions
    Eigen::MatrixXd V;
    /// \brief Shifted selection matrix for the previewed feet positions
    Eigen::MatrixXd Vshift;
    /// \brief Transpose of V
    Eigen::MatrixXd VT;
    /// \brief Selection matrix multiplied with the current foot position
    Eigen::VectorXd VcX, VcY;
    /// \brief Shifted selection matrix multiplied with the current feet
    /// position
    Eigen::VectorXd VcshiftX, VcshiftY;
    /// \brief Selection matrix for the current foot position
    Eigen::VectorXd Vc_fX, Vc_fY;
    /// \brief Selection matrix for relative feet positions
    Eigen::MatrixXd V_f;
    /// \brief Current support state
    support_state_t SupportState;
  };
  typedef state_variant_s state_variant_t;
  /// \}

  /// \name Least square objective's dependent elements
  /// \{
  struct objective_variant_s {
    /// \brief Ponderation
    double weight;

    /// \brief Minimization objective type
    objective_e type;

    std::ostream &print(std::ostream &o) const;
    void dump(const char *filename) const;
  };
  typedef objective_variant_s objective_variant_t;
  /// \}

  //
  // Public methods
  //
public:
  /// \name Constructors and destructors.
  /// \{
  IntermedQPMat();
  ~IntermedQPMat();
  /// \}

  /// \name Accessors and mutators
  /// \{
  inline state_variant_t const &State() const { return StateMatrices_; };
  inline state_variant_t &State() { return StateMatrices_; };

  objective_variant_t const &Objective(objective_e type) const;
  objective_variant_t &Objective(objective_e type);

  linear_inequality_t const &Inequalities(ineq_e type) const;
  linear_inequality_t &Inequalities(ineq_e type);

  inline com_t const &CoM() const { return StateMatrices_.CoM; };
  inline void CoM(const com_t &CoM) { StateMatrices_.CoM = CoM; };

  inline trunk_t const &Trunk() const { return StateMatrices_.Trunk; }
  inline void Trunk(const trunk_t &Trunk) { StateMatrices_.Trunk = Trunk; }

  inline reference_t const &Reference() const { return StateMatrices_.Ref; };
  inline reference_t &Reference() { return StateMatrices_.Ref; };
  inline void Reference(const reference_t &Ref) { StateMatrices_.Ref = Ref; };

  inline support_state_t const &SupportState() const {
    return StateMatrices_.SupportState;
  };
  inline support_state_t &SupportState() {
    return StateMatrices_.SupportState;
  };
  inline void SupportState(const support_state_t &SupportState) {
    StateMatrices_.SupportState = SupportState;
  };
  /// \}

  /// \name Displaying
  /// \{
  /// \brief Dump
  void dump_objective(objective_e type, std::ostream &aos);
  void dump_state(std::ostream &aos);
  void dump_objective(const char *filename, objective_e type);
  void dump_state(const char *filename);
  /// \}

  //
  // Private members
  //
private:
  objective_variant_t MeanVelocity_, InstantVelocity_, COPCentering_, JerkMin_;

  state_variant_t StateMatrices_;

  linear_inequality_t IneqCoP_, IneqCoM_, IneqFeet_;
};

std::ostream &operator<<(std::ostream &o,
                         const IntermedQPMat::objective_variant_s &r);
} // namespace PatternGeneratorJRL

#endif /* INTERMEDQPMAT_HH_ */
