/*
 * Copyright 2011,
 *
 * Mehdi BENALLEGUE
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
/*! This object constructs a QP as proposed by Herdt IROS 2010 with constraints on relative positions of the steps.
 */

#ifndef _GENERATORSTEPPOS_HH_
#define _GENERATORSTEPPOS_HH_

#include <ZMPRefTrajectoryGeneration/generator-vel-ref.hh>

#include <Mathematics/relative-feet-equalities.hh>


namespace PatternGeneratorJRL
{
 
  /// \brief Generate optimization problem as proposed in Herdt2010IROS
  class  GeneratorStepPos : public GeneratorVelRef
  {
  public:
	GeneratorStepPos(SimplePluginManager *lSPM);

	GeneratorStepPos(const GeneratorVelRef &genvr);


    /// \brief Compute constraints
    ///
    /// \param[in] Pb
    /// \param[in] FCALS
    /// \param[in] AbsoluteLeftFootPositions
    /// \param[in] AbsoluteRightFootPositions
    /// \param[in] deqSupportStates
    /// \param[in] PreviewedSupportAngles
    virtual void build_constraints( QPProblem & Pb,
			  RelativeFeetInequalities * RFI,
			  const std::deque< FootAbsolutePosition> & AbsoluteLeftFootPositions,
			  const std::deque<FootAbsolutePosition> & AbsoluteRightFootPositions,
			  const std::deque<support_state_t> & deqSupportStates,
			  const std::deque<double> & PreviewedSupportAngles);


	/// \brief set the velocity mode / step pos mode
    ///
    /// \param[in] vm
	void setVelocityMode(bool vm);

	 /// \brief Set the weights on an objective term
	  ///
	  /// \param[in] weight
	  /// \param[in] objective
	  virtual void Ponderation(double weight, int objective );

	  /// \brief Set the weights on an objective term without taking care of the velocity mode
	  ///
	  /// \param[in] weight
	  /// \param[in] objective
	  void explicitPonderation(double weight, int objective);

	    /// \brief Compute feet constraints corresponding to the set of equalities
    ///
    /// \param[in] EqFeet
    /// \param[in] State
    /// \param[in] NbStepsPreviewed
    /// \param[in] Pb
    void build_constraints_step_pos(const linear_inequality_t & IneqFeet,
                              const IntermedQPMat::state_variant_t & State,
                              int NbStepsPreviewed, QPProblem & Pb);

	
    /// \brief Generate a queue of equality constraints on
    /// the steps positions with respect to previous foot positions, 
	/// it gets the number of constraints as an output
    ///
    /// \param[out] equalities
    /// \param[in] FCALS
    /// \param[in] AbsoluteLeftFootPositions
    /// \param[in] AbsoluteRightFootPositions
    /// \param[in] deqSupportStates
    /// \param[in] PreviewedSupportAngles
	/// \param[in] stepPos
      int build_equalities_step_pos(linear_inequality_t & equalities,
                               RelativeFeetInequalities * FCALS,
                               const std::deque< FootAbsolutePosition> & AbsoluteLeftFootPositions,
                               const std::deque<FootAbsolutePosition> & AbsoluteRightFootPositions,
                               const std::deque<support_state_t> & deqSupportStates,
                               const std::deque<double> & PreviewedSupportAngles,
							   RelativeStepPositionQueue& stepPos) const;

	  /// \brief Handling methods for the plugin mecanism.
	  virtual void CallMethod( std::string &Method, std::istringstream &Args );

   
  protected:
	  bool velocityMode_;
	  RelativeFeetEqualities rfe_;
  };
}

#endif /* _GENERATORSTEPPOS_HH_ */
