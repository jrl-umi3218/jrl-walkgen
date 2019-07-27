/*
 * Copyright 2010,
 *
 * Andrei Herdt
 * Olivier Stasse
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
/* \file mpc-trajectory-generation.cpp
   \brief Abstract object for trajectory generation via model 
   predictive control.*/

#ifndef _TRAJ_GEN_H_
#define _TRAJ_GEN_H_


#include <deque>
#include <string>

#include <jrl/walkgen/pgtypes.hh>
#include <SimplePlugin.hh>

namespace PatternGeneratorJRL
{

  /*! This class defines an abstract interface to generate 
    low-dimensional walking trajectories (CoM/CdP/Feet).
   */
  class  MPCTrajectoryGeneration : public SimplePlugin
  {

    //
    // Protected members
    //
  protected:

    /* ! \brief Time for single support. */
    double Tsingle_;

    /* ! \brief Time for double support. */
    double Tdble_;

    /// \brief Sampling periods control and preview
    double Tctr_, Tprw_;

    /* ! \brief Preview control window in second. */
    double PreviewControlTime_;

    /// \brief Size of the preview window (Nb. of samplings)
    unsigned int N_;

    /// \brief Number of variables
    int NbVariables_;

    /* ! \brief Step height for the walking pattern. */
    double StepHeight_;

    /* ! \brief Current time in the control loop (in seconds). */
    double CurrentTime_;

    /*! \brief Specifies if we are or not in on line mode. */
    bool OnLineMode_;

    /// \brief CoM height.
    double CoMHeight_;

    /// \brief Security margins for the ZMP
    double SecurityMargin_;

    /// \name Members related to the generation of feet trajectories.
    /// @{
    /// \brief ModulationSupportCoefficient coeeficient to wait
    /// a little before foot is of the ground
    double ModulationSupportCoefficient_;

    /// \brief The foot orientation for the lift off and the landing
    double Omega_;

    /// \brief The foot orientation for the lift off and the landing
    double Omega2_;
    /// @}


    //
    // Public methods
    //
  public:

    /// \brief Default constructor
    MPCTrajectoryGeneration(SimplePluginManager * lSPM);

    /// \brief Default destructor.
    virtual ~MPCTrajectoryGeneration() {};

    /*! Set the current time. */
    void CurrentTime(double Time)
    {
      CurrentTime_ = Time;
    }

    /*! Get the current time. */
    double CurrentTime()
    {
      return CurrentTime_;
    }

    /// \brief Set the length of the preview window.
    inline void PreviewLength(double aPP)
    {
      PreviewControlTime_ = aPP;
    };

    /// \brief Get the preview control time window.
    inline const double & PreviewLength( ) const
    {
      return PreviewControlTime_;
    };

    /// \brief Return the Com Height.
    inline const double & ComHeight() const
    {
      return CoMHeight_;
    };

    /// \brief Return the Com Height.
    inline void ComHeight(double ComHeight)
    {
      CoMHeight_ = ComHeight;
    };

    /// \brief Return the single support time.
    inline const double & TSingleSupport() const
    {
      return Tsingle_;
    };

    /// \brief Set the single support time.
    inline void TSingleSupport(double TSingleSupport)
    {
      Tsingle_ = TSingleSupport;
    };

    /// \brief Return the double support time.
    inline const double & TDoubleSupport() const
    {
      return Tdble_;
    };

    /// \brief Set the double support time. */
    inline void TDoubleSupport(double Tdble)
    {
      Tdble_ = Tdble;
    };

    /// \brief Get the sampling period for the control, set
    /// to 0.005 by default. */
    inline const double & SamplingPeriodControl() const
    {
      return Tctr_;
    };

    /// \brief Set the sampling period for the control. */
    inline void SamplingPeriodControl(double SamplingPeriod)
    {
      Tctr_ = SamplingPeriod;
    };

    /// \brief Get the sampling period for the preview, set to 0.100 by default.
    inline const double & SamplingPeriodPreview() const
    {
      return Tprw_;
    };

    /// \brief Set the sampling period for the preview.
    inline void SamplingPeriodPreview(double SamplingPeriod)
    {
      Tprw_ = SamplingPeriod;
    };

    /// \brief Get the sampling period for the preview, set to 0.100 by default.
    inline const unsigned int & NbPrwSamplings() const
    {
      return N_;
    };

    /// \brief Set the sampling period for the preview.
    inline void NbPrwSamplings(int NbSamplings)
    {
      N_ = NbSamplings;
    };

    /// \brief Set the number of optimization parameters.
    inline void NbVariables(int NbVariables)
    {
      NbVariables_ = NbVariables;
    };

    /// \brief Set the number of optimization parameters.
    inline const int & NbVariables() const
    {
      return NbVariables_;
    };

    /// \brief Set the security margin for the zmp
    ///
    /// \param Margin
    inline void SecurityMargin(double Margin)
    {
      SecurityMargin_ = Margin;
    };


    /// \name Methods related to the generation of feet trajectories.
    /// @{
    /// \brief Returns the step height.
    inline const double & StepHeight() const
    {
      return StepHeight_;
    };

    /// \brief Specify the step height.
    inline void StepHeight(double aSSH)
    {
      StepHeight_ = aSSH;
    };

    /// \brief Returns the ModulationSupportCoefficient.
    inline const double &ModulationSupportCoefficient() const
    {
      return ModulationSupportCoefficient_;
    }

    /// \brief Specify the modulation support coefficient.
    inline void  ModulationSupportCoefficient(double af)
    {
      ModulationSupportCoefficient_ = af;
    }

    /// \brief Set the pitch angle of foot when landing and taking off.
    inline void setOmega(double anOmega)
    {
      Omega_ = anOmega;
    };

    /// \brief Get the pitch angle of foot when landing and taking off.
    inline const double & getOmega(void) const
    {
      return Omega_;
    };
    /// @}



    /// \brief Handling methods for the plugin mecanism.
    virtual void CallMethod(std::string & Method, std::istringstream &strm);

    /*! \name Methods related to the on line status generation of the ZMP.
      @{
    */
    /*! \brief Returns the current status of the ZMP trajectory generator.
      The online mode is determines internally.
      A ZMP-generator can be still in on-line mode even the step-generator
      is not because the ZMP-generator is generating the ending phase.
    */
    bool GetOnLineMode();
    /// @}  */



  };
}

#endif /* _TRAJ_GEN_H_ */
