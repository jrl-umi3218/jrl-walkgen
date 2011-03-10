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
   \brief Abstract object for trajectory generation via model predictive control.*/

#ifndef _TRAJ_GEN_H_
#define _TRAJ_GEN_H_

#include <jrl/mal/matrixabstractlayer.hh>


#include <deque>
#include <string>

#include <jrl/walkgen/pgtypes.hh>
#include <SimplePlugin.h>

namespace PatternGeneratorJRL
{
	
  /*! This class defines an abstract interface to generate low-dimensional walking trajectories (CoM/CdP/Feet). 
  */
  class  MPCTrajectoryGeneration : public SimplePlugin
  {

    //
     // Protected members
     //
   protected:

     /* ! \brief Time for single support. */
     double m_Tsingle;

     /* ! \brief Time for double support. */
     double m_Tdble;

     /// \brief Sampling periods control and preview
     double m_SamplingPeriodControl, m_SamplingPeriodPreview;

     /* ! \brief Preview control window in second. */
     double m_PreviewControlTime;

     /* ! \brief Step height for the walking pattern. */
     double m_StepHeight;

     /* ! \brief Current time in the control loop (in seconds). */
     double m_CurrentTime;

     /*! \brief Specifies if we are or not in on line mode. */
     bool m_OnLineMode;

     /*! \brief Specifies Com Height. */
     double m_CoMHeight;

     /// \brief Security margins for the ZMP
     double m_SecurityMargin;

     /// \name Members related to the generation of feet trajectories.
     /// @{
     /// \brief ModulationSupportCoefficient coeeficient to wait a little before foot is of the ground
     double m_ModulationSupportCoefficient;

     /// \brief The foot orientation for the lift off and the landing
     double m_Omega;
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
    void setCurrentTime(const double & aTime)
    { m_CurrentTime = aTime;}
		
    /*! Get the current time. */
    double getCurrentTime()
    { return m_CurrentTime;}
		
    /// \brief Set the length of the preview window. 
    inline void setPreviewLength(const double & aPP)
    { m_PreviewControlTime = aPP;};
		
    /// \brief Get the preview control time window.
    inline const double & getPreviewLength( ) const
    { return m_PreviewControlTime; };
		
    /// \brief Returns the Com Height.
    inline const double & getComHeight() const
    { return m_CoMHeight; };
		
    /// \brief Returns the Com Height. 
    inline void setComHeight(const double & aComHeight)
    { m_CoMHeight = aComHeight; };
		
    /// \brief Returns the single support time. 
    inline const double & getTSingleSupport() const
    { return m_Tsingle; };
		
    /// \brief Set the single support time.
    inline void setTSingleSupport(const double & aTSingleSupport)
    { m_Tsingle = aTSingleSupport; };
		
    /// \brief Returns the double support time.
    inline const double & getTDoubleSupport() const
    {return m_Tdble;};
		
    /// \brief Set the double support time. */
    inline void setTDoubleSupport(const double & aTdble)
    { m_Tdble = aTdble;};
		
    /// \brief Get the sampling period for the control, set to 0.005 by default. */
    inline const double & getSamplingPeriodControl() const
    { return m_SamplingPeriodControl; };
		
    /// \brief Set the sampling period for the control. */
    inline void setSamplingPeriodControl(const double &aSamplingPeriod)
    { m_SamplingPeriodControl = aSamplingPeriod;};
		
    /// \brief Get the sampling period for the preview, set to 0.100 by default.
    inline const double & getSamplingPeriodPreview() const
    { return m_SamplingPeriodPreview; };
		
    /// \brief Set the sampling period for the preview. */
    inline void setSamplingPeriodPreview(const double &aSamplingPeriod)
    { m_SamplingPeriodPreview = aSamplingPeriod;};
		
    /// \brief Set the security margin for the zmp
    ///
    /// \param Margin
    inline void setSecurityMargin(const double & Margin)
    {m_SecurityMargin = Margin; };
		
		
    /// \name Methods related to the generation of feet trajectories.
    /// @{
    /// \brief Returns the step height.
    inline const double & getStepHeight() const
    { return m_StepHeight;};
		
    /// \brief Specify the step height. 
    inline void setStepHeight(const double & aSSH)
    { m_StepHeight = aSSH;};
		
    /// \brief Returns the ModulationSupportCoefficient. 
    inline const double &getModulationSupportCoefficient() const
    {
      return m_ModulationSupportCoefficient;
    }
		
    /// \brief Specify the modulation support coefficient. 
    inline void  setModulationSupportCoefficient(const double &af)
    {
      m_ModulationSupportCoefficient = af;
    }
		
    /// \brief Set the pitch angle of foot when landing and taking off.
    inline void setOmega(const double & anOmega) 
    { m_Omega = anOmega;};
		
    /// \brief Get the pitch angle of foot when landing and taking off. 
    inline const double & getOmega(void) const
    { return m_Omega;};
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
