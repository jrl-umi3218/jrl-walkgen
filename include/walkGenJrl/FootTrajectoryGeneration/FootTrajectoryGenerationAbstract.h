/*!\file FootTrajectoryGenerationAbstract.h
   \brief This class determinate how it s generate all the values for the foot trajectories.

   @ingroup foottrajectorygeneration

   Copyright (c) 2007, 
   @author Olivier Stasse,

   $Id$
   
   JRL-Japan, CNRS/AIST

   All rights reserved.
   
   Redistribution and use in source and binary forms, with or without modification, 
   are permitted provided that the following conditions are met:
   
   * Redistributions of source code must retain the above copyright notice, 
   this list of conditions and the following disclaimer.
   * Redistributions in binary form must reproduce the above copyright notice, 
   this list of conditions and the following disclaimer in the documentation 
   and/or other materials provided with the distribution.
   * Neither the name of the CNRS/AIST nor the names of its contributors 
   may be used to endorse or promote products derived from this software without specific prior written permission.
   
   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS 
   OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY 
   AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER 
   OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, 
   OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS 
   OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) 
   HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, 
   STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING 
   IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/


#ifndef _FOOT_TRAJECTORY_GENERATION_ABSTRACT_H_
#define _FOOT_TRAJECTORY_GENERATION_ABSTRACT_H_

/* System related inclusions */
#include <deque>

/* dynamics JRL Japan related inclusions */
#include <dynamicsJRLJapan/HumanoidSpecificities.h>

/* Walking pattern generation related inclusions */
#include <walkGenJrl/PGTypes.h>
#include <walkGenJrl/SimplePlugin.h>

namespace PatternGeneratorJRL
{

  /** @ingroup foottrajectorygeneration
      This class defines the abstract interface to interact with foot generation object.

      Two parameters \f$ T_{DS} \f$ and \f$ T_{SS} \f$ defines respectively 
      the double support time and the single support time.

      \f$ \omega \f$ defines the angle of the feet for landing and taking off.
      The foot rotates around the toe from \f$ 0 \f$ to \f$ \omega \f$ 
      for taking off. Whereas for the landing the foot rotates around the 
      heel from \f$ \omega \f$ to \f$ 0 \f$.
      
      The sampling control time indicates the amount of time between two
      iteration of the algorithm. This parameter is used in the method
      UpdateFootPosition to compute the time from the iteration number
      given in parameters.

      An instance of a class derived from FootTrajectoryGenerationAbstract,
      should call InitializeInternalDataStructures() once all the internal
      parameters of the object are set.
      
      The virtual function FreeInternalDataStructures() is used when changing some
      parameters and by the destructor.

      The most important function is UpdateFootPosition() which populates a
      queue of foot absolute positions data structure. 

      See a derived class such as FootTrajectoryGenerationStandard 
      for more precise information on the usage and sample codes. 
      
  */
  class FootTrajectoryGenerationAbstract :public SimplePlugin
  {
  public:
    
    /*! Constructor: In order to compute some appropriate strategies,
      this class needs to extract specific details from the humanoid model. */
    FootTrajectoryGenerationAbstract(SimplePluginManager *lSPM,
				     dynamicsJRLJapan::HumanoidSpecificities *aHS) ;


    /*! Default destructor. */
    virtual ~FootTrajectoryGenerationAbstract(){};

    /*! This method computes the position of the swinging foot during single support phase,
      and maintian a constant position for the support foot.
      @param SupportFootAbsolutePositions: Queue of absolute position for the support foot.
      This method will set the foot position at index CurrentAbsoluteIndex of the queue.
      This position is supposed to be constant.
      @param NoneSupportFootAbsolutePositions: Queue of absolute position for the swinging
      foot. This method will set the foot position at index NoneSupportFootAbsolutePositions
      of the queue. 
      @param CurrentAbsoluteIndex: Index in the queues of the foot position to be set.
      @param IndexInitial: Index in the queues which correspond to the starting point
      of the current single support phase.
      @param ModulatedSingleSupportTime: Amount of time where the foot is flat.
      @param StepType: Type of steps (for book-keeping).
    */
    virtual void UpdateFootPosition(std::deque<FootAbsolutePosition> &SupportFootAbsolutePositions,
				    std::deque<FootAbsolutePosition> &NoneSupportFootAbsolutePositions,
				    int CurrentAbsoluteIndex,  
				    int IndexInitial, 
				    double ModulatedSingleSupportTime,
				    int StepType)=0;

    /*! Initialize internal data structures. */
    virtual void InitializeInternalDataStructures()=0;
    
    /*! Free internal data structures */
    virtual void FreeInternalDataStructures()=0;

    /*! \name Setter and getter for parameters 
      @{
     */
    
    /*! \name Single Support Time 
      @{
    */
    
    /*! \brief Set single support time */
    void SetSingleSupportTime(double aTSingle)
    { m_TSingle =aTSingle;};
    
    /*! \brief Get single support time */
    double GetSingleSupportTime()
    { return m_TSingle;};
    /*! @} */

    /*! \name Double Support Time 
      @{
    */

    /*! \brief Set double support time */
    void SetDoubleSupportTime(double aTDouble)
    { m_TDouble =aTDouble;};
    
    /*! \brief Get single support time */
    double GetDoubleSupportTime()
    { return m_TDouble;};

    /*! @}*/

    /*! \name Sampling control Time 
      @{
    */
    
    /*! \brief Set single support time */
    void SetSamplingPeriod(double aSamplingPeriod)
    { m_SamplingPeriod = aSamplingPeriod;};
    
    /*! \brief Get single support time */
    double GetSamplingPeriod()
    { return m_SamplingPeriod;};

    /*!@}*/
    
    /*! \name Omega. 
      @{*/

    /*! Get Omega */
    double GetOmega()
    { return m_Omega; };

    /*! Set Omega */
    void SetOmega(double anOmega)
    { m_Omega = anOmega; };
    
    /*! @} */
    /*! @} */

    /*! \brief Reimplementation of the call method for the plugin manager. 
      More explicitly this object will deal with the call which initialize
      the feet behaviors (\f$omega\f$, \f$ stepheight \f$) .
    */
    virtual void CallMethod(std::string &Method, std::istringstream &strm);

  protected: 

    /*! Sampling period of the control. */
    double m_SamplingPeriod;

    /*! Duration time for single support. */
    double m_TSingle;

    /*! Duration time for double support. */
    double m_TDouble;

    /*! Store a pointer to Humanoid Specificities. */
    dynamicsJRLJapan::HumanoidSpecificities * m_HS;
    
    /*! Omega the angle for taking off and landing. */
    double m_Omega;
  };
  
  
};
#endif /* _FOOT_TRAJECTORY_GENERATION_ABSTRACT_H_ */

