/*!\file FootTrajectoryGenerationAbstract.h
   \brief This class determinate how it s generate all the values for the foot trajectories.

   @ingroup foottrajectorygeneration

   Copyright (c) 2007-2009, 
   @author Francois Keith, Olivier Stasse,

   $Id$
   
   JRL-Japan, CNRS/AIST

   All rights reserved.
   
   Please see License.txt for further information on license.
*/


#ifndef _FOOT_TRAJECTORY_GENERATION_ABSTRACT_H_
#define _FOOT_TRAJECTORY_GENERATION_ABSTRACT_H_

/* System related inclusions */
#include <deque>

/*! MatrixAbstractLayer */
#include <MatrixAbstractLayer/MatrixAbstractLayer.h>

/* dynamics JRL Japan related inclusions */
#include <robotDynamics/jrlFoot.h>

/* Walking pattern generation related inclusions */

#include <PGTypes.h>
#include <SimplePlugin.h>

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
  class  FootTrajectoryGenerationAbstract :public SimplePlugin
  {
  public:
    
    /*! Constructor: In order to compute some appropriate strategies,
      this class needs to extract specific details from the humanoid model. */
    FootTrajectoryGenerationAbstract(SimplePluginManager *lSPM,
				     CjrlFoot *inFoot) ;


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
      @param LeftOrRight: Specify if it is left (1) or right (-1).
    */
    virtual void UpdateFootPosition(std::deque<FootAbsolutePosition> &SupportFootAbsolutePositions,
				    std::deque<FootAbsolutePosition> &NoneSupportFootAbsolutePositions,
				    int CurrentAbsoluteIndex,  
				    int IndexInitial, 
				    double ModulatedSingleSupportTime,
				    int StepType, int LeftOrRight);

    virtual void UpdateFootPosition(std::deque<FootAbsolutePosition> &SupportFootAbsolutePositions,
				    std::deque<FootAbsolutePosition> &NoneSupportFootAbsolutePositions,
				    int StartIndex, int k,
				    double LocalInterpolationStartTime,
				    double ModulatedSingleSupportTime,
				    int StepType, int LeftOrRight);

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

    /*! Store a pointer to Foot information. */
    CjrlFoot * m_Foot;
    
    /*! Omega the angle for taking off and landing. */
    double m_Omega;
  };
  
  
};
#endif /* _FOOT_TRAJECTORY_GENERATION_ABSTRACT_H_ */

