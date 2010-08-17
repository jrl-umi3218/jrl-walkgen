/* \file Abstract Object test aim at testing various walking algorithms
 * Olivier Stasse
 * (c) 2010, JRL, UMI 3218/CRT, JRL, CNRS/AIST
 * 
 */


#ifndef _TEST_OBJECT_PATTERN_GENERATOR_UTESTING_H_
#define _TEST_OBJECT_PATTERN_GENERATOR_UTESTING_H_

#ifdef UNIX
#include <sys/time.h>
#include <stdlib.h>
#endif /*UNIX*/

#ifdef WIN32
#include <Windows.h>
#include "TimeUtilsWindows.h"
#endif /*WIN32*/

#include <ostream>

#include <MatrixAbstractLayer/MatrixAbstractLayer.h>

#include <dynamicsJRLJapan/dynamicsJRLJapanFactory.h>

#include <walkGenJrl/PatternGeneratorInterface.h>

#include "CommonTools.h"
#include "ClockCPUTime.h"


namespace PatternGeneratorJRL
{
  namespace TestSuite
  {

    /*! \brief Class running one test per algorithm */
    class TestObject
    {
    public:
      /*! \brief Constructor for the test named TestName.
       All generated files will have their names prefixed by TestName*/
      TestObject(int argc, char *argv[], 
		 std::string &TestName,
		 int lPGIInterface=0);

      /*! \name Destructor */
      ~TestObject();
  
      /*! \brief Perform test. */
      void doTest(std::ostream &os);

    protected:

      /*! \brief Choose which test to perform. */
      virtual void chooseTestProfile()=0;
      
      /*! \brief Generate events. */
      virtual void generateEvent()=0;

      /*! \brief Profile of the test to perform. */
      unsigned int m_TestProfile;
  
      /*! \name Vectors storing the robot's state.
	@{
      */
      /*! \brief Configuration of the robot joints. */
      MAL_VECTOR(m_CurrentConfiguration,double);

      /*! \brief Velocity of the robot joints*/
      MAL_VECTOR(m_CurrentVelocity,double);

      /*! \brief Acceleration of the robot joints. */
      MAL_VECTOR(m_CurrentAcceleration,double);

      /*! \brief Previous configuration of the robot. */
      MAL_VECTOR(m_PreviousConfiguration,double) ;

      /*! \brief Previous velocity */
      MAL_VECTOR(m_PreviousVelocity,double);
  
      /*! \brief Previous acceleration */
      MAL_VECTOR(m_PreviousAcceleration,double);

      /*! @} */

      /* !\name Handle on the Humanoids models
	 @{
      */
      /*! \brief Abstract model of the humanoid robot considered */
      CjrlHumanoidDynamicRobot * m_HDR ;

      /*! \brief Abstract model of the humanoid robot for debugging purposes. */
      CjrlHumanoidDynamicRobot * m_DebugHDR ;

      /*! @} */

      /*! \brief Pointer towards the Pattern Generator Interface */
      PatternGeneratorInterface * m_PGI;

      /*! \name Debugging information 
	@{
      */
      /*! \brief ZMP of the multibody robot. 
	This flag makes sense only for algorithm allowing to compute
	the whole robot articular trajectories.
      */
      bool m_DebugZMP2;
  
      /*! \brief Output Com, ZMP and feet trajectories
	for a single mass robot model. */
      bool m_DebugFGPI;

      /*! \brief Reset debug files according to flags. */
      void prepareDebugFiles();
  
      /*! \brief Fill in the debug files with the appropriate
	information */
      void fillInDebugFiles();

      /*! @} */

      /*! \brief Information related to one step of computation. */
      struct OneStep m_OneStep;

      /*! \brief Name of the test */
      std::string m_TestName;

      /*! \brief Clock CPU timing 
	This object measure three parts of the algorithm:
	off-line, on-line, and during modification.
       */
      ClockCPUTime m_clock;

      /*! \brief Number of maximum iterations for outer loop. 
	Default value is set to 1.
       */
      unsigned int m_OuterLoopNbItMax;

      /*! \brief Patten Generator Interface. */
      int m_PGIInterface;
    };

    
  }; /* end of TestSuite namespace */
}; /* end of PatternGeneratorJRL namespace */
#endif /* _TEST_OBJECT_PATTERN_GENERATOR_UTESTING_H_*/
