/*
 * Copyright 2010, 
 *
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
/* \file Abstract Object test aim at testing various walking algorithms */


#ifndef _TEST_OBJECT_PATTERN_GENERATOR_UTESTING_H_
#define _TEST_OBJECT_PATTERN_GENERATOR_UTESTING_H_

#ifdef UNIX
#include <sys/time.h>
#include <stdlib.h>
#endif /*UNIX*/

#ifdef WIN32
#include <Windows.h>
#include "portability/gettimeofday.hh"
#endif /*WIN32*/

#include <ostream>
#include <string>


#include <jrl/mal/matrixabstractlayer.hh>
#include <jrl/dynamics/dynamicsfactory.hh>
#include <jrl/walkgen/patterngeneratorinterface.hh>

#include "CommonTools.hh"
#include "ClockCPUTime.hh"


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
  
      /*! \brief Initialize the test object. */
      void init();

      /*! \brief Perform test. */
      bool doTest(std::ostream &os);

      /*! \brief Decide from which object the robot is build from. */
      virtual void SpecializedRobotConstructor(CjrlHumanoidDynamicRobot *& aHDR,
					       CjrlHumanoidDynamicRobot *& aDebugHDR );
      
    protected:

      /*! \brief Choose which test to perform. */
      virtual void chooseTestProfile()=0;
      
      /*! \brief Generate events. */
      virtual void generateEvent()=0;

      /*! \brief Profile of the test to perform. */
      unsigned int m_TestProfile;

      /*! \brief Useful methods to create the robot model. 
	@{
       */
      /*! */
      void CreateAndInitializeHumanoidRobot(std::string &RobotFileName,
					    std::string &LinkJointRank,
					    std::string &SpecificitiesFileName,
					    std::string &InitConfig,
					    CjrlHumanoidDynamicRobot *& aHDR,
					    CjrlHumanoidDynamicRobot *& aDebugHDR,
					    PatternGeneratorJRL::PatternGeneratorInterface *&aPGI);
      
      /*! @} */
      
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

      /*! \brief Compare debug files with references. */
      bool compareDebugFiles();

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

      /*! \brief Store options 
       @{*/
      /*! \brief Path to the VRML. */
      std::string m_VRMLPath;
      /*! \brief Name of the VRML. */
      std::string m_VRMLFileName;
      /*! \brief File describing the specificities of the robot. */
      std::string m_SpecificitiesFileName;
      /*! \brief File describing the relationship between the Joints
	and their rank in the robot's state vector */
      std::string m_LinkJointRank;

      std::string m_InitConfig;

      /*! @} */
    };

    
  } /* end of TestSuite namespace */
} /* end of PatternGeneratorJRL namespace */
#endif /* _TEST_OBJECT_PATTERN_GENERATOR_UTESTING_H_*/
