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
#include "CommonTools.hh"
#include "ClockCPUTime.hh"
#include <jrl/mal/matrixabstractlayer.hh>
#include <jrl/walkgen/patterngeneratorinterface.hh>
#include <jrl/walkgen/pinocchiorobot.hh>

namespace PatternGeneratorJRL
{
  namespace TestSuite
  {
    double filterprecision(double adb);

    /*! \brief Class running one test per algorithm */
    class TestObject
    {
    public:
      // overload the new[] eigen operator
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW

      /*! \brief Constructor for the test named TestName.
       All generated files will have their names prefixed by TestName*/
      TestObject(int argc, char *argv[],
		 std::string &TestName,
		 int lPGIInterface=0);

      /*! \name Destructor */
      ~TestObject();

      /*! \brief Initialize the test object. */
      bool init();
     // void initIK();

      /*! \brief Perform test. */
      virtual bool doTest(std::ostream &os);

    protected:

      /*! \brief Choose which test to perform. */
      virtual void chooseTestProfile()=0;

      /*! \brief Generate events. */
      virtual void generateEvent()=0;

      /*! \brief Profile of the test to perform. */
      unsigned int m_TestProfile;

      /*! \brief Verify that the input files exist. */
      bool checkFiles();

      /*! \brief Useful methods to create the robot model.
    @{
       */
      /*! */
      void CreateAndInitializeHumanoidRobot(std::string &URDFFile,
          std::string &SRDFFile,
          PinocchioRobot *& aPR,
          PinocchioRobot *& aDebugPR);
      /*! @} */

      /*! \brief Useful methods to parse srdf file.
    @{
       */
      /*! */
      void InitializeRobotWithSRDF(PinocchioRobot & aPR,
                                   const std::string & filename);
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

      /*! \brief Half Sitting Configuration */
      MAL_VECTOR(m_HalfSitting,double);


      /*! @} */

      /* !\name Handle on the Humanoids models
	 @{
      */
      /*! \brief Abstract model of the humanoid robot considered */
      se3::Model m_robotModel ;

      /*! \brief Abstract model of the humanoid robot considered */
      PinocchioRobot * m_PR ;
      se3::Data *m_robotData;

      /*! \brief Abstract model of the humanoid robot for debugging purposes. */
      PinocchioRobot * m_DebugPR ;
      se3::Data *m_DebugRobotData;


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
      virtual void fillInDebugFiles();

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
      /*! \brief full path to the URDF. */
      std::string m_URDFPath;
      /*! \brief full path of the SRDF. */
      std::string m_SRDFPath;

      /*! @} */
    // utilities for Herdt and Naveau
    public :


      void startTurningLeft(PatternGeneratorInterface &aPGI)
      {
        std::istringstream strm2(":setVelReference  0.2 0.0 6.0832");
        aPGI.ParseCmd(strm2);
      }

      void startTurningRight(PatternGeneratorInterface &aPGI)
      {
        //istringstream strm2(":setVelReference  0.2 0.0 -0.2");
        std::istringstream strm2(":setVelReference  0.15 0.0 -0.08");
        aPGI.ParseCmd(strm2);
      }

      void startTurningRight2(PatternGeneratorInterface &aPGI)
      {
        std::istringstream strm2(":setVelReference  0.2 0.0 -0.17");
        aPGI.ParseCmd(strm2);
      }

      void startTurningLeft2(PatternGeneratorInterface &aPGI)
      {
        std::istringstream strm2(":setVelReference  0.0 0.0 0.4");
        aPGI.ParseCmd(strm2);
      }

      void startTurningLeftOnSpot(PatternGeneratorInterface &aPGI)
      {
        std::istringstream strm2(":setVelReference  0.0 0.0 10.0");
        aPGI.ParseCmd(strm2);
      }

      void startTurningRightOnSpot(PatternGeneratorInterface &aPGI)
      {
        std::istringstream strm2(":setVelReference  0.0 0.0 -10.");
        aPGI.ParseCmd(strm2);
      }

      void stop(PatternGeneratorInterface &aPGI)
      {
        std::istringstream strm2(":setVelReference 0.0 0.0 0.0");
        aPGI.ParseCmd(strm2);
      }
      void walkForward1m_s(PatternGeneratorInterface &aPGI)
      {
        std::istringstream strm2(":setVelReference  0.1 0.0 0.0");
        aPGI.ParseCmd(strm2);
      }
      void walkForward2m_s(PatternGeneratorInterface &aPGI)
      {
        std::istringstream strm2(":setVelReference  0.2 0.0 0.0");
        aPGI.ParseCmd(strm2);
      }
      void walkForward3m_s(PatternGeneratorInterface &aPGI)
      {
        std::istringstream strm2(":setVelReference  0.3 0.0 0.0");
        aPGI.ParseCmd(strm2);
      }
      void walkSidewards1m_s(PatternGeneratorInterface &aPGI)
      {
        std::istringstream strm2(":setVelReference  0.0 -0.1 0.0");
        aPGI.ParseCmd(strm2);
      }
      void walkSidewards2m_s(PatternGeneratorInterface &aPGI)
      {
        std::istringstream strm2(":setVelReference  0.0 -0.2 0.0");
        aPGI.ParseCmd(strm2);
      }
      void walkSidewards3m_s(PatternGeneratorInterface &aPGI)
      {
        std::istringstream strm2(":setVelReference  0.0 -0.3 0.0");
        aPGI.ParseCmd(strm2);
      }
      void startWalkInDiagonal1m_s(PatternGeneratorInterface &aPGI)
      {
        std::istringstream strm2(":setVelReference  0.1 0.1 0.0");
        aPGI.ParseCmd(strm2);
      }
      void startWalkInDiagonal2m_s(PatternGeneratorInterface &aPGI)
      {
        std::istringstream strm2(":setVelReference  0.2 0.2 0.0");
        aPGI.ParseCmd(strm2);
      }

      void startWalkInDiagonal3m_s(PatternGeneratorInterface &aPGI)
      {
        std::istringstream strm2(":setVelReference  0.3 0.3 0.0");
        aPGI.ParseCmd(strm2);
      }

      void stopOnLineWalking(PatternGeneratorInterface &aPGI)
      {
        std::istringstream strm2(":setVelReference  0.0 0.0 0.0");
        aPGI.ParseCmd(strm2);
        std::istringstream strm3(":stoppg");
        aPGI.ParseCmd(strm3);
      }

      void walkOnSpot(PatternGeneratorInterface &aPGI)
      {
        std::istringstream strm2(":setVelReference  0.0001 0.0 0.0");
        aPGI.ParseCmd(strm2);
      }

      void perturbationForce(PatternGeneratorInterface &aPGI)
      {
        std::istringstream strm2(":perturbationforce  -20.0 8.0 0.0");
        aPGI.ParseCmd(strm2);
      }
    }; /* end of TestObject class */
  } /* end of TestSuite namespace */
} /* end of PatternGeneratorJRL namespace */
#endif /* _TEST_OBJECT_PATTERN_GENERATOR_UTESTING_H_*/
