/*
 * Copyright 2010,
 *
 * Olivier Stasse
 *
 * JRL, CNRS/AIST
 *
 * This file is part of jrl-walkgen.
 * jrl-walkgen is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * jrl-walkgen is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Lesser Public License for more details.
 * You should have received a copy of the GNU Lesser General Public License
 * along with jrl-walkgen.  If not, see <http://www.gnu.org/licenses/>.
 *
 *  Research carried out within the scope of the
 *  Joint Japanese-French Robotics Laboratory (JRL)
 */
/* \file This file tests M. Morisawa's walking algorithm for
 * real-time CoM and ZMP trajectory generation
 */

#include "CommonTools.hh"
#include "TestObject.hh"

using namespace::PatternGeneratorJRL;
using namespace::PatternGeneratorJRL::TestSuite;
using namespace std;

enum Profiles_t {
  PROFIL_ANALYTICAL_ONLINE_WALKING,          // 1
  PROFIL_ANALYTICAL_SHORT_STRAIGHT_WALKING,  // 2
  PROFIL_ANALYTICAL_CLIMBING_STAIRS_10,      // 3
  PROFIL_ANALYTICAL_CLIMBING_STAIRS_15,      // 4
  PROFIL_ANALYTICAL_GOING_DOWN_STAIRS_10,    // 5
  PROFIL_ANALYTICAL_GOING_DOWN_STAIRS_15,    // 6
  PROFIL_ANALYTICAL_STEPPING_STONES,         // 7
  PROFIL_ANALYTICAL_WALKING_ON_BEAM,         // 8
  PROFIL_ANALYTICAL_GO_THROUGH_WALL          // 9
};
#define NB_PROFILE 9

#define NBOFPREDEFONLINEFOOTSTEPS 11

double OnLineFootSteps[NBOFPREDEFONLINEFOOTSTEPS][4]={
  { 0.05, 0.0, 0.0, 0.0},
  { 0.05, 0.0, 0.0, 0.0},
  { 0.05, 0.0, 0.0, 0.0},
  { 0.05, 0.0, 0.0, 0.0},
  { 0.05, 0.0, 0.0, 0.0},
  { 0.05, 0.0, 0.0, 0.0},
  { 0.05, 0.0, 0.0, 0.0},
  { 0.05, 0.0, 0.0, 0.0},
  { 0.05, 0.0, 0.0, 0.0},
  { 0.05, 0.0, 0.0, 0.0},
  { 0.05, 0.0, 0.0, 0.0}
};

class TestMorisawa2007: public TestObject
{

private:
  bool m_TestChangeFoot;
  unsigned long int m_NbStepsModified;
  // New time between two steps.
  double m_deltatime;

  // iteration for the files
  int iteration ;

public:
  TestMorisawa2007(int argc, char*argv[], string &aString, int TestProfile):
    TestObject(argc, argv, aString)
  {
    m_DebugFGPIFull = true;
    m_DebugFGPI = true;
    m_TestProfile = TestProfile;
    m_TestChangeFoot = true;
    m_NbStepsModified = 0;
    m_deltatime = 0;
    iteration = 0 ;
  }

  ~TestMorisawa2007()
  {}

protected:

  double filterprecision(double adb)
  {
    if (fabs(adb)<1e-7)
      return 0.0;

    double ladb2 = adb * 1e7;
    double lintadb2 = trunc(ladb2);
    return lintadb2/1e7;
  }

  void StartAnalyticalOnLineWalking(PatternGeneratorInterface &aPGI)
  {
    CommonInitialization(aPGI);

    {
      istringstream strm2(":SetAlgoForZmpTrajectory Morisawa");
      aPGI.ParseCmd(strm2);
    }

    {
      istringstream strm2(":onlinechangestepframe relative");
      aPGI.ParseCmd(strm2);
    }

    {
      istringstream strm2(":SetAutoFirstStep false");
      aPGI.ParseCmd(strm2);
    }

    {
      istringstream strm2(":StartOnLineStepSequencing \
                          0.0 -0.105 0.0 0.0 \
                          0.2 0.19 0.0 0.0   \
                          0.2 -0.19 0.0 0.0  \
                          0.2 0.19 0.0 0.0   \
                          0.2 -0.19 0.0 0.0  \
                          0.2 0.19 0.0 0.0   \
                          0.0 -0.19 0.0 0.0  \
                          ");
                          aPGI.ParseCmd(strm2);
    }

    {
      istringstream strm2(":useDynamicFilter false");
      aPGI.ParseCmd(strm2);
    }
  }

  void StopOnLineWalking(PatternGeneratorInterface &aPGI)
  {
    istringstream strm2(":StopOnLineStepSequencing");
    aPGI.ParseCmd(strm2);
  }

  void AnalyticalShortStraightWalking(PatternGeneratorInterface &aPGI)
  {
    CommonInitialization(aPGI);
    {
      istringstream strm2(":SetAlgoForZmpTrajectory Morisawa");
      aPGI.ParseCmd(strm2);
    }
    {
      istringstream strm2(":stepstairseq \
                          0.0 -0.105 0.0 0.0 \
                          0.2 0.19 0.0 0.0 \
                          0.2 -0.19 0.0 0.0 \
                          0.2 0.19 0.0 0.0 \
                          0.2 -0.19 0.0 0.0 \
                          0.2 0.19 0.0 0.0 \
                          0.0 -0.19 0.0 0.0\
                          ");
                          aPGI.ParseCmd(strm2);
    }
    {
      istringstream strm2(":useDynamicFilter true");
      aPGI.ParseCmd(strm2);
    }
  }

  void AnalyticalClimbingStairs10(PatternGeneratorInterface &aPGI)
  {
    CommonInitialization(aPGI);
    {
      istringstream strm2(":SetAlgoForZmpTrajectory Morisawa");
      aPGI.ParseCmd(strm2);
    }

    {
      istringstream strm2(":singlesupporttime 1.6");
      aPGI.ParseCmd(strm2);
    }

    {
      // warning  !! high speed double support !!
      istringstream strm2(":doublesupporttime 0.2");
      aPGI.ParseCmd(strm2);
    }

    {
      istringstream strm2(":stepstairseq 0.0 -0.105 0.0 0.0\
                          0.30 0.19 0.10 0.0\
                          0.0 -0.19 0.0 0.0\
                          0.31 0.19 0.10 0.0\
                          0.0 -0.19 0.0 0.0\
                          0.30 0.19 0.10 0.0\
                          0.0 -0.19 0.0 0.0\
                          0.30 0.19 0.10 0.0\
                          0.0 -0.19 0.0 0.0\
                          0.30 0.19 0.10 0.0\
                          0.0 -0.19 0.0 0.0\
                          0.31 0.19 0.10 0.0\
                          0.0 -0.19 0.0 0.0\
                          ");
                          aPGI.ParseCmd(strm2);
    }
    {
      istringstream strm2(":useDynamicFilter true");
      aPGI.ParseCmd(strm2);
    }

  }

  void AnalyticalClimbingStairs15(PatternGeneratorInterface &aPGI)
  {
    CommonInitialization(aPGI);
    {
      istringstream strm2(":SetAlgoForZmpTrajectory Morisawa");
      aPGI.ParseCmd(strm2);
    }

    {
      istringstream strm2(":singlesupporttime 1.6");
      aPGI.ParseCmd(strm2);
    }

    {
      // warning  !! high speed double support !!
      istringstream strm2(":doublesupporttime 0.2");
      aPGI.ParseCmd(strm2);
    }

    {
      istringstream strm2(":stepstairseq 0.0 -0.105 0.0 0.0\
                          0.30 0.19 0.15 0.0\
                          0.0 -0.19 0.0 0.0\
                          0.30 0.19 0.15 0.0\
                          0.0 -0.19 0.0 0.0\
                          0.30 0.19 0.15 0.0\
                          0.0 -0.19 0.0 0.0\
                          0.30 0.19 0.15 0.0\
                          0.0 -0.19 0.0 0.0\
                          ");
                          aPGI.ParseCmd(strm2);
    }
    {
      istringstream strm2(":useDynamicFilter true");
      aPGI.ParseCmd(strm2);
    }

  }

  void AnalyticalGoingDownStairs15(PatternGeneratorInterface &aPGI)
  {
    CommonInitialization(aPGI);
    {
      istringstream strm2(":SetAlgoForZmpTrajectory Morisawa");
      aPGI.ParseCmd(strm2);
    }

    {
      istringstream strm2(":singlesupporttime 1.6");
      aPGI.ParseCmd(strm2);
    }

    {
      istringstream strm2(":doublesupporttime 0.2");
      aPGI.ParseCmd(strm2);
    }

    {
      istringstream strm2(":stepstairseq 0.0 -0.105 0.0 0.0\
                          0.30 0.19 -0.15 0.0\
                          0.0 -0.19 0.0 0.0\
                          0.30 0.19 -0.15 0.0\
                          0.0 -0.19 0.0 0.0\
                          0.30 0.19 -0.15 0.0\
                          0.0 -0.19 0.0 0.0\
                          0.30 0.19 -0.15 0.0\
                          0.0 -0.19 0.0 0.0\
                          ");
                          aPGI.ParseCmd(strm2);
    }
    {
      istringstream strm2(":useDynamicFilter false");
      aPGI.ParseCmd(strm2);
    }
  }

  void AnalyticalGoingDownStairs10(PatternGeneratorInterface &aPGI)
  {
    CommonInitialization(aPGI);
    {
      istringstream strm2(":SetAlgoForZmpTrajectory Morisawa");
      aPGI.ParseCmd(strm2);
    }

    {
      istringstream strm2(":singlesupporttime 1.6");
      aPGI.ParseCmd(strm2);
    }

    {
      istringstream strm2(":doublesupporttime 0.2");
      aPGI.ParseCmd(strm2);
    }

    {
      istringstream strm2(":stepstairseq 0.0 -0.105 0.0 0.0\
                          0.31 0.19 -0.10 0.0\
                          0.0 -0.19 0.0 0.0\
                          0.30 0.19 -0.10 0.0\
                          0.0 -0.19 0.0 0.0\
                          0.30 0.19 -0.10 0.0\
                          0.0 -0.19 0.0 0.0\
                          0.30 0.19 -0.10 0.0\
                          0.0 -0.19 0.0 0.0\
                          0.31 0.19 -0.10 0.0\
                          0.0 -0.19 0.0 0.0\
                          0.30 0.19 -0.10 0.0\
                          0.0 -0.19 0.0 0.0\
                          ");
                          aPGI.ParseCmd(strm2);
    }
    {
      istringstream strm2(":useDynamicFilter false");
      aPGI.ParseCmd(strm2);
    }
  }

  void AnalyticalSteppingStones(PatternGeneratorInterface &aPGI)
  {
    CommonInitialization(aPGI);
    {
      istringstream strm2(":SetAlgoForZmpTrajectory Morisawa");
      aPGI.ParseCmd(strm2);
    }

    {
      istringstream strm2(":singlesupporttime 1.4");
      aPGI.ParseCmd(strm2);
    }

    {
      istringstream strm2(":doublesupporttime 0.2");
      aPGI.ParseCmd(strm2);
    }

    {
      istringstream strm2(":stepstairseq 0.0 -0.105 0.0 0.0\
                          0.25 0.19 0.05 0.0\
                          0.2 -0.19 0.05 0.0\
                          0.25 0.19 0.05 0.0\
                          0.2 -0.19 0.05 0.0\
                          0.2 0.19 0.0 0.0\
                          0.2 -0.19 -0.05 0.0\
                          0.2 0.19 -0.05 0.0\
                          0.2 -0.19 -0.05 0.0\
                          0.2 0.19 0.0 0.0\
                          0.0 -0.19 0.0 0.0");

                          aPGI.ParseCmd(strm2);
    }
    {
      istringstream strm2(":useDynamicFilter false");
      aPGI.ParseCmd(strm2);
    }
  }

  // Define here the function to Generate Walking on Beam motion
  void AnalyticalWalkingOnBeam(PatternGeneratorInterface &aPGI)
  {
    CommonInitialization(aPGI);
    {
      istringstream strm2(":SetAlgoForZmpTrajectory Morisawa");
      aPGI.ParseCmd(strm2);
    }

    {
      istringstream strm2(":singlesupporttime 2.1");
      aPGI.ParseCmd(strm2);
    }

    {
      istringstream strm2(":doublesupporttime 0.7");
      aPGI.ParseCmd(strm2);
    }

    {
      istringstream strm2(":stepstairseq\
                          0.0   0.105 0.0 0.0\
                          0.28 -0.145 0.0 0.0\
                          0.28  0.100 0.0 0.0\
                          0.28 -0.100 0.0 0.0\
                          0.28  0.100 0.0 0.0\
                          0.28 -0.100 0.0 0.0\
                          0.28  0.100 0.0 0.0\
                          0.28 -0.100 0.0 0.0\
                          0.28  0.100 0.0 0.0\
                          0.28 -0.100 0.0 0.0\
                          0.28  0.100 0.0 0.0\
                          0.28 -0.100 0.0 0.0\
                          0.28  0.145 0.0 0.0\
                          0.2  -0.190 0.0 0.0\
                          0.0   0.190 0.0 0.0\
                          ");
                          aPGI.ParseCmd(strm2);
    }
    {
      istringstream strm2(":useDynamicFilter false");
      aPGI.ParseCmd(strm2);
    }
  }

  void AnalyticalGoThroughWall(PatternGeneratorInterface &aPGI)
  {
    CommonInitialization(aPGI);
    {
      istringstream strm2(":SetAlgoForZmpTrajectory Morisawa");
      aPGI.ParseCmd(strm2);
    }

    {
      istringstream strm2(":singlesupporttime 1.05");
      aPGI.ParseCmd(strm2);
    }

    {
      istringstream strm2(":doublesupporttime 0.15");
      aPGI.ParseCmd(strm2);
    }

    {
      istringstream strm2(":stepheight 0.02");
      aPGI.ParseCmd(strm2);
    }

    {
      // wait for andreas the step sequence
      istringstream strm2(":stepstairseq\
                          0.06 2.0 0.0 0.0   \
                          0.0 -0.3 0.0 0.0   \
                          0.0 0.2 0.0 0.0    \
                          0.0 -0.3 0.0 0.0   \
                          0.0 0.2 0.0 0.0    \
                          0.0 -0.3 0.0 0.0   \
                          0.0 0.2 0.0 0.0    \
                          0.0 -0.3 0.0 0.0   \
                          0.0 0.2 0.0 0.0    \
                          0.0 -0.3 0.0 0.0   \
                          0.0 0.2 0.0 0.0    \
                          0.0 -0.3 0.0 0.0   \
                          0.0 0.2 0.0 0.0    \
                          0.0 -0.3 0.0 0.0   \
                          0.0 0.2 0.0 0.0    \
                          0.0 -0.3 0.0 0.0   \
                          0.0 0.2 0.0 0.0    \
                          0.0 -0.3 0.0 0.0   \
                          0.0 0.2 0.0 0.0    \
                          0.0 -0.3 0.0 0.0   \
                          0.0 0.2 0.0 0.0    \
                          0.0 -0.3 0.0 0.0   \
                          0.0 0.2 0.0 0.0    \
                          0.0 -0.3 0.0 0.0   \
                          0.0 0.2 0.0 0.0    \
                          0.0 -0.3 0.0 0.0   \
                          0.0 0.2 0.0 0.0    \
                          0.0 -0.3 0.0 0.0   \
                          0.0 0.2 0.0 0.0    \
                          0.0 -0.3 0.0 0.0   \
                          0.0 0.2 0.0 0.0    \
                          0.0 -0.3 0.0 0.0   \
                          0.0 0.2 0.0 0.0    \
                          0.0 -0.3 0.0 0.0   \
                          0.0 0.2 0.0 0.0    \
                          0.0 -0.3 0.0 0.0   \
                          0.0 0.2 0.0 0.0    \
                          0.0 -0.3 0.0 0.0   \
                          0.0 0.2 0.0 0.0    \
                          0.0 -0.3 0.0 0.0   \
                          0.0 0.2 0.0 0.0    \
                          0.0 -0.3 0.0 0.0   \
                          0.0 0.2 0.0 0.0    \
                          0.0 -0.3 0.0 0.0   \
                          0.0 0.2 0.0 0.0    \
                          0.0 -0.3 0.0 0.0   \
                          0.0 0.2 0.0 0.0    \
                          0.0 -0.3 0.0 0.0   \
                          0.0 0.2 0.0 0.0    \
                          0.0 -0.3 0.0 0.0   \
                          0.0 0.2 0.0 0.0    \
                          0.0 -0.3 0.0 0.0   \
                          0.0 0.2 0.0 0.0    \
                          0.0 -0.3 0.0 0.0   \
                          0.0 0.2 0.0 0.0    \
                          0.0 -0.3 0.0 0.0   \
                          0.0 0.2 0.0 0.0    \
                          0.0 -0.3 0.0 0.0   \
                          0.0 0.2 0.0 0.0    \
                          0.0 -0.3 0.0 0.0   \
                          0.0 0.2 0.0 0.0    \
                          0.0 -0.3 0.0 0.0   \
                          0.0 0.2 0.0 0.0    \
                          0.0 -0.3 0.0 0.0   \
                          0.0 0.2 0.0 0.0    \
                          0.0 -0.3 0.0 0.0   \
                          0.0 0.2 0.0 0.0    \
                          0.0 -0.3 0.0 0.0   \
                          0.0 0.2 0.0 0.0    \
                          0.0 -0.3 0.0 0.0   \
                          0.0 0.2 0.0 0.0    \
                          0.0 -0.3 0.0 0.0   \
                          0.0 0.2 0.0 0.0    \
                          0.0 -0.3 0.0 0.0   \
                          0.0 0.2 0.0 0.0    \
                          0.0 -0.3 0.0 0.0   \
                          0.0 0.2 0.0 0.0    \
                          0.0 -0.3 0.0 0.0   \
                          0.0 0.2 0.0 0.0    \
                          0.0 -0.3 0.0 0.0   \
                          0.0 0.2 0.0 0.0    \
                          ");
                          aPGI.ParseCmd(strm2);
    }
    {
      istringstream strm2(":useDynamicFilter false");
      aPGI.ParseCmd(strm2);
    }
  }

  void chooseTestProfile()
  {

    switch(m_TestProfile)
    {
      case PROFIL_ANALYTICAL_SHORT_STRAIGHT_WALKING:
        AnalyticalShortStraightWalking(*m_PGI);
        break;

      case PROFIL_ANALYTICAL_CLIMBING_STAIRS_10:
        AnalyticalClimbingStairs10(*m_PGI);
        break;

      case PROFIL_ANALYTICAL_CLIMBING_STAIRS_15:
        AnalyticalClimbingStairs15(*m_PGI);
        break;

      case PROFIL_ANALYTICAL_GOING_DOWN_STAIRS_10:
        AnalyticalGoingDownStairs10(*m_PGI);
        break;

      case PROFIL_ANALYTICAL_GOING_DOWN_STAIRS_15:
        AnalyticalGoingDownStairs15(*m_PGI);
        break;

      case PROFIL_ANALYTICAL_STEPPING_STONES:
        AnalyticalSteppingStones(*m_PGI);
        break;

      case PROFIL_ANALYTICAL_ONLINE_WALKING:
        StartAnalyticalOnLineWalking(*m_PGI);
        break;

      case PROFIL_ANALYTICAL_WALKING_ON_BEAM:
        AnalyticalWalkingOnBeam(*m_PGI);
        break;

      case PROFIL_ANALYTICAL_GO_THROUGH_WALL:
        AnalyticalGoThroughWall(*m_PGI);
        break;


      default:
        throw("No correct test profile");
        break;
    }
  }

  void generateEvent()
  {
    if (m_TestProfile==PROFIL_ANALYTICAL_SHORT_STRAIGHT_WALKING)
      return;
    if (m_TestProfile==PROFIL_ANALYTICAL_CLIMBING_STAIRS_10)
      return;
    if (m_TestProfile==PROFIL_ANALYTICAL_CLIMBING_STAIRS_15)
      return;
    if (m_TestProfile==PROFIL_ANALYTICAL_GOING_DOWN_STAIRS_10)
      return;
    if (m_TestProfile==PROFIL_ANALYTICAL_GOING_DOWN_STAIRS_15)
      return;
    if (m_TestProfile==PROFIL_ANALYTICAL_STEPPING_STONES)
      return;
    if (m_TestProfile==PROFIL_ANALYTICAL_WALKING_ON_BEAM)
      return;
    if (m_TestProfile==PROFIL_ANALYTICAL_GO_THROUGH_WALL)
      return;

    unsigned int StoppingTime = 20*200;


    double r = 100.0*(double)m_OneStep.NbOfIt/(double)StoppingTime;


    /* Stop after 30 seconds the on-line stepping */
    if (m_OneStep.NbOfIt>StoppingTime)
    {
      StopOnLineWalking(*m_PGI);
    }
    else
    {
      /* Stay on the spot during 5.0 s before stopping. */
      if (m_OneStep.NbOfIt<StoppingTime-200*5.0)
      {
        if (m_OneStep.NbOfIt%200==0)
        {
          cout << "\r" << "Progress " << (unsigned int)r << " " ;
          cout.flush();
        }

        double triggertime = 9.8*200 + m_deltatime*200;
        if ((m_OneStep.NbOfIt>triggertime) &&
            m_TestChangeFoot)
        {
          PatternGeneratorJRL::FootAbsolutePosition aFAP;
          if (m_NbStepsModified<NBOFPREDEFONLINEFOOTSTEPS)
          {
            aFAP.x = OnLineFootSteps[m_NbStepsModified][0];
            aFAP.y = OnLineFootSteps[m_NbStepsModified][1];
            aFAP.z = OnLineFootSteps[m_NbStepsModified][3];
            aFAP.theta = OnLineFootSteps[m_NbStepsModified][2];
          }
          else
          {
            aFAP.x=0.1;
            aFAP.y=0.0;
            aFAP.z=0.0;
            aFAP.theta=0.0;
          }
          double newtime;
          bool stepHandledCorrectly=true;
          try
          {
            m_PGI->ChangeOnLineStep(0.805,aFAP,newtime);
          }
          catch(...)
          {
            cerr << "Step not well handled." << endl;
            stepHandledCorrectly=false;
          }
          if (stepHandledCorrectly)
          {
            m_deltatime += newtime+0.025;
            m_TestChangeFoot=true;
            m_NbStepsModified++;
            if (m_NbStepsModified==360)
              m_TestChangeFoot=false;
          }
          else
          {
            m_deltatime += 0.005;
          }
        }
      }
    }
  }

  void fillInDebugFiles()
  {
    TestObject::fillInDebugFiles();
    TestObject::fillInDebugFilesFull();
    TestObject::generateOpenHRPTrajectories();
  }
};

int PerformTests(int argc, char *argv[])
{
  std::string CompleteName = string(argv[0]);
  unsigned found = CompleteName.find_last_of("/\\");
  std::string TestName =  CompleteName.substr(found+1);
  int TestProfiles[NB_PROFILE] =
  { PROFIL_ANALYTICAL_ONLINE_WALKING,
    PROFIL_ANALYTICAL_SHORT_STRAIGHT_WALKING,
    PROFIL_ANALYTICAL_CLIMBING_STAIRS_10,
    PROFIL_ANALYTICAL_CLIMBING_STAIRS_15,
    PROFIL_ANALYTICAL_GOING_DOWN_STAIRS_10,
    PROFIL_ANALYTICAL_GOING_DOWN_STAIRS_15,
    PROFIL_ANALYTICAL_STEPPING_STONES,
    PROFIL_ANALYTICAL_WALKING_ON_BEAM,
    PROFIL_ANALYTICAL_GO_THROUGH_WALL
  };
  int indexProfile=-1;

  if (TestName.compare(16,6,"OnLine")==0)
    indexProfile=PROFIL_ANALYTICAL_ONLINE_WALKING;
  if (TestName.compare(16,9,"ShortWalk")==0)
    indexProfile=PROFIL_ANALYTICAL_SHORT_STRAIGHT_WALKING;
  if (TestName.compare(16,10,"Climbing10")==0)
    indexProfile=PROFIL_ANALYTICAL_CLIMBING_STAIRS_10;
  if (TestName.compare(16,10,"Climbing15")==0)
    indexProfile=PROFIL_ANALYTICAL_CLIMBING_STAIRS_15;
  if (TestName.compare(16,11,"GoingDown10")==0)
    indexProfile=PROFIL_ANALYTICAL_GOING_DOWN_STAIRS_10;
  if (TestName.compare(16,11,"GoingDown15")==0)
    indexProfile=PROFIL_ANALYTICAL_GOING_DOWN_STAIRS_15;
  if (TestName.compare(16,14,"SteppingStones")==0)
    indexProfile=PROFIL_ANALYTICAL_STEPPING_STONES;
  if (TestName.compare(16,13,"WalkingOnBeam")==0)
    indexProfile=PROFIL_ANALYTICAL_WALKING_ON_BEAM;
  if (TestName.compare(16,13,"GoThroughWall")==0)
    indexProfile=PROFIL_ANALYTICAL_GO_THROUGH_WALL;


  if (indexProfile==-1)
  {
    std::cerr << "CompleteName: " << CompleteName << std::endl;
    std::cerr<< " TestName: " << TestName <<std::endl;
    std::cerr<< "Failure to find the proper indexFile:" << TestName.substr(17,6) << endl;
    exit(-1);
  }
  TestMorisawa2007 aTM2007(argc,argv,
                           TestName,
                           TestProfiles[indexProfile]);
  aTM2007.init();
  try
  {
    if (!aTM2007.doTest(std::cout))
    {
      cout << "Failed test " << indexProfile << endl;
      return -1;
    }
    else
      cout << "Passed test " << indexProfile << endl;
  }
  catch (const char * astr)
  { cerr << "Failed on following error " << astr << std::endl;
    return -1; }
  return 0;
}

int main(int argc, char *argv[])
{
  try
  {
    return PerformTests(argc,argv);
  }
  catch (const std::string& msg)
  {
    std::cerr << msg << std::endl;
  }
  return 1;
}


