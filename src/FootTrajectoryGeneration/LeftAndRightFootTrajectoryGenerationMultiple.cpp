/*
 * Copyright 2008, 2009, 2010,
 *
 * Paul     Evrard
 * Francois Keith
 * Olivier  Stasse
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

/* @doc This object generate all the values for the foot trajectories.
   @ingroup foottrajectorygeneration
   @endgroup
*/

#include "Debug.hh"
/*! Walking Pattern Generator inclusion */
#include "LeftAndRightFootTrajectoryGenerationMultiple.hh"

using namespace PatternGeneratorJRL;

LeftAndRightFootTrajectoryGenerationMultiple::
LeftAndRightFootTrajectoryGenerationMultiple
(SimplePluginManager *lSPM,
 PRFoot * lFoot) : SimplePlugin(lSPM)
{
  m_Omega = 0.0;
  m_Omega2 = 0.0;
  m_Foot = lFoot;

  /** m_StepHeight and m_StepCurving definition -- To Modify*/
  m_StepHeight = 0.05;
  m_StepCurving = 0.15;  // Heuristic value
  m_WayPointThreshold = 0.020 ;  /*19cm (squared) from the line*/

  m_LeftFootTrajectory = new FootTrajectoryGenerationMultiple(lSPM,m_Foot);
  m_RightFootTrajectory = new FootTrajectoryGenerationMultiple(lSPM,m_Foot);

  string aMethodName[4] =
    {":omega",":stepheight", ":singlesupporttime",":doublesupporttime"};

  for (int i=0; i<4; i++)
    {
      if (!RegisterMethod(aMethodName[i]))
        {
          std::cerr << "Unable to register " << aMethodName << std::endl;
        }
    }

  {
    for(unsigned int i=0;
        i<m_MiddleWayPoint.size();
        m_MiddleWayPoint[i++]=0.0);
  } ;

  wayPoint.resize(2,0.0);

}

PRFoot *
LeftAndRightFootTrajectoryGenerationMultiple::
getFoot() const
{
  return m_Foot;
}

LeftAndRightFootTrajectoryGenerationMultiple::
LeftAndRightFootTrajectoryGenerationMultiple
(const LeftAndRightFootTrajectoryGenerationMultiple & aLRFTGM):
  SimplePlugin(aLRFTGM.getSimplePluginManager())

{
  LeftAndRightFootTrajectoryGenerationMultiple
    (aLRFTGM.getSimplePluginManager(),
     aLRFTGM.getFoot());
  *this = aLRFTGM;
}

LeftAndRightFootTrajectoryGenerationMultiple::
~LeftAndRightFootTrajectoryGenerationMultiple()
{

  if (m_LeftFootTrajectory!=0)
    delete m_LeftFootTrajectory;

  if (m_RightFootTrajectory!=0)
    delete m_RightFootTrajectory;

}

/*! Handling methods for the plugin mecanism. */
void LeftAndRightFootTrajectoryGenerationMultiple::
CallMethod(std::string & Method, std::istringstream &strm)
{
  if (Method==":omega")
    {
      strm >> m_Omega;
    }
  else if (Method==":omega2")
    {
      strm >> m_Omega2;
    }
  else if (Method==":stepheight")
    {
      strm >> m_StepHeight;
    }
  else if (Method==":singlesupporttime")
    {
      strm >> m_SingleSupportTime;
    }
  else if (Method==":doublesupporttime")
    {
      strm >> m_DoubleSupportTime;
    }
}

void LeftAndRightFootTrajectoryGenerationMultiple::
SetAnInterval
(unsigned int IntervalIndex,
 FootTrajectoryGenerationMultiple * aFTGM,
 FootAbsolutePosition &FootInitialPosition,
 FootAbsolutePosition &FootFinalPosition,
 vector<double> MiddlePos)
{

  ODEBUG("Set interval " << IntervalIndex << "/"
         << m_DeltaTj.size() << " : " << m_DeltaTj[IntervalIndex] << " X: ("
         << FootFinalPosition.x << ","
         << FootInitialPosition.x << ","
         << FootInitialPosition.dx << ")("
         << FootFinalPosition.y << ","
         << FootInitialPosition.y << ","
         << FootInitialPosition.dy << ")("
         << FootFinalPosition.z << ","
         << FootInitialPosition.z << ","
         << FootInitialPosition.dz << ")");
  aFTGM->SetNatureInterval(IntervalIndex,FootFinalPosition.stepType);

  double ModulationSupportCoefficient = 0.7;
  double UnlockedSwingPeriod = m_DeltaTj[IntervalIndex] *
    ModulationSupportCoefficient;

  // X axis.
  aFTGM->SetParametersWithInitPosInitSpeedInitAcc
    (IntervalIndex,
     FootTrajectoryGenerationStandard::X_AXIS,
     UnlockedSwingPeriod,
     FootFinalPosition.x,
     FootInitialPosition.x,
     FootInitialPosition.dx,
     FootInitialPosition.ddx,
     MiddlePos);

  // Y axis.
  aFTGM->SetParametersWithInitPosInitSpeedInitAcc
    (IntervalIndex,
     FootTrajectoryGenerationStandard::Y_AXIS,
     UnlockedSwingPeriod,
     FootFinalPosition.y,
     FootInitialPosition.y,
     FootInitialPosition.dy,
     FootInitialPosition.ddy,
     MiddlePos);

  // Z axis.
  aFTGM->SetParametersWithInitPosInitSpeedInitAcc
    (IntervalIndex,
     FootTrajectoryGenerationStandard::Z_AXIS,
     m_DeltaTj[IntervalIndex],
     FootFinalPosition.z,
     FootInitialPosition.z,
     FootInitialPosition.dz,
     FootInitialPosition.ddz,
     MiddlePos);

  // THETA
  aFTGM->SetParametersWithInitPosInitSpeedInitAcc
    (IntervalIndex,
     FootTrajectoryGenerationStandard::THETA_AXIS,
     UnlockedSwingPeriod,
     FootFinalPosition.theta,
     FootInitialPosition.theta,
     FootInitialPosition.dtheta,
     FootInitialPosition.ddtheta);

  // Omega
  aFTGM->SetParametersWithInitPosInitSpeed
    (IntervalIndex,
     FootTrajectoryGenerationStandard::OMEGA_AXIS,
     UnlockedSwingPeriod,
     FootFinalPosition.omega,
     FootInitialPosition.omega,
     FootInitialPosition.domega);

  // Omega 2
  aFTGM->SetParametersWithInitPosInitSpeed
    (IntervalIndex,
     FootTrajectoryGenerationStandard::OMEGA2_AXIS,
     UnlockedSwingPeriod,
     FootFinalPosition.omega2,
     FootInitialPosition.omega2,
     FootInitialPosition.domega2);
}

void LeftAndRightFootTrajectoryGenerationMultiple::
InitializeFromRelativeSteps
(deque<RelativeFootPosition> &RelativeFootPositions,
 FootAbsolutePosition &LeftFootInitialPosition,
 FootAbsolutePosition &RightFootInitialPosition,
 deque<FootAbsolutePosition> &SupportFootAbsoluteFootPositions,
 bool IgnoreFirst, bool Continuity)
{
  ODEBUG("LeftFootInitialPosition.stepType: "
         << LeftFootInitialPosition.stepType
         << " RightFootInitialPosition.stepType: "
         << RightFootInitialPosition.stepType);
  /*! Makes sure the size of the SupportFootAbsolutePositions is the same than
    the relative foot positions. */
  if (SupportFootAbsoluteFootPositions.size()!=
      RelativeFootPositions.size())
    SupportFootAbsoluteFootPositions.resize(RelativeFootPositions.size());

  deque<RelativeFootPosition>::size_type
    lNbOfIntervals = RelativeFootPositions.size();
  /*! It is assumed that a set of relative positions for the support foot
    are given as an input. */
  deque<FootAbsolutePosition> AbsoluteFootPositions;

  /*! Those two variables are needed to compute intermediate
    initial positions for the feet. */
  FootAbsolutePosition LeftFootTmpInitPos,RightFootTmpInitPos;
  /*! Those two variables are needed to compute intermediate
    final positions for the feet. */
  FootAbsolutePosition LeftFootTmpFinalPos,RightFootTmpFinalPos;

  AbsoluteFootPositions.resize(lNbOfIntervals);
  lNbOfIntervals = 2*lNbOfIntervals+1;

  /*! Resize the Left and Right foot trajectories. */
  m_LeftFootTrajectory->SetNumberOfIntervals((int)lNbOfIntervals);
  m_RightFootTrajectory->SetNumberOfIntervals((int)lNbOfIntervals);
  ODEBUG("resize left and right foot trajectories: " << lNbOfIntervals);

  /*! Compute the absolute coordinates of the steps.  */
  double CurrentAbsTheta=0.0,c=0.0,s=0.0;
  Eigen::Matrix<double,2,2> MM;;
  Eigen::Matrix<double,3,3> CurrentSupportFootPosition;;
  CurrentSupportFootPosition.setIdentity();
  Eigen::Matrix<double,2,2> Orientation;;
  Orientation.setIdentity();
  Eigen::Matrix<double,2,1> v;;
  Eigen::Matrix<double,2,1> v2;;

  if (m_DeltaTj.size()!=lNbOfIntervals)
    m_DeltaTj.resize(lNbOfIntervals);

  /*! Who is the first support foot. */
  int SupportFoot=1; // Left

  if (
      // The flying foot is on the left, thus the support foot is on the right.
      // and this is not the beginning of the stepping.
      ((RelativeFootPositions[0].sy>0) && (IgnoreFirst==false)) ||
      // There is no flying foot because this is the beginning of the stepping.
      ((RelativeFootPositions[0].sy<0) && (IgnoreFirst==true))
      )
    {
      ODEBUG("Detect support foot on the right.");
      SupportFoot=-1;
      CurrentAbsTheta = RightFootInitialPosition.theta;
      v2(0,0) = RightFootInitialPosition.x;
      v2(1,0) = RightFootInitialPosition.y;
      CurrentSupportFootPosition(2,2) = RightFootInitialPosition.z;
      // v2(2,0) = RightFootInitialPosition.z;
    }
  else
    {
      ODEBUG("Detect support foot on the left.");
      CurrentAbsTheta = LeftFootInitialPosition.theta;
      v2(0,0) = LeftFootInitialPosition.x;
      v2(1,0) = LeftFootInitialPosition.y;
      CurrentSupportFootPosition(2,2) = LeftFootInitialPosition.z;
      //   v2(2,0) = LeftFootInitialPosition.z;
    }
  ODEBUG("Support Foot : " << v2(0,0) << " "
         << v2(1,0) << " " << CurrentAbsTheta);

  // Initial Position of the current support foot.
  c = cos(CurrentAbsTheta*M_PI/180.0);
  s = sin(CurrentAbsTheta*M_PI/180.0);
  MM(0,0) = Orientation(0,0) = c;
  MM(0,1) = Orientation(0,1) = -s;
  MM(1,0) = Orientation(1,0) = s;
  MM(1,1) = Orientation(1,1) = c;
  // MM(2,0) = Orientation(1,0) = 0;      MM(2,1) = Orientation(1,1) = 0;
  for(int k=0; k<2; k++)
    for(int l=0; l<2; l++)
      CurrentSupportFootPosition(k,l) = MM(k,l);

  for(int k=0; k<2; k++)
    CurrentSupportFootPosition(k,2) = v2(k,0);

  /*! Initialize the temporary initial position. */

  LeftFootTmpInitPos = LeftFootInitialPosition;
  RightFootTmpInitPos = RightFootInitialPosition;

  /* Keep track of the interval index once this is
     for single support, once for double support */
  int IntervalIndex=0;
  ODEBUG("LeftFootTmpInitPos.x " << LeftFootTmpInitPos.x << endl <<
         "LeftFootTmpInitPos.y " << LeftFootTmpInitPos.y << endl <<
         "LeftFootTmpInitPos.z " << LeftFootTmpInitPos.z << endl <<
         "LeftFootTmpInitPos.dx " << LeftFootTmpInitPos.dx << endl <<
         "LeftFootTmpInitPos.dy " << LeftFootTmpInitPos.dy << endl <<
         "LeftFootTmpInitPos.dz " << LeftFootTmpInitPos.dz << endl );


  ODEBUG("RightFootTmpInitPos.x " << RightFootTmpInitPos.x << endl <<
         "RightFootTmpInitPos.y " << RightFootTmpInitPos.y << endl <<
         "RightFootTmpInitPos.z " << RightFootTmpInitPos.z << endl <<
         "RightFootTmpInitPos.dx " << RightFootTmpInitPos.dx << endl <<
         "RightFootTmpInitPos.dy " << RightFootTmpInitPos.dy << endl <<
         "RightFootTmpInitPos.dz " << RightFootTmpInitPos.dz << endl );

  bool FirstIntervalIsSingleSupport = true;
  if (LeftFootInitialPosition.stepType>10)
    FirstIntervalIsSingleSupport = false;

  ODEBUG("CurrentSupportFootPosition: " << CurrentSupportFootPosition);
  ODEBUG("RelativeFootPositions: " << RelativeFootPositions.size());
  for(unsigned int i=0; i<RelativeFootPositions.size(); i++)
    {
      if ((i!=0) || (FirstIntervalIsSingleSupport==false))
        {
          /*! At this stage the phase of double support is dealt with */
          ODEBUG("Double support phase");
          //LeftFootTmpInitPos.z = CurrentSupportFootPosition(2,2);
          // 0.0;//LeftFootTmpInitPos.z + LeftFootTmpFinalPos.z/1.5;
          LeftFootTmpInitPos.dz = 0;
          LeftFootTmpInitPos.stepType=11;

          SetAnInterval(IntervalIndex,m_LeftFootTrajectory,
                        LeftFootTmpInitPos,
                        LeftFootTmpInitPos);

          /// RightFootTmpInitPos.z = CurrentSupportFootPosition(2,2);
          RightFootTmpInitPos.dz = 0;
          RightFootTmpInitPos.stepType=9;
          SetAnInterval(IntervalIndex,m_RightFootTrajectory,
                        RightFootTmpInitPos,
                        RightFootTmpInitPos);
          ODEBUG("LeftFootTmpInitPos.stepType="<<LeftFootTmpInitPos.stepType);
          ODEBUG("RightFootTmpInitPos.stepType="<<RightFootTmpInitPos.stepType);
          ODEBUG("End of Double support phase");

          IntervalIndex++;


          ODEBUG("It: " << i << " Double support Phase :" << endl <<
                 "\t Init Left: ( " <<
                 LeftFootTmpInitPos.x << " , " <<
                 LeftFootTmpInitPos.y << " , " <<
                 LeftFootTmpInitPos.z <<" ) " <<
                 endl << "Right : ( " <<
                 RightFootTmpInitPos.x << " , " <<
                 RightFootTmpInitPos.y << " , " <<
                 RightFootTmpInitPos.z <<") " << endl <<
                 "\t Final Left: ( " <<
                 LeftFootTmpFinalPos.x << " , " <<
                 LeftFootTmpFinalPos.y << " , " <<
                 RightFootTmpFinalPos.z << " ) " <<
                 endl << "Right : ( " <<
                 RightFootTmpFinalPos.x << " , " <<
                 RightFootTmpFinalPos.y << " , " <<
                 RightFootTmpFinalPos.z << " ) " << endl <<
                 "\t RelativeFootPosition: ( " <<
                 RelativeFootPositions[i].sx << " , " <<
                 RelativeFootPositions[i].sy << " , " <<
                 RelativeFootPositions[i].sz << " , " <<
                 RelativeFootPositions[i].theta << " )");
        }

      /*! Compute Orientation matrix related to the relative orientation
        of the support foot */
      c = cos(RelativeFootPositions[i].theta*M_PI/180.0);
      s = sin(RelativeFootPositions[i].theta*M_PI/180.0);
      MM(0,0) = c;
      MM(0,1) = -s;
      MM(1,0) = s;
      MM(1,1) = c;
      // MM(2,0) = 0;      MM(2,1) = 0; MM(2,2) = 0;
      /*! Update the orientation */
      CurrentAbsTheta+= RelativeFootPositions[i].theta;

      /*! Extract the current absolute orientation matrix. */
      for(int k=0; k<2; k++)
        for(int l=0; l<2; l++)
          Orientation(k,l) = CurrentSupportFootPosition(k,l);

      /*! Put in a vector form the translation of the relative foot. */
      v(0,0) = RelativeFootPositions[i].sx;
      v(1,0) = RelativeFootPositions[i].sy;
      // v(2,0) = RelativeFootPositions[i].sz;
      /*! Compute the new orientation of the foot vector. */
      Orientation = MM*Orientation;

      v2 = Orientation*v;

      /*! Update the world coordinates of the support foot. */
      if ((!IgnoreFirst) || (i>0))
        {
          for(int k=0; k<2; k++)
            for(int l=0; l<2; l++)
              CurrentSupportFootPosition(k,l) = Orientation(k,l);

          for(int k=0; k<2; k++)
            CurrentSupportFootPosition(k,2) += v2(k,0);

          CurrentSupportFootPosition(2,2) += RelativeFootPositions[i].sz;
        }
      AbsoluteFootPositions[i].x = CurrentSupportFootPosition(0,2);
      AbsoluteFootPositions[i].y = CurrentSupportFootPosition(1,2);
      AbsoluteFootPositions[i].z = CurrentSupportFootPosition(2,2);
      AbsoluteFootPositions[i].theta = CurrentAbsTheta;

      ODEBUG("CSFP:" << CurrentSupportFootPosition(0,2) << " "
             << CurrentSupportFootPosition(1,2) << " "
             << CurrentSupportFootPosition(2,2) << " "
             << CurrentAbsTheta);

      /*! We deal with the single support phase,
        i.e. the target of the next single support phase
        is the current target of the swinging foot. */
      if ((!IgnoreFirst) || (i>0))
        {
          if (SupportFoot==1)
            {
              /*! The current support foot is the left one.*/
              RightFootTmpFinalPos.x = CurrentSupportFootPosition(0,2);
              RightFootTmpFinalPos.y = CurrentSupportFootPosition(1,2);
              RightFootTmpFinalPos.z = CurrentSupportFootPosition(2,2);
              RightFootTmpFinalPos.theta = CurrentAbsTheta;
              RightFootTmpFinalPos.omega = m_Omega;
              RightFootTmpFinalPos.omega2 = m_Omega2;
              RightFootTmpFinalPos.dx = 0.0;
              RightFootTmpFinalPos.dy = 0.0;
              RightFootTmpFinalPos.dz = 0.0;
              RightFootTmpFinalPos.dtheta = 0.0;
              RightFootTmpFinalPos.domega = 0.0;
              RightFootTmpFinalPos.domega2 = 0.0;
              RightFootTmpFinalPos.stepType = 1;
              LeftFootTmpFinalPos = LeftFootTmpInitPos;
              LeftFootTmpFinalPos.dz = 0.0;
              LeftFootTmpFinalPos.stepType = -1;
            }
          else
            {
              /*! The current support foot is the right one.*/
              LeftFootTmpFinalPos.x = CurrentSupportFootPosition(0,2);
              LeftFootTmpFinalPos.y = CurrentSupportFootPosition(1,2);
              LeftFootTmpFinalPos.z = CurrentSupportFootPosition(2,2);
              LeftFootTmpFinalPos.theta = CurrentAbsTheta;
              LeftFootTmpFinalPos.omega = m_Omega;
              LeftFootTmpFinalPos.omega2 = m_Omega2;
              LeftFootTmpFinalPos.dx = 0.0;
              LeftFootTmpFinalPos.dy = 0.0;
              LeftFootTmpFinalPos.dz = 0.0;
              LeftFootTmpFinalPos.dtheta = 0.0;
              LeftFootTmpFinalPos.domega = 0.0;
              LeftFootTmpFinalPos.domega2 = 0.0;
              LeftFootTmpFinalPos.stepType = 1;

              RightFootTmpFinalPos = RightFootTmpInitPos;
              // RightFootTmpFinalPos.z = 0.0;
              RightFootTmpFinalPos.dz = 0.0;
              RightFootTmpFinalPos.stepType = -1;
            }
        }
      else
        {
          LeftFootTmpFinalPos = LeftFootTmpInitPos;

          LeftFootTmpFinalPos.z = CurrentSupportFootPosition(2,2);
          LeftFootTmpFinalPos.omega = m_Omega;
          LeftFootTmpFinalPos.omega2 = m_Omega2;
          LeftFootTmpFinalPos.dx = LeftFootTmpInitPos.dx = 0.0;
          LeftFootTmpFinalPos.dy = LeftFootTmpInitPos.dy =0.0;
          LeftFootTmpFinalPos.dz = LeftFootTmpInitPos.dz =0.0;
          LeftFootTmpFinalPos.domega = LeftFootTmpInitPos.domega =0.0;
          LeftFootTmpFinalPos.domega2 = LeftFootTmpInitPos.domega2 =0.0;
          LeftFootTmpFinalPos.stepType = 11;

          RightFootTmpFinalPos = RightFootTmpInitPos;
          RightFootTmpFinalPos.z = CurrentSupportFootPosition(2,2);
          RightFootTmpFinalPos.omega = m_Omega;
          RightFootTmpFinalPos.omega2 = m_Omega2 ;
          RightFootTmpFinalPos.dx = RightFootTmpInitPos.dx = 0.0;
          RightFootTmpFinalPos.dy = RightFootTmpInitPos.dy =0.0;
          RightFootTmpFinalPos.dz = RightFootTmpInitPos.dz =0.0;
          RightFootTmpFinalPos.domega = RightFootTmpInitPos.domega =0.0;
          RightFootTmpFinalPos.domega2 = RightFootTmpInitPos.domega2 =0.0;
          RightFootTmpFinalPos.stepType = 9;

        }


      if ((i!=0)|| (Continuity))
        {
          {
            // verify auto collision
            Eigen::Matrix<double,2,1> currSupp;
            Eigen::Matrix<double,2,1> InitPos;
            Eigen::Matrix<double,2,1> FinalPos;
            Eigen::Matrix<double,2,1> relWayPoint;
            if (SupportFoot==1)
              {
                currSupp(0) = LeftFootTmpInitPos.x ;
                currSupp(1) = LeftFootTmpInitPos.y ;
                InitPos(0)  = RightFootTmpInitPos.x ;
                InitPos(1)  = RightFootTmpInitPos.y ;
                FinalPos(0) = RightFootTmpFinalPos.x ;
                FinalPos(1) = RightFootTmpFinalPos.y ;
                relWayPoint(0)=0.0 ;
                relWayPoint(1)=-m_StepCurving;
              }
            else
              {
                currSupp(0) = RightFootTmpInitPos.x ;
                currSupp(1) = RightFootTmpInitPos.y ;
                InitPos(0)  = LeftFootTmpInitPos.x ;
                InitPos(1)  = LeftFootTmpInitPos.y ;
                FinalPos(0) = LeftFootTmpFinalPos.x ;
                FinalPos(1) = LeftFootTmpFinalPos.y ;
                relWayPoint(0)=0.0 ;
                relWayPoint(1)=m_StepCurving;
              }

            double dx, dy, dc, distSquareToLine ;
            dx = InitPos(1) - FinalPos(1) ;
            dy = FinalPos(0) - InitPos(0) ;
            m_MiddleWayPoint.resize(2);
            {
              for(unsigned int i=0;
                  i<m_MiddleWayPoint.size();
                  m_MiddleWayPoint[i++]=0.0);
            };
            if ( dx*dx>=1e-6 || dy*dy>=1e-6 )// not moving implies no collision
              {
                dc = -(dx * InitPos(0) + dy *InitPos(1)) ;
                distSquareToLine = (dx*currSupp(0)  + dy*currSupp(1) + dc)*
                  (dx*currSupp(0)  + dy*currSupp(1) + dc)/(dx*dx + dy*dy);
                if( distSquareToLine < m_WayPointThreshold )
                  {
                    double x(currSupp(0)), y(currSupp(1)),
                      x0((FinalPos(0)+InitPos(0))*0.5),
                      y0((FinalPos(1)+InitPos(1))*0.5),
                      R2((dx*dx+dy+dy)*0.5*0.5);
                    bool autocollision = (x-x0)*(x-x0)+(y-y0)*(y-y0)<=R2;
                    if( autocollision )
                      {
                        m_MiddleWayPoint = Orientation*relWayPoint + currSupp  ;
                      }
                  }
              }
          }
          std::vector<double> leftMidPos (3,0);
          std::vector<double> rightMidPos (3,0);
          if (SupportFoot==1)
            {
              // X offset to curve the step
              rightMidPos[0] = m_MiddleWayPoint[0];
              // Y offset to curve the step
              rightMidPos[1] = m_MiddleWayPoint[1];
              // Z offset
              if(RightFootTmpInitPos.dx*RightFootTmpInitPos.dx<=0.000001)
                {
                  rightMidPos[2] = m_StepHeight ;
                }
            }
          else
            {
              // X offset to curve the step
              leftMidPos[0] = m_MiddleWayPoint[0];
              // Y offset to curve the step
              leftMidPos[1] = m_MiddleWayPoint[1];
              // Z offset
              if(LeftFootTmpInitPos.dx*LeftFootTmpInitPos.dx<=0.000001)
                {
                  leftMidPos[2] = m_StepHeight ;
                }
            }

          /* Initialize properly the interval in single support phase */
          ODEBUG("Single support phase");
          ODEBUG("LeftFootTmpInitPos.stepType="<<LeftFootTmpInitPos.stepType);
          ODEBUG("LeftFootTmpFinalPos.stepType="<<LeftFootTmpFinalPos.stepType);
          SetAnInterval(IntervalIndex,m_LeftFootTrajectory,
                        LeftFootTmpInitPos,
                        LeftFootTmpFinalPos,leftMidPos);


          ODEBUG("LeftInit: ( " << LeftFootTmpInitPos.x << " , "
                 << LeftFootTmpInitPos.y << " , "
                 << LeftFootTmpInitPos.z << " ) ( "
                 << LeftFootTmpInitPos.dx << " , "
                 << LeftFootTmpInitPos.dy << " , "
                 << LeftFootTmpInitPos.dz << " ) "
                 << endl << "LeftFinal : ( "
                 << LeftFootTmpFinalPos.x << " , "
                 << LeftFootTmpFinalPos.y << " , "
                 << LeftFootTmpFinalPos.z << " ) ( "
                 << LeftFootTmpFinalPos.dx << " , "
                 << LeftFootTmpFinalPos.dy << " , "
                 << LeftFootTmpFinalPos.dz << " ) " );

          ODEBUG("RightFootTmpInitPos.stepType=" <<
                 RightFootTmpInitPos.stepType);
          ODEBUG("RightFootTmpFinalPos.stepType=" <<
                 RightFootTmpFinalPos.stepType);
          ODEBUG("End of Single support phase");
          SetAnInterval(IntervalIndex,m_RightFootTrajectory,
                        RightFootTmpInitPos,
                        RightFootTmpFinalPos,rightMidPos);

          if (SupportFoot==1)
            RightFootTmpFinalPos.z = CurrentSupportFootPosition(2,2);
          else
            LeftFootTmpFinalPos.z = CurrentSupportFootPosition(2,2);
          ODEBUG("RightInit: ( " << RightFootTmpInitPos.x << " , "
                 << RightFootTmpInitPos.y << " , "
                 << RightFootTmpInitPos.z << " ) ( "
                 << RightFootTmpInitPos.dx << " , "
                 << RightFootTmpInitPos.dy << " , "
                 << RightFootTmpInitPos.dz << " ) "
                 << endl << "RightFinal : ( "
                 << RightFootTmpFinalPos.x << " , "
                 << RightFootTmpFinalPos.y << " , "
                 << RightFootTmpFinalPos.z << " ) ( "
                 << RightFootTmpFinalPos.dx << " , "
                 << RightFootTmpFinalPos.dy << " , "
                 << RightFootTmpFinalPos.dz << " ) " );
          // Switch from single support to double support.
          IntervalIndex++;
        }

      if ((!Continuity) && ((i==0) || (i==RelativeFootPositions.size()-1)))
        {
          /*! At this stage the phase of double support is dealt with */
          unsigned int limitk=1;

          /*! If we are at the end a second double support phase 
            has to be added. */
          if (i==RelativeFootPositions.size()-1)
            limitk=2;

          for(unsigned int lk=0; lk<limitk; lk++)
            {
              LeftFootTmpFinalPos.z = CurrentSupportFootPosition(2,2);
              LeftFootTmpFinalPos.dz = 0;
              LeftFootTmpFinalPos.stepType = -1;

              SetAnInterval(IntervalIndex,m_LeftFootTrajectory,
                            LeftFootTmpFinalPos,
                            LeftFootTmpFinalPos);
              RightFootTmpFinalPos.z = CurrentSupportFootPosition(2,2);
              RightFootTmpFinalPos.dz = 0;
              RightFootTmpFinalPos.stepType = -1;
              SetAnInterval(IntervalIndex,m_RightFootTrajectory,
                            RightFootTmpFinalPos,
                            RightFootTmpFinalPos);
              IntervalIndex++;
            }
        }


      /* The final position become the new initial position */
      LeftFootTmpInitPos = LeftFootTmpFinalPos;
      RightFootTmpInitPos = RightFootTmpFinalPos;


      /* Populate the set of support foot absolute positions */
      SupportFootAbsoluteFootPositions[i].x = CurrentSupportFootPosition(0,2);
      SupportFootAbsoluteFootPositions[i].y = CurrentSupportFootPosition(1,2);
      SupportFootAbsoluteFootPositions[i].z = CurrentSupportFootPosition(2,2);
      SupportFootAbsoluteFootPositions[i].theta = CurrentAbsTheta;


      if ((!IgnoreFirst) || (i>0))
        SupportFoot=-SupportFoot;
    }

  /*! This part initializes correctly the last two intervals
    if the system is in real-time foot modification. In this case,
    the representation of the intervals shift from:
    ONE DOUBLE SUPPORT STARTING PHASE - 1st foot single support phase -
    double support phase - 2nd foot single support phase
    - double support phase - 3rd single support phase -
    ending double support phase
    to
    1st foot single support phase - double support phase -
    2nd foot single support phase
    - double support phase - 3rd single support phase
    Two intervals are missing and should be set by
    default to the end position of the feet
    if Continuity is set to true, and if the number of intervals
    so far is the number of
    intervals minus 2.
  */
  if ((Continuity) && (IntervalIndex==(int)(m_DeltaTj.size()-2)))
    {
      for(unsigned int lk=0; lk<2; lk++)
        {
          LeftFootTmpFinalPos.z = CurrentSupportFootPosition(2,2);
          LeftFootTmpFinalPos.dz = 0;
          SetAnInterval(IntervalIndex,m_LeftFootTrajectory,
                        LeftFootTmpFinalPos,
                        LeftFootTmpFinalPos);
          RightFootTmpFinalPos.z = CurrentSupportFootPosition(2,2);
          RightFootTmpFinalPos.dz = 0;
          SetAnInterval(IntervalIndex,m_RightFootTrajectory,
                        RightFootTmpFinalPos,
                        RightFootTmpFinalPos);
          IntervalIndex++;
        }

    }

}

void LeftAndRightFootTrajectoryGenerationMultiple::
ComputeAbsoluteStepsFromRelativeSteps
(deque<RelativeFootPosition> &RelativeFootPositions,
 FootAbsolutePosition &LeftFootInitialPosition,
 FootAbsolutePosition &RightFootInitialPosition,
 deque<FootAbsolutePosition> &SupportFootAbsoluteFootPositions)
{
  FootAbsolutePosition aSupportFootAbsolutePosition;

  if (RelativeFootPositions[0].sy>0)
    {
      // The flying foot is on the left, thus the support foot is on the right.
      ODEBUG("Detect support foot on the right.");
      aSupportFootAbsolutePosition = RightFootInitialPosition;
    }
  else
    {
      ODEBUG("Detect support foot on the left.");
      aSupportFootAbsolutePosition = LeftFootInitialPosition;
    }
  aSupportFootAbsolutePosition.stepType=-1;

  ComputeAbsoluteStepsFromRelativeSteps(RelativeFootPositions,
                                        aSupportFootAbsolutePosition,
                                        SupportFootAbsoluteFootPositions);

}
void LeftAndRightFootTrajectoryGenerationMultiple::
ComputeAbsoluteStepsFromRelativeSteps
(deque<RelativeFootPosition> &RelativeFootPositions,
 FootAbsolutePosition &SupportFootInitialAbsolutePosition,
 deque<FootAbsolutePosition> &SupportFootAbsoluteFootPositions)
{
  /*! Makes sure the size of the SupportFootAbsolutePositions is the same than
    the relative foot positions. */
  if (SupportFootAbsoluteFootPositions.size()!=
      RelativeFootPositions.size())
    SupportFootAbsoluteFootPositions.resize(RelativeFootPositions.size());

  long unsigned int lNbOfIntervals = RelativeFootPositions.size();
  /*! It is assumed that a set of relative positions for the support foot
    are given as an input. */
  deque<FootAbsolutePosition> AbsoluteFootPositions;

  AbsoluteFootPositions.resize(lNbOfIntervals);
  lNbOfIntervals = 2*lNbOfIntervals+1;

  /*! Compute the absolute coordinates of the steps.  */
  double CurrentAbsTheta=0.0,c=0.0,s=0.0;
  Eigen::Matrix<double,2,2> MM;;
  Eigen::Matrix<double,3,3> CurrentSupportFootPosition;;
  CurrentSupportFootPosition.setIdentity();
  Eigen::Matrix<double,2,2> Orientation;;
  Orientation.setIdentity();
  Eigen::Matrix<double,2,1> v;;
  Eigen::Matrix<double,2,1> v2;;

  ODEBUG("Detect support foot on the right.");
  CurrentAbsTheta = SupportFootInitialAbsolutePosition.theta;
  v2(0,0) = SupportFootInitialAbsolutePosition.x;
  v2(1,0) = SupportFootInitialAbsolutePosition.y;

  // Initial Position of the current support foot.
  c = cos(CurrentAbsTheta*M_PI/180.0);
  s = sin(CurrentAbsTheta*M_PI/180.0);
  MM(0,0) = Orientation(0,0) = c;
  MM(0,1) = Orientation(0,1) = -s;
  MM(1,0) = Orientation(1,0) = s;
  MM(1,1) = Orientation(1,1) = c;

  for(int k=0; k<2; k++)
    for(int l=0; l<2; l++)
      CurrentSupportFootPosition(k,l) = MM(k,l);

  for(int k=0; k<2; k++)
    CurrentSupportFootPosition(k,2) = v2(k,0);


  /* Keep track of the interval index once this is
     for single support, once for double support */

  ODEBUG("CurrentSupportFootPosition: " << CurrentSupportFootPosition);
  ODEBUG("RelativeFootPositions: " << RelativeFootPositions.size());

  for(unsigned int i=0; i<RelativeFootPositions.size(); i++)
    {

      ODEBUG( i << " : " <<
              RelativeFootPositions[i].sx << " " <<
              RelativeFootPositions[i].sy);

      /*! Compute Orientation matrix related to the relative orientation
        of the support foot */
      c = cos(RelativeFootPositions[i].theta*M_PI/180.0);
      s = sin(RelativeFootPositions[i].theta*M_PI/180.0);
      MM(0,0) = c;
      MM(0,1) = -s;
      MM(1,0) = s;
      MM(1,1) = c;

      /*! Update the orientation */
      CurrentAbsTheta+= RelativeFootPositions[i].theta;
      CurrentAbsTheta = fmod(CurrentAbsTheta,180.0);

      /*! Extract the current absolute orientation matrix. */
      for(int k=0; k<2; k++)
        for(int l=0; l<2; l++)
          Orientation(k,l) = CurrentSupportFootPosition(k,l);

      /*! Put in a vector form the translation of the relative foot. */
      v(0,0) = RelativeFootPositions[i].sx;
      v(1,0) = RelativeFootPositions[i].sy;

      /*! Compute the new orientation of the foot vector. */
      Orientation = MM*Orientation;
      v2 = Orientation*v;

      /*! Update the world coordinates of the support foot. */
      for(int k=0; k<2; k++)
        for(int l=0; l<2; l++)
          CurrentSupportFootPosition(k,l) = Orientation(k,l);

      for(int k=0; k<2; k++)
        CurrentSupportFootPosition(k,2) += v2(k,0);

      CurrentSupportFootPosition(2,2) = SupportFootInitialAbsolutePosition.z ;
      CurrentSupportFootPosition(2,2) += RelativeFootPositions[i].sz;

      AbsoluteFootPositions[i].x = CurrentSupportFootPosition(0,2);
      AbsoluteFootPositions[i].y = CurrentSupportFootPosition(1,2);
      AbsoluteFootPositions[i].z = CurrentSupportFootPosition(2,2);
      AbsoluteFootPositions[i].theta = CurrentAbsTheta;

      ODEBUG("CSFP:" << CurrentSupportFootPosition(0,2) << " "
             << CurrentSupportFootPosition(1,2));

      /* Populate the set of support foot absolute positions */
      SupportFootAbsoluteFootPositions[i].x = CurrentSupportFootPosition(0,2);
      SupportFootAbsoluteFootPositions[i].y = CurrentSupportFootPosition(1,2);
      SupportFootAbsoluteFootPositions[i].z = CurrentSupportFootPosition(2,2);
      SupportFootAbsoluteFootPositions[i].theta = CurrentAbsTheta;
      SupportFootAbsoluteFootPositions[i].stepType = -1;

    }
}

void LeftAndRightFootTrajectoryGenerationMultiple::
ChangeRelStepsFromAbsSteps
(deque<RelativeFootPosition> &RelativeFootPositions,
 FootAbsolutePosition &SupportFootInitialPosition,
 deque<FootAbsolutePosition> &SupportFootAbsoluteFootPositions,
 unsigned int ChangedInterval)
{
  if (ChangedInterval>=SupportFootAbsoluteFootPositions.size())
    {
      LTHROW("Pb: ChangedInterval is after the size of absolute foot stack.");
      return;
    }

  Eigen::Matrix3d KM1;
  KM1.setIdentity();
  Eigen::Matrix3d K;
  K.setIdentity();
  Eigen::Matrix3d KP1;
  KP1.setIdentity();

  double thetakm1,xkm1,ykm1,c,s;

  // Change the previous relative position.
  if (ChangedInterval==0)
    {
      thetakm1= SupportFootInitialPosition.theta;
      xkm1 = SupportFootInitialPosition.x;
      ykm1 = SupportFootInitialPosition.y;
    }
  else
    {
      thetakm1= SupportFootAbsoluteFootPositions[ChangedInterval-1].theta;
      xkm1 = SupportFootAbsoluteFootPositions[ChangedInterval-1].x;
      ykm1 = SupportFootAbsoluteFootPositions[ChangedInterval-1].y;
    }
  ODEBUG("Changed interval: " << ChangedInterval);
  ODEBUG("K-1 position: " << xkm1 << " " << ykm1 << " " << thetakm1 );
  c = cos(thetakm1*M_PI/180.0);
  s = sin(thetakm1*M_PI/180.0);
  KM1(0,0) = c;
  KM1(0,1) = -s;
  KM1(0,2) = xkm1;
  KM1(1,0) = s;
  KM1(1,1) = c;
  KM1(1,2) = ykm1;

  double thetak,xk,yk;
  thetak= SupportFootAbsoluteFootPositions[ChangedInterval].theta;
  xk = SupportFootAbsoluteFootPositions[ChangedInterval].x;
  yk = SupportFootAbsoluteFootPositions[ChangedInterval].y;

  ODEBUG("K position: " << xk << " " << yk << " " << thetak );
  c = cos(thetak*M_PI/180.0);
  s = sin(thetak*M_PI/180.0);
  K(0,0) = c;
  K(0,1) = -s;
  K(0,2) = xk;
  K(1,0) = s;
  K(1,1) = c;
  K(1,2) = yk;

  Eigen::Matrix3d iKM1;
  iKM1=KM1.inverse();
  Eigen::Matrix3d relMotionM1;

  relMotionM1=iKM1+K;

  RelativeFootPositions[ChangedInterval].sx = relMotionM1(0,2);
  RelativeFootPositions[ChangedInterval].sy = relMotionM1(1,2);
  RelativeFootPositions[ChangedInterval].theta =
    atan2(relMotionM1(1,0),relMotionM1(0,0));

  double thetakp1,xkp1,ykp1;

  // Change the next relative position
  if (ChangedInterval<SupportFootAbsoluteFootPositions.size()-1)
    {
      thetakp1= SupportFootAbsoluteFootPositions[ChangedInterval+1].theta;
      xkp1 = SupportFootAbsoluteFootPositions[ChangedInterval+1].x;
      ykp1 = SupportFootAbsoluteFootPositions[ChangedInterval+1].y;

      c = cos(thetakp1*M_PI/180.0);
      s = sin(thetakp1*M_PI/180.0);
      KP1(0,0) = c;
      KP1(0,1) = -s;
      KP1(0,2) = xkp1;
      KP1(1,0) = s;
      KP1(1,1) = c;
      KP1(1,2) = ykp1;

      Eigen::Matrix3d iK;
      iK=K.inverse();
      Eigen::Matrix3d relMotionP1;

      relMotionP1=iK+KP1;

      RelativeFootPositions[ChangedInterval+1].sx = relMotionP1(0,2);
      RelativeFootPositions[ChangedInterval+1].sy = relMotionP1(1,2);
      RelativeFootPositions[ChangedInterval+1].theta =
        atan2(relMotionP1(1,0),relMotionP1(0,0));

    }

  ODEBUG("KP1 position: " << xkp1 << " " << ykp1 << " " << thetakp1 );
  ODEBUG("Changed intervals : " << ChangedInterval-1 << " "
         << ChangedInterval << " " << ChangedInterval + 1);
}

bool LeftAndRightFootTrajectoryGenerationMultiple::
ComputeAnAbsoluteFootPosition
(int LeftOrRight,
 double time,
 FootAbsolutePosition & aFAP)
{

  ODEBUG("Left (1) or right (-1) : " <<  LeftOrRight);

  if (LeftOrRight==1)
    {
      /*! Left or Right Foot Step - 0: Left Foot; 1: Right Foot*/
      //aFAP.LeftOrRightFoot = 0; // Left Foot
      bool r = m_LeftFootTrajectory->Compute(time,aFAP);
      if (!r)
        {
          std::cerr << "Unable to compute left foot abs pos at time "
                    <<time << std::endl;
          LTHROW("Pb in computing absolute foot position");
        }
      return r;
    }
  else
    {
      /*! Left or Right Foot Step - 0: Left Foot; 1: Right Foot*/
      //aFAP.LeftOrRightFoot = 1; // Right Foot
      bool r = m_RightFootTrajectory->Compute(time,aFAP);
      if (!r)
        {
          std::cerr
            << "Unable to compute right foot abs pos at time " <<time
            << std::endl;
          LTHROW("Pb in computing absolute foot position");
        }
      return r;
    }

  return false;
}

bool LeftAndRightFootTrajectoryGenerationMultiple::
ComputeAnAbsoluteFootPosition
(int LeftOrRight,
 double time,
 FootAbsolutePosition & aFAP,
 unsigned int IndexInterval)
{

  ODEBUG(this << " " << m_LeftFootTrajectory << " "
         << m_RightFootTrajectory);

  if (LeftOrRight==1)
    return m_LeftFootTrajectory->Compute(time,aFAP,IndexInterval);
  else
    return m_RightFootTrajectory->Compute(time,aFAP,IndexInterval);

  return false;
}
/*
  bool LeftAndRightFootTrajectoryGenerationMultiple::
  ComputeAnAbsoluteFootPosition
  (int LeftOrRight,
  double time,
  deque<FootAbsolutePosition> & adFAP,
  unsigned int IndexInterval)
  {

  ODEBUG(this << " " << m_LeftFootTrajectory << " " << m_RightFootTrajectory);

  if (LeftOrRight==1)
  return m_LeftFootTrajectory->Compute(time,adFAP,IndexInterval);
  else
  return m_RightFootTrajectory->Compute(time,adFAP,IndexInterval);

  return false;
  }*/

void LeftAndRightFootTrajectoryGenerationMultiple::
SetDeltaTj(vector<double> &aDeltaTj)
{
  ODEBUG("SetDeltaTj :" << aDeltaTj.size() << " " << aDeltaTj[0]);

  m_DeltaTj = aDeltaTj;
  if (m_LeftFootTrajectory!=0)
    m_LeftFootTrajectory->SetTimeIntervals(m_DeltaTj);
  if (m_RightFootTrajectory!=0)
    m_RightFootTrajectory->SetTimeIntervals(m_DeltaTj);

}

void LeftAndRightFootTrajectoryGenerationMultiple::
DisplayIntervals()
{
  ODEBUG("Left intervals");
  m_LeftFootTrajectory->DisplayIntervals();
  ODEBUG("Right intervals");
  m_RightFootTrajectory->DisplayIntervals();
}

void LeftAndRightFootTrajectoryGenerationMultiple::
GetDeltaTj(vector<double> &aDeltaTj) const
{
  aDeltaTj = m_DeltaTj;
}

void LeftAndRightFootTrajectoryGenerationMultiple::
SetStepHeight(double aStepHeight)
{
  m_StepHeight = aStepHeight;
}

double LeftAndRightFootTrajectoryGenerationMultiple::
GetStepHeight() const
{
  return m_StepHeight;
}

void LeftAndRightFootTrajectoryGenerationMultiple::
SetStepCurving(double aStepCurving)
{
  m_StepCurving = aStepCurving;
}

double LeftAndRightFootTrajectoryGenerationMultiple::
GetStepCurving() const
{
  return m_StepCurving;
}

double LeftAndRightFootTrajectoryGenerationMultiple::
GetAbsoluteTimeReference() const
{
  double res=0.0;
  double LeftATR=0.0,RightATR=0.0;
  if (m_LeftFootTrajectory!=0)
    LeftATR = m_LeftFootTrajectory->GetAbsoluteTimeReference();
  if (m_RightFootTrajectory!=0)
    RightATR = m_RightFootTrajectory->GetAbsoluteTimeReference();
  if (LeftATR!=RightATR)
    res=-1;
  else
    res = LeftATR;

  return res;
}

void LeftAndRightFootTrajectoryGenerationMultiple::
SetAbsoluteTimeReference(double anATR)
{
  if (m_LeftFootTrajectory!=0)
    m_LeftFootTrajectory->SetAbsoluteTimeReference(anATR);
  if (m_RightFootTrajectory!=0)
    m_RightFootTrajectory->SetAbsoluteTimeReference(anATR);
}

LeftAndRightFootTrajectoryGenerationMultiple &
LeftAndRightFootTrajectoryGenerationMultiple::operator=
(const LeftAndRightFootTrajectoryGenerationMultiple & aLRFTGM)
{
  ODEBUG("Went through this.");
  if (this == &aLRFTGM)
    return *this;

  SetAbsoluteTimeReference(aLRFTGM.GetAbsoluteTimeReference());
  SetStepHeight(aLRFTGM.GetStepHeight());
  SetStepCurving(aLRFTGM.GetStepCurving());
  aLRFTGM.GetDeltaTj(m_DeltaTj);

  *m_LeftFootTrajectory = *(aLRFTGM.getLeftFootTrajectory());
  *m_RightFootTrajectory = *(aLRFTGM.getRightFootTrajectory());

  m_Foot = aLRFTGM.getFoot();

  return *this;

}

FootTrajectoryGenerationMultiple *
LeftAndRightFootTrajectoryGenerationMultiple::
getLeftFootTrajectory() const
{
  return m_LeftFootTrajectory;
}

FootTrajectoryGenerationMultiple *
LeftAndRightFootTrajectoryGenerationMultiple::
getRightFootTrajectory() const
{
  return m_RightFootTrajectory;
}
