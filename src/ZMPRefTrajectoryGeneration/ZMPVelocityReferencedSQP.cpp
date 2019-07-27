/*
 * Copyright 2010,
 *
 *
 * Maximilien Naveau
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

/*! This object generate all the values for the foot trajectories,
  and the desired ZMP based on a sequence of steps following a SQP
  formulation using QPoases solver as proposed by Naveau

  M. Naveau, M. Kudruss, O. Stasse, C. Kirches, K. Mombaur, and P. Sou` eres,
  "A reactive walking pattern generator based on nonlinear model
  predictive control," in IEEE Robotics and Automation Letters (RAL), 2016.

*/

#include "portability/gettimeofday.hh"

#ifdef WIN32
# include <Windows.h>
#endif /* WIN32 */

#include <time.h>
#include <iostream>
#include <fstream>

#include <privatepgtypes.hh>
#include <ZMPRefTrajectoryGeneration/ZMPVelocityReferencedSQP.hh>

#include <Debug.hh>

//#define DEBUG

using namespace std;
using namespace PatternGeneratorJRL;

ZMPVelocityReferencedSQP::
ZMPVelocityReferencedSQP
(SimplePluginManager *SPM,
 string, PinocchioRobot *aPR ) :
  ZMPRefTrajectoryGeneration(SPM),OFTG_(NULL),
  dynamicFilter_(NULL),CurrentIndexUpperBound_(40)
{
  // Save the reference to HDR
  PR_ = aPR ;

  // PG management
  Running_ = false ;
  TimeBuffer_ = 0.04 ;
  SQP_T_ = 0.1 ;
  SQP_N_ = 16 ;
  SQP_nf_ = 2 ;
  m_SamplingPeriod = 0.005 ;
  outputPreviewDuration_ = SQP_T_ ;

  // Generator Management
  InterpolationPeriod_ = m_SamplingPeriod*7;
  previewSize_ = 9 ;
  previewDuration_ =  (previewSize_-1)*SQP_T_ ;
  NbSampleOutput_ = (int)round(outputPreviewDuration_/m_SamplingPeriod) + 1 ;
  NbSampleControl_ = (int)round(SQP_T_/m_SamplingPeriod) ;
  NbSampleInterpolation_ = (int)round(SQP_T_/InterpolationPeriod_) ;
  StepPeriod_ = 0.8 ;
  m_Tsingle = 0.7 ;
  m_Tdble = 0.1 ;
  FeetDistance_ = 0.2 ;
  CoMHeight_ = 0.814 ;
  UpperTimeLimitToUpdate_ = 0.0 ;

  // perturbation management
  PerturbationOccured_ = false ;
  RobotMass_ = PR_->mass() ;

  // interpolation management
  StepHeight_ = 0.05 ;
  CurrentIndex_ = 1 ;

  // Initialize  the 2D LIPM
  LIPM_.SetSimulationControlPeriod( SQP_T_ );
  LIPM_.SetRobotControlPeriod( m_SamplingPeriod );
  LIPM_.InitializeSystem();

  // Create and initialize online interpolation of feet trajectories:
  // ----------------------------------------------------------------
  OFTG_ = new OnLineFootTrajectoryGeneration(SPM,PR_->leftFoot());

  NMPCgenerator_ = new NMPCgenerator(SPM,PR_);

  dynamicFilter_ = new DynamicFilter(SPM,PR_);

  // Register method to handle
  const unsigned int NbMethods = 8;
  string aMethodName[NbMethods] =
    {
     ":previewcontroltime",
     ":numberstepsbeforestop",
     ":stoppg",
     ":setfeetconstraint",
     ":addoneobstacle",
     ":updateoneobstacle",
     ":deleteallobstacles",
     ":perturbationforce"
    };

  for(unsigned int i=0; i<NbMethods; i++)
    {
      if (!RegisterMethod(aMethodName[i]))
        {
          std::cerr << "Unable to register " << aMethodName << std::endl;
        }
    }

  // init of the buffer for the kajita's dynamic filter
  // size = numberOfIterationOfThePreviewControl * NumberOfSample
  // + Margin(=CurrentIndex)
  // Waring current index is higher on the robot than in the jrl Test suit
  unsigned int IndexMax = (int)round((previewDuration_)/InterpolationPeriod_ );
  ZMPTraj_deq_.resize(IndexMax);
  COMTraj_deq_.resize(IndexMax);
  LeftFootTraj_deq_.resize(IndexMax);
  RightFootTraj_deq_.resize(IndexMax);
  ZMPTraj_deq_ctrl_      .
    resize( previewSize_ * NbSampleControl_ +
            CurrentIndexUpperBound_);
  COMTraj_deq_ctrl_      .
    resize( previewSize_ * NbSampleControl_ +
            CurrentIndexUpperBound_);
  LeftFootTraj_deq_ctrl_ .
    resize( previewSize_ * NbSampleControl_ +
            CurrentIndexUpperBound_);
  RightFootTraj_deq_ctrl_.
    resize( previewSize_ * NbSampleControl_ +
            CurrentIndexUpperBound_);

  deltaCOMTraj_deq_.
    resize((int)round(outputPreviewDuration_/m_SamplingPeriod));

  JerkX_      .clear();
  JerkY_      .clear();
  FootStepX_  .clear();
  FootStepY_  .clear();
  FootStepYaw_.clear();
}


ZMPVelocityReferencedSQP::~ZMPVelocityReferencedSQP()
{
  if (NMPCgenerator_!=NULL)
    {
      delete NMPCgenerator_;
      NMPCgenerator_= NULL ;
    }
  if (OFTG_!=NULL)
    {
      delete OFTG_;
      OFTG_ = NULL ;
    }
  if (dynamicFilter_!=NULL)
    {
      delete dynamicFilter_;
      dynamicFilter_ = NULL ;
    }
}

void ZMPVelocityReferencedSQP::
setCoMPerturbationForce(istringstream &strm)
{
  double tmp ;
  PerturbationAcceleration_.resize(6);
  PerturbationAcceleration_.setZero();
  strm >> PerturbationAcceleration_(2);
  strm >> PerturbationAcceleration_(5);
  strm >> tmp ;
  PerturbationAcceleration_(2) = PerturbationAcceleration_(2)/RobotMass_;
  PerturbationAcceleration_(5) = PerturbationAcceleration_(5)/RobotMass_;
  PerturbationOccured_ = true;
}

void ZMPVelocityReferencedSQP::
setCoMPerturbationForce(double x, double y)
{

  PerturbationAcceleration_.resize(6);
  PerturbationAcceleration_.setZero();

  PerturbationAcceleration_(2) = x/RobotMass_;
  PerturbationAcceleration_(5) = y/RobotMass_;
  PerturbationOccured_ = true;
}

//--------------------------------------
//
//
//-----------new functions--------------
void ZMPVelocityReferencedSQP::
CallMethod(std::string & Method, std::istringstream &strm)
{
  if (Method==":previewcontroltime")
    {
      strm >> m_PreviewControlTime;
    }
  if (Method==":numberstepsbeforestop")
    {
      support_state_t & CurrentSupport = NMPCgenerator_->currentSupport();
      strm >> CurrentSupport.NbStepsLeft;
      NMPCgenerator_->setNbStepsLeft(CurrentSupport.NbStepsLeft);
    }
  if (Method==":stoppg")
    {
      EndingPhase_ = true;
    }
  if(Method==":setfeetconstraint")
    {
      NMPCgenerator_->RFI()->CallMethod(Method,strm);
    }
  if(Method==":addoneobstacle")
    {
      double x(0),y(0),r(0);
      strm >> x;
      strm >> y;
      strm >> r;
      NMPCgenerator_->addOneObstacle(x,y,r);
    }
  if(Method==":deleteallobstacles")
    {
      NMPCgenerator_->deleteAllObstacles();
    }
  if(Method==":updateoneobstacle")
    {
      unsigned id(0);
      double x(0),y(0),r(0);
      strm >> id;
      strm >> x;
      strm >> y;
      strm >> r;
      NMPCgenerator_->updateOneObstacle(id,x,y,r);
    }
  if(Method==":perturbationforce")
    {
      setCoMPerturbationForce(strm);
    }

  ZMPRefTrajectoryGeneration::CallMethod(Method,strm);

  if (Method==":singlesupporttime" || Method==":doublesupporttime")
    {
      StepPeriod_ = m_Tsingle + m_Tdble ;
    }

  if (Method==":doublesupporttime")
    {
      SQP_T_ = m_Tdble ;
    }
}

std::size_t ZMPVelocityReferencedSQP::
InitOnLine
(deque<ZMPPosition> & FinalZMPTraj_deq,
 deque<COMState> & FinalCoMPositions_deq,
 deque<FootAbsolutePosition> & FinalLeftFootTraj_deq,
 deque<FootAbsolutePosition> & FinalRightFootTraj_deq,
 FootAbsolutePosition & InitLeftFootAbsolutePosition,
 FootAbsolutePosition & InitRightFootAbsolutePosition,
 deque<RelativeFootPosition> &, // RelativeFootPositions,
 COMState & lStartingCOMState,
 Eigen::Vector3d & lStartingZMPPosition)
{

  // Generator Management
  previewSize_ = 8 ;
  InterpolationPeriod_ = m_SamplingPeriod*7;
  outputPreviewDuration_ = m_SamplingPeriod ;
  //outputPreviewDuration_ = SQP_T_ ;
  previewDuration_ =  (previewSize_-1)*SQP_T_ ;
  NbSampleOutput_ = (int)round(outputPreviewDuration_/m_SamplingPeriod) + 1 ;
  NbSampleControl_ = (int)round(SQP_T_/m_SamplingPeriod) ;
  NbSampleInterpolation_ = (int)round(SQP_T_/InterpolationPeriod_) ;

  UpperTimeLimitToUpdate_ = 0.0;
  FootAbsolutePosition CurrentLeftFootAbsPos, CurrentRightFootAbsPos;

  // Set the internal state of the ZMPRefTrajectory object.
  m_OnLineMode = true;
  EndingPhase_ = false;
  TimeToStopOnLineMode_ = -1.0;

  // INITIALIZE FEET POSITIONS:
  // --------------------------
  CurrentLeftFootAbsPos = InitLeftFootAbsolutePosition;
  CurrentRightFootAbsPos = InitRightFootAbsolutePosition;

  // FILL THE QUEUES:
  // ----------------
  int AddArraySize;
  {
    assert(m_SamplingPeriod > 0);
    double ldAddArraySize = TimeBuffer_/m_SamplingPeriod;
    AddArraySize = (int)ldAddArraySize;
  }

  FinalZMPTraj_deq.resize(AddArraySize);
  FinalCoMPositions_deq.resize(AddArraySize);
  FinalLeftFootTraj_deq.resize(AddArraySize);
  FinalRightFootTraj_deq.resize(AddArraySize);
  int CurrentZMPindex=0;
  m_CurrentTime = 0.0;
  for( unsigned int i=0; i<FinalZMPTraj_deq.size(); i++ )
    {
      // Smooth ramp
      FinalZMPTraj_deq[CurrentZMPindex].px = lStartingZMPPosition(0);
      FinalZMPTraj_deq[CurrentZMPindex].py = lStartingZMPPosition(1);
      FinalZMPTraj_deq[CurrentZMPindex].pz = lStartingZMPPosition(2);
      FinalZMPTraj_deq[CurrentZMPindex].theta = 0.0;
      FinalZMPTraj_deq[CurrentZMPindex].time = m_CurrentTime;
      FinalZMPTraj_deq[CurrentZMPindex].stepType = 0;

      // Set CoM positions.
      FinalCoMPositions_deq[CurrentZMPindex] = lStartingCOMState;
      // Set Left Foot positions.
      FinalLeftFootTraj_deq[CurrentZMPindex]  = CurrentLeftFootAbsPos;
      FinalRightFootTraj_deq[CurrentZMPindex] = CurrentRightFootAbsPos;
      FinalLeftFootTraj_deq[CurrentZMPindex].time =
        FinalRightFootTraj_deq[CurrentZMPindex].time = m_CurrentTime;
      FinalLeftFootTraj_deq[CurrentZMPindex].stepType =
        FinalRightFootTraj_deq[CurrentZMPindex].stepType = 10;

      m_CurrentTime += m_SamplingPeriod;
      CurrentZMPindex++;
    }

  // INITIAL SUPPORT STATE:
  // ----------------------
  support_state_t currentSupport ;
  currentSupport.Phase = DS;
  currentSupport.Foot = LEFT;
  currentSupport.TimeLimit = 1e+9;
  currentSupport.NbStepsLeft = 1;
  currentSupport.StateChanged = false;
  currentSupport.X = FinalLeftFootTraj_deq[0].x ;
  currentSupport.Y = FinalLeftFootTraj_deq[0].y ;
  currentSupport.Yaw=0.0;
  currentSupport.StartTime = 0.0;

  // INITIAL FOOT TRAJ GENERATOR:
  // ----------------------------
  OFTG_->InitializeInternalDataStructures();
  OFTG_->SetSingleSupportTime( m_Tsingle );
  OFTG_->SetDoubleSupportTime( m_Tdble );
  OFTG_->SetSamplingPeriod( m_SamplingPeriod );
  OFTG_->QPSamplingPeriod( SQP_T_ );
  OFTG_->NbSamplingsPreviewed( SQP_N_ );
  OFTG_->FeetDistance( FeetDistance_ );
  OFTG_->SetStepHeight( StepHeight_ );
  OFTG_->SetStepStairOn(0) ;

  // INITIAL SOLVER:
  // ---------------
  NMPCgenerator_->T(SQP_T_);
  NMPCgenerator_->N(SQP_N_);
  NMPCgenerator_->T_step(StepPeriod_);
  bool useLineSearch = false ;
  if(outputPreviewDuration_==m_SamplingPeriod &&
     m_Tsingle == 0.7 && m_Tdble == 0.1 )
    useLineSearch = true ;
  useLineSearch = false ; // WARNING remove to use the line search
  SQP_nf_ = (int)ceil(SQP_N_*SQP_T_/StepPeriod_-1e-6);
  NMPCgenerator_->initNMPCgenerator(useLineSearch,
                                    currentSupport,
                                    lStartingCOMState,
                                    VelRef_,SQP_N_,SQP_nf_,
                                    SQP_T_,StepPeriod_);



  // init of the buffer for the kajita's dynamic filter
  // size = numberOfIterationOfThePreviewControl * NumberOfSample
  // + Margin(=CurrentIndex)
  // Waring current index is higher on the robot than in the jrl Test suit
  unsigned int IndexMax = (int)round((previewDuration_)/InterpolationPeriod_ );
  ZMPTraj_deq_.resize(IndexMax);
  COMTraj_deq_.resize(IndexMax);
  LeftFootTraj_deq_.resize(IndexMax);
  RightFootTraj_deq_.resize(IndexMax);
  ZMPTraj_deq_ctrl_      .resize( previewSize_ * NbSampleControl_
                                  + CurrentIndexUpperBound_);
  COMTraj_deq_ctrl_      .resize( previewSize_ * NbSampleControl_
                                  + CurrentIndexUpperBound_);
  LeftFootTraj_deq_ctrl_ .resize( previewSize_ * NbSampleControl_
                                  + CurrentIndexUpperBound_);
  RightFootTraj_deq_ctrl_.resize( previewSize_ * NbSampleControl_
                                  + CurrentIndexUpperBound_);

  deltaCOMTraj_deq_.
    resize((int)round(outputPreviewDuration_/m_SamplingPeriod));

  JerkX_      .clear();
  JerkY_      .clear();
  FootStepX_  .clear();
  FootStepY_  .clear();
  FootStepYaw_.clear();

  // INITIALIZE CENTER OF MASS:
  // --------------------------
  com_t CoM;
  CoM.x[0] = lStartingCOMState.x[0];
  CoM.x[1] = lStartingCOMState.x[1];
  CoM.x[2] = lStartingCOMState.x[2];
  CoM.y[0] = lStartingCOMState.y[0];
  CoM.y[1] = lStartingCOMState.y[1];
  CoM.y[2] = lStartingCOMState.y[2];
  CoM.z[0] = lStartingCOMState.z[0];
  CoM.z[1] = lStartingCOMState.z[1];
  CoM.z[2] = lStartingCOMState.z[2];
  initCOM_ = lStartingCOMState;
  LIPM_.SetComHeight(lStartingCOMState.z[0]);
  LIPM_.SetSimulationControlPeriod( SQP_T_ );
  LIPM_.SetRobotControlPeriod( m_SamplingPeriod );
  LIPM_.InitializeSystem();
  LIPM_(CoM);

  dynamicFilter_->getComAndFootRealization()->ShiftFoot(true);
  dynamicFilter_->init(m_SamplingPeriod,
                       InterpolationPeriod_,
                       outputPreviewDuration_,
                       previewDuration_,
                       previewDuration_-outputPreviewDuration_,
                       lStartingCOMState);

  m_CurrentConfiguration_ = PR_->currentRPYConfiguration() ;
  m_CurrentVelocity_      = PR_->currentRPYVelocity() ;
  m_CurrentAcceleration_  = PR_->currentRPYAcceleration() ;

  initZMP_       = FinalZMPTraj_deq      [0] ;
  initCOM_       = FinalCoMPositions_deq [0] ;
  itCOM_         = FinalCoMPositions_deq [0] ;
  initLeftFoot_  = FinalLeftFootTraj_deq [0] ;
  initRightFoot_ = FinalRightFootTraj_deq[0] ;
  CurrentIndex_ = 1 ;

  JerkX_      .resize((long unsigned int)NMPCgenerator_->N()) ;
  JerkY_      .resize((long unsigned int)NMPCgenerator_->N()) ;
  FootStepX_  .resize((long unsigned int)(NMPCgenerator_->nf()+1));
  FootStepY_  .resize((long unsigned int)(NMPCgenerator_->nf()+1));
  FootStepYaw_.resize((long unsigned int)(NMPCgenerator_->nf()+1));
  return 0;
}

void ZMPVelocityReferencedSQP::
OnLine(double time,
       deque<ZMPPosition> & FinalZMPTraj_deq,
       deque<COMState> & FinalCOMTraj_deq,
       deque<FootAbsolutePosition> & FinalLeftFootTraj_deq,
       deque<FootAbsolutePosition> & FinalRightFootTraj_deq)
{
  // If on-line mode not activated we go out.
  if (!m_OnLineMode)
    {
      return;
    }

  // Test if the end of the online mode has been reached.
  if ((EndingPhase_) && (time>=TimeToStopOnLineMode_))
    {
      m_OnLineMode = false;
    }

  // UPDATE WALKING TRAJECTORIES:
  // ----------------------------
  if(time + 0.00001 > UpperTimeLimitToUpdate_)
    {
      // UPDATE INTERNAL DATA:
      // ---------------------
      if(PerturbationOccured_ &&
         NMPCgenerator_->currentSupport().NbStepsLeft>1 &&
         NMPCgenerator_->SupportStates_deq().back().StepNumber >0)
        {
          initCOM_.x[2]+=PerturbationAcceleration_(2);
          initCOM_.y[2]+=PerturbationAcceleration_(5);
          itCOM_.x[2]+=PerturbationAcceleration_(2);
          itCOM_.y[2]+=PerturbationAcceleration_(5);
          PerturbationOccured_=false;
        }
      VelRef_=NewVelRef_;

      //    struct timeval begin ;
      //    gettimeofday(&begin,0);

      NMPCgenerator_->updateInitialCondition(
                                             time,
                                             initLeftFoot_,
                                             initRightFoot_,
                                             itCOM_,
                                             //initCOM_,
                                             VelRef_);

      // SOLVE PROBLEM:
      // --------------
      NMPCgenerator_->solve();

      //    static int warning=0;
      //    struct timeval end ;
      //    gettimeofday(&end,0);
      //    double ltime = end.tv_sec-begin.tv_sec
      //        + 0.000001 * (end.tv_usec - begin.tv_usec);

      //    bool endline = false ;
      //    if(ltime >= 0.0005)
      //    {
      //      cout << ltime * 1000 << " "
      //           << NMPCgenerator_->cput()*1000 << " "
      //           << ltime * 1000 - NMPCgenerator_->cput()*1000 ;
      //      endline = true;
      //    }
      //    if((ltime * 1000 - NMPCgenerator_->cput()*1000)>= 0.5)
      //    {
      //      ++warning;
      //      cout << " : warning on cpu time ; " << warning ;
      //      endline = true;
      //    }
      //    if(endline)
      //    {
      //      cout << endl;
      //    }

      // INITIALIZE INTERPOLATION:
      // ------------------------
      for (unsigned int i = 0  ; i < 1 + CurrentIndex_ ; ++i )
        {
          ZMPTraj_deq_ctrl_[i] = initZMP_ ;
          COMTraj_deq_ctrl_[i] = initCOM_ ;
          LeftFootTraj_deq_ctrl_[i] = initLeftFoot_ ;
          RightFootTraj_deq_ctrl_[i] = initRightFoot_ ;
        }

      // INTERPOLATION
      // ------------------------
      // Compute the full trajectory in the preview window
      FullTrajectoryInterpolation(time);

      // Take only the data that are actually used by the robot
      FinalZMPTraj_deq.resize(NbSampleOutput_);
      FinalLeftFootTraj_deq .resize(NbSampleOutput_);
      FinalCOMTraj_deq.resize(NbSampleOutput_);
      FinalRightFootTraj_deq.resize(NbSampleOutput_);
      for(unsigned i=0 ; i < NbSampleOutput_ ; ++i)
        {
          FinalZMPTraj_deq      [i] = ZMPTraj_deq_ctrl_      [i] ;
          FinalCOMTraj_deq      [i] = COMTraj_deq_ctrl_      [i] ;
          FinalLeftFootTraj_deq [i] = LeftFootTraj_deq_ctrl_ [i] ;
          FinalRightFootTraj_deq[i] = RightFootTraj_deq_ctrl_[i] ;
        }
      dynamicFilter_->OnLinefilter(COMTraj_deq_,ZMPTraj_deq_ctrl_,
                                   LeftFootTraj_deq_,
                                   RightFootTraj_deq_,
                                   deltaCOMTraj_deq_);
#ifdef DEBUG
      dynamicFilter_->Debug(COMTraj_deq_ctrl_,
                            LeftFootTraj_deq_ctrl_,
                            RightFootTraj_deq_ctrl_,
                            COMTraj_deq_,ZMPTraj_deq_ctrl_,
                            LeftFootTraj_deq_,
                            RightFootTraj_deq_,
                            deltaCOMTraj_deq_);
#endif
      // Correct the CoM.
      for (unsigned int i = 0 ; i < deltaCOMTraj_deq_.size() ; ++i)
        {
          for(int j=0; j<3; j++)
            {
              FinalCOMTraj_deq[i].x[j] += deltaCOMTraj_deq_[i].x[j] ;
              FinalCOMTraj_deq[i].y[j] += deltaCOMTraj_deq_[i].y[j] ;
              //        FinalZMPTraj_deq[i].px = FinalCOMTraj_deq[i].x[0] -
              //            CoMHeight_ / 9.81 * FinalCOMTraj_deq[i].x[2];
              //        FinalZMPTraj_deq[i].py = FinalCOMTraj_deq[i].y[0] -
              //            CoMHeight_ / 9.81 * FinalCOMTraj_deq[i].y[2];
              //        FinalZMPTraj_deq[i].pz = 0.0 ;
            }
        }

      // Specify that we are in the ending phase.
      if (time <= m_SamplingPeriod )
        {
          if (EndingPhase_ == false)
            {
              TimeToStopOnLineMode_ = UpperTimeLimitToUpdate_
                + outputPreviewDuration_ * SQP_N_
                + m_SamplingPeriod;
            }
          UpperTimeLimitToUpdate_ = UpperTimeLimitToUpdate_
            + outputPreviewDuration_ + m_SamplingPeriod ;
        }
      else
        {
          if (EndingPhase_ == false)
            {
              TimeToStopOnLineMode_ = UpperTimeLimitToUpdate_
                + outputPreviewDuration_ * SQP_N_;
            }
          UpperTimeLimitToUpdate_ = UpperTimeLimitToUpdate_
            + outputPreviewDuration_;
        }
    }
  //-----------------------------------
  //
  //
  //----------"Real-time" loop---------
}

void ZMPVelocityReferencedSQP::FullTrajectoryInterpolation(double time)
{
  if(LeftFootTraj_deq_ctrl_.size() <
     CurrentIndex_ + previewSize_ * NbSampleControl_)
    {
      COMTraj_deq_ctrl_.resize(CurrentIndex_ + previewSize_ * NbSampleControl_
                               + CurrentIndexUpperBound_,itCOM_);
      ZMPTraj_deq_ctrl_.resize(CurrentIndex_ + previewSize_ * NbSampleControl_
                               + CurrentIndexUpperBound_,initZMP_);
      LeftFootTraj_deq_ctrl_.resize(CurrentIndex_ + previewSize_ *
                                    NbSampleControl_
                                    + CurrentIndexUpperBound_,initLeftFoot_);
      RightFootTraj_deq_ctrl_.resize(CurrentIndex_ + previewSize_ *
                                     NbSampleControl_
                                     + CurrentIndexUpperBound_,initRightFoot_);
    }

  NMPCgenerator_->getSolution(JerkX_, JerkY_, FootStepX_,
                              FootStepY_, FootStepYaw_);

  const std::deque<support_state_t> & SupportStates_deq =
    NMPCgenerator_->SupportStates_deq();
  LIPM_.setState(itCOM_);

  CoMZMPInterpolation(JerkX_,JerkY_,&LIPM_,
                      NbSampleControl_,0,
                      CurrentIndex_,SupportStates_deq);
  itCOM_ = COMTraj_deq_ctrl_[NbSampleOutput_-1];

  support_state_t currentSupport = SupportStates_deq[0] ;
  currentSupport.StepNumber=0;
  OFTG_->interpolate_feet_positions
    (time, CurrentIndex_, currentSupport,
     FootStepX_, FootStepY_, FootStepYaw_,
     LeftFootTraj_deq_ctrl_, RightFootTraj_deq_ctrl_);

  double currentTime = time + NMPCgenerator_->Tfirst();
  double currentIndex = CurrentIndex_ +
    (int)round(NMPCgenerator_->Tfirst()/m_SamplingPeriod);
  for ( unsigned int i = 1 ; i<previewSize_ ; i++ )
    {
      LIPM_.setState(COMTraj_deq_ctrl_[(long unsigned int)
                                       (currentIndex-1)]);
      CoMZMPInterpolation(JerkX_,JerkY_,&LIPM_,
                          NbSampleControl_,
                          i,(unsigned int)(currentIndex),
                          SupportStates_deq);
      OFTG_->interpolate_feet_positions(currentTime,
                                        (unsigned int)(currentIndex),
                                        SupportStates_deq[i],
                                        FootStepX_, FootStepY_,
                                        FootStepYaw_,
                                        LeftFootTraj_deq_ctrl_,
                                        RightFootTraj_deq_ctrl_);
      currentTime  += SQP_T_;
      currentIndex += (int)round(SQP_T_/m_SamplingPeriod) ;
    }

  for (unsigned i=CurrentIndex_ ;
       i<previewSize_*NbSampleControl_ ; i++ )
    {
      COMTraj_deq_ctrl_[i].roll[0]  = 0.0 ;
      COMTraj_deq_ctrl_[i].pitch[0] = 0.0 ;
      COMTraj_deq_ctrl_[i].yaw[0]   =
        0.5*(LeftFootTraj_deq_ctrl_ [i].theta
             +RightFootTraj_deq_ctrl_[i].theta)*M_PI/180 ;

      COMTraj_deq_ctrl_[i].roll[1]  = 0.0 ;
      COMTraj_deq_ctrl_[i].pitch[1] = 0.0 ;
      COMTraj_deq_ctrl_[i].yaw[1]   = 0.5*
        (LeftFootTraj_deq_ctrl_ [i].dtheta
         +RightFootTraj_deq_ctrl_[i].dtheta)*M_PI/180 ;

      COMTraj_deq_ctrl_[i].roll[2]  = 0.0 ;
      COMTraj_deq_ctrl_[i].pitch[2] = 0.0 ;
      COMTraj_deq_ctrl_[i].yaw[2]   = 0.5*
        (LeftFootTraj_deq_ctrl_ [i].ddtheta
         +RightFootTraj_deq_ctrl_[i].ddtheta)*M_PI/180 ;
    }

  COMTraj_deq_ctrl_      .push_back(COMTraj_deq_ctrl_      .back());
  ZMPTraj_deq_ctrl_      .push_back(ZMPTraj_deq_ctrl_      .back());
  LeftFootTraj_deq_ctrl_ .push_back(LeftFootTraj_deq_ctrl_ .back());
  RightFootTraj_deq_ctrl_.push_back(RightFootTraj_deq_ctrl_.back());
  COMTraj_deq_ctrl_      .pop_front();
  ZMPTraj_deq_ctrl_      .pop_front();
  LeftFootTraj_deq_ctrl_ .pop_front();
  RightFootTraj_deq_ctrl_.pop_front();

  unsigned int IndexMax = (int)round((previewDuration_)/
                                     InterpolationPeriod_ );
  int inc =  (int)round(InterpolationPeriod_ / m_SamplingPeriod) ;

  for (unsigned int i = 0, j = 0 ; j < IndexMax ; i = i + inc, ++j )
    {
      ZMPTraj_deq_[j] = ZMPTraj_deq_ctrl_[i] ;
      COMTraj_deq_[j] = COMTraj_deq_ctrl_[i] ;
      COMTraj_deq_[j].roll[0]  = 180/M_PI* COMTraj_deq_ctrl_[i].roll[0];
      COMTraj_deq_[j].pitch[0] = 180/M_PI* COMTraj_deq_ctrl_[i].pitch[0];
      COMTraj_deq_[j].yaw[0]   = 180/M_PI* COMTraj_deq_ctrl_[i].yaw[0] ;
      LeftFootTraj_deq_[j] = LeftFootTraj_deq_ctrl_[i] ;
      RightFootTraj_deq_[j] = RightFootTraj_deq_ctrl_[i] ;
    }
  return ;
}

void ZMPVelocityReferencedSQP::
CoMZMPInterpolation
(std::vector<double> & JerkX,           // INPUT
 std::vector<double> & JerkY,           // INPUT
 LinearizedInvertedPendulum2D * LIPM, // INPUT/OUTPUT
 const unsigned,        // INPUT
 const int IterationNumber,           // INPUT
 const unsigned int currentIndex,     // INPUT
 const std::deque<support_state_t> & SupportStates_deq )// INPUT
{
  if(SupportStates_deq[0].Phase==DS && SupportStates_deq[0].NbStepsLeft == 0)
    {
      unsigned int i = currentIndex ;
      double jx = (RightFootTraj_deq_ctrl_[i-1].x +
                   LeftFootTraj_deq_ctrl_[i-1].x)/2
        - COMTraj_deq_ctrl_[i-1].x[0];
      double jy = (RightFootTraj_deq_ctrl_[i-1].y +
                   LeftFootTraj_deq_ctrl_[i-1].y)/2
        - COMTraj_deq_ctrl_[i-1].y[0];

      if(fabs(jx) < 1e-3 && fabs(jy) < 1e-3 && IterationNumber==0 )
        {
          Running_ = false;
        }

      const double tf = 0.75;
      jx = 6/(tf*tf*tf)*(jx - tf*COMTraj_deq_ctrl_[i-1].x[1]
                         - (tf*tf/2)*COMTraj_deq_ctrl_[i-1].x[2]);
      jy = 6/(tf*tf*tf)*(jy - tf*COMTraj_deq_ctrl_[i-1].y[1]
                         - (tf*tf/2)*COMTraj_deq_ctrl_[i-1].y[2]);
      LIPM->Interpolation( COMTraj_deq_ctrl_, ZMPTraj_deq_ctrl_,
                           currentIndex, jx, jy);
    }
  else
    {
      Running_ = true;
      LIPM->Interpolation( COMTraj_deq_ctrl_, ZMPTraj_deq_ctrl_, currentIndex,
                           JerkX[IterationNumber], JerkY[IterationNumber] );
    }
  return ;
}

// TODO: New parent class needed
void ZMPVelocityReferencedSQP::
GetZMPDiscretization
(deque<ZMPPosition> &,
 deque<COMState> &,
 deque<RelativeFootPosition> &,
 deque<FootAbsolutePosition> &,
 deque<FootAbsolutePosition> &,
 double,
 COMState &,
 Eigen::Vector3d &,
 FootAbsolutePosition &,
 FootAbsolutePosition & )
{
  cout << "To be removed" << endl;
}


void ZMPVelocityReferencedSQP::
OnLineAddFoot
(RelativeFootPosition &,
 deque<ZMPPosition> &,
 deque<COMState> &,
 deque<FootAbsolutePosition> &,
 deque<FootAbsolutePosition> &,
 bool)
{
  cout << "To be removed" << endl;
}

int ZMPVelocityReferencedSQP::
OnLineFootChange
(double,
 FootAbsolutePosition &,
 deque<ZMPPosition> &,
 deque<COMState> &,
 deque<FootAbsolutePosition> &,
 deque<FootAbsolutePosition> &,
 StepStackHandler  *)
{
  cout << "To be removed" << endl;
  return -1;
}

void ZMPVelocityReferencedSQP::
EndPhaseOfTheWalking
(deque<ZMPPosition> &,
 deque<COMState> &,
 deque<FootAbsolutePosition> &,
 deque<FootAbsolutePosition> &)
{
  cout << "To be removed" << endl;
}

int ZMPVelocityReferencedSQP::ReturnOptimalTimeToRegenerateAStep()
{
  int r = (int)(m_PreviewControlTime/m_SamplingPeriod);
  return 2*r;
}
