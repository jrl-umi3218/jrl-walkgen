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
/* \file This file tests A. Herdt's walking algorithm for
 * automatic foot placement giving an instantaneous CoM velocity reference.
 */
#include "Debug.hh"
#include "CommonTools.hh"
#include "TestObject.hh"
#include <jrl/walkgen/pgtypes.hh>
#include <hrp2-dynamics/hrp2OptHumanoidDynamicRobot.h>
#include <MotionGeneration/ComAndFootRealizationByGeometry.hh>



using namespace::PatternGeneratorJRL;
using namespace::PatternGeneratorJRL::TestSuite;
using namespace std;

class TestMulticontactInverseGeometry: public TestObject
{

private:
  ComAndFootRealizationByGeometry * CFRG_;
  SimplePluginManager * SPM_ ;
  bool once_ ;

  deque<MAL_VECTOR_TYPE(double)> comPos_ ;
  deque<MAL_VECTOR_TYPE(double)> comSpeed_ ;
  deque<MAL_VECTOR_TYPE(double)> comAcc_ ;
  deque<MAL_VECTOR_TYPE(double)> lfoot_ ;
  deque<MAL_VECTOR_TYPE(double)> rfoot_ ;
  deque<MAL_VECTOR_TYPE(double)> rhand_ ;


  deque<MAL_VECTOR_TYPE(double)> q_ ;
  deque<MAL_VECTOR_TYPE(double)> dq_ ;
  deque<MAL_VECTOR_TYPE(double)> ddq_ ;

  vector<MAL_VECTOR_TYPE(double)> lfs_ ; // support foort deque
  vector<MAL_VECTOR_TYPE(double)> rfs_ ; // support foort deque
  vector<MAL_VECTOR_TYPE(double)> rhs_ ; // support foort deque
  vector<double> timing_ ; // support foort deque
  deque<double> data_time_ ;

  BSplinesFoot *LFX_;
  BSplinesFoot *LFY_;
  BSplinesFoot *LFZ_;

  BSplinesFoot *RFX_;
  BSplinesFoot *RFY_;
  BSplinesFoot *RFZ_;

  BSplinesFoot *RHX_;
  BSplinesFoot *RHY_;
  BSplinesFoot *RHZ_;
  BSplinesFoot *RHOMEGA_;

  double movingPeriod_   ;
  double supportPeriod_  ;
  double initialTime_    ;
  double finalTime_      ;
  double samplingPeriod_ ;

public:
  TestMulticontactInverseGeometry(int argc, char *argv[], string &aString):
      TestObject(argc,argv,aString)
  {
    SPM_ = NULL ;
    once_ = true ;
    movingPeriod_  = 1.4 ;
    supportPeriod_ = 0.2 ;
    initialTime_   = 5.0 ;
    finalTime_     = 5.0 ;
    samplingPeriod_ = 0.005 ;
  }

  ~TestMulticontactInverseGeometry()
  {
    if ( SPM_ != 0 )
    {
      delete SPM_ ;
      SPM_ = 0 ;
    }

    m_DebugHDR = 0;
  }

  typedef void (TestMulticontactInverseGeometry::* localeventHandler_t)(PatternGeneratorInterface &);

  struct localEvent
  {
    unsigned time;
    localeventHandler_t Handler ;
  };

  bool doTest(ostream &os)
  {
    for (unsigned int step = 0 ; step < timing_.size() ; ++step )
      {
        interpolate_support(step);
      }

    readData();
    com_sampling_correction();

    q_  .resize(comPos_.size()) ;
    dq_ .resize(comPos_.size()) ;
    ddq_.resize(comPos_.size()) ;

    matrix4d identity, rightHandPose, rs_w ;
    MAL_VECTOR_DIM(rarm_q, double, 6) ;
    CjrlHumanoidDynamicRobot * HDR = CFRG_->getHumanoidDynamicRobot();
    CjrlJoint* right_shoulder = NULL ;


    MAL_S4x4_MATRIX_SET_IDENTITY(identity);
    MAL_S4x4_MATRIX_SET_IDENTITY(rightHandPose);
    MAL_S4x4_MATRIX_SET_IDENTITY(rs_w);

    MAL_S4x4_MATRIX_ACCESS_I_J(rs_w,0,3) = -0.04   ;
    MAL_S4x4_MATRIX_ACCESS_I_J(rs_w,1,3) = 0.25  ;
    MAL_S4x4_MATRIX_ACCESS_I_J(rs_w,2,3) = -0.5317 ;

    std::vector<CjrlJoint*> actuatedJoints = HDR->getActuatedJoints() ;
    for (unsigned int i = 0 ; i < actuatedJoints.size() ; ++i )
      {
        if ( actuatedJoints[i]->getName() == "RARM_JOINT0" )
          right_shoulder = actuatedJoints[i];
      }


    cout << comPos_.size() << endl ;
    cout << lfoot_.size() << endl ;

    assert(comPos_.size() == lfoot_.size() );
    ofstream aof;
    string aFileName;
    static int iteration = 0 ;

    aFileName = "";
    aFileName+=m_TestName;
    aFileName+="Full.bin";
    if ( iteration == 0 ){
      aof.open(aFileName.c_str(), ios::out | ios::binary);
      aof.close();
    }
    aof.open(aFileName.c_str(), ios::app | ios::binary);
    aof.precision(8);
    aof.setf(ios::scientific, ios::floatfield);

    for(unsigned int i = 0 ; i < lfoot_.size() ; i++){
      aof << filterprecision( data_time_[i] ) << " "  ; // 1
      aof << filterprecision( lfoot_[i](0)  ) << " "  ; // 2   x
      aof << filterprecision( lfoot_[i](1)  ) << " "  ; // 3   y
      aof << filterprecision( lfoot_[i](2)  ) << " "  ; // 4   z

      aof << filterprecision( rfoot_[i](0)  ) << " "  ; // 5   x
      aof << filterprecision( rfoot_[i](1)  ) << " "  ; // 6   y
      aof << filterprecision( rfoot_[i](2)  ) << " "  ; // 7   z

      aof << filterprecision( rhand_[i](0)  ) << " "  ; // 8  x
      aof << filterprecision( rhand_[i](1)  ) << " "  ; // 9  y
      aof << filterprecision( rhand_[i](2)  ) << " "  ; // 10  z
      aof << filterprecision( rhand_[i](3)  ) << " "  ; // 11  pitch angle

      if (i*0.005 < timing_[0])
      {
        aof << filterprecision(  0.9639598  ) << " "  ; // 12  R(0.0) rotation matrix
        aof << filterprecision( -0.04494346 ) << " "  ; // 13  R(0.1) of the right hand
        aof << filterprecision( -0.26222429 ) << " "  ; // 14  R(0.2) half sitting compatible
        aof << filterprecision(  0.08682409 ) << " "  ; // 15  R(1.0) position
        aof << filterprecision(  0.98480775 ) << " "  ; // 16  R(1.1)
        aof << filterprecision(  0.15038373 ) << " "  ; // 17  R(1.2)
        aof << filterprecision(  0.25148175 ) << " "  ; // 18  R(2.0)
        aof << filterprecision( -0.16773126 ) << " "  ; // 19  R(2.1)
        aof << filterprecision(  0.95321726 ) << " "  ; // 20  R(2.2)
      }else{
        aof << filterprecision( cos(rhand_[i](3))  ) << " "  ; // 12  R(0.0) rotation matrix
        aof << filterprecision( sin(rhand_[i](3))  ) << " "  ; // 13  R(0.1) of the right hand
        aof << filterprecision( 0.0                ) << " "  ; // 14  R(0.2)
        aof << filterprecision( -sin(rhand_[i](3)) ) << " "  ; // 15  R(1.0)
        aof << filterprecision( cos(rhand_[i](3))  ) << " "  ; // 16  R(1.1)
        aof << filterprecision( 0.0                ) << " "  ; // 17  R(1.2)
        aof << filterprecision( 0.0                ) << " "  ; // 18  R(2.0)
        aof << filterprecision( 0.0                ) << " "  ; // 19  R(2.1)
        aof << filterprecision( 1.0                ) << " "  ; // 20  R(2.2)
      }

      aof << filterprecision( comPos_[i](0) ) << " "  ; // 21  x
      aof << filterprecision( comPos_[i](1) ) << " "  ; // 22  y
      aof << filterprecision( comPos_[i](2) ) << " "  ; // 23  z
      aof << filterprecision( comPos_[i](3) ) << " "  ; // 24  roll
      aof << filterprecision( comPos_[i](4) ) << " "  ; // 25  pitch
      aof << filterprecision( comPos_[i](5) ) << " "  ; // 26  yaw
      aof << endl ;
    }

    aof.close();

    ++iteration ;

    fillInDebugFiles();
    return true ;
  }

  void init()
  {
    // Instanciate and initialize.
    string RobotFileName = m_VRMLPath + m_VRMLFileName;

    bool fileExist = false;
    {
      std::ifstream file (RobotFileName.c_str ());
      fileExist = !file.fail ();
    }
    if (!fileExist)
      throw std::string ("failed to open robot model");

    // Creating the humanoid robot.
    SpecializedRobotConstructor(m_HDR);
    if(m_HDR==0)
    {
      if (m_HDR!=0) delete m_HDR;
      dynamicsJRLJapan::ObjectFactory aRobotDynamicsObjectConstructor;
      m_HDR = aRobotDynamicsObjectConstructor.createHumanoidDynamicRobot();
    }

    // Parsing the file.
    dynamicsJRLJapan::parseOpenHRPVRMLFile(*m_HDR,RobotFileName,
                                           m_LinkJointRank,
                                           m_SpecificitiesFileName);
    // Create Pattern Generator Interface
    m_PGI = patternGeneratorInterfaceFactory(m_HDR);

    vector<CjrlJoint *> actuatedJoints = m_HDR->getActuatedJoints();
    unsigned int lNbActuatedJoints = actuatedJoints.size();
    double * dInitPos = new double[lNbActuatedJoints];
    ifstream aif;
    aif.open(m_InitConfig.c_str(),ifstream::in);
    if (aif.is_open())
    {
      for(unsigned int i=0;i<lNbActuatedJoints;i++)
        aif >> dInitPos[i];
    }
    aif.close();

    bool DebugConfiguration = true;
    ofstream aofq;
    if (DebugConfiguration)
    {
      aofq.open("TestConfiguration.dat",ofstream::out);
      if (aofq.is_open())
        {
          for(unsigned int k=0;k<30;k++)
      {
        aofq << dInitPos[k] << " ";
      }
          aofq << endl;
        }

    }

    MAL_VECTOR_RESIZE(InitialPosition,lNbActuatedJoints);
    // This is a vector corresponding to the DOFs actuated of the robot.
    bool conversiontoradneeded=true;
    if (conversiontoradneeded)
      for(unsigned int i=0;i<MAL_VECTOR_SIZE(InitialPosition);i++)
        InitialPosition(i) = dInitPos[i]*M_PI/180.0;
    else
      for(unsigned int i=0;i<MAL_VECTOR_SIZE(InitialPosition);i++)
        InitialPosition(i) = dInitPos[i];

    // This is a vector corresponding to ALL the DOFS of the robot:
    // free flyer + actuated DOFS.
    unsigned int lNbDofs = 36 ;
    MAL_VECTOR_DIM(CurrentConfiguration,double,lNbDofs);
    MAL_VECTOR_DIM(CurrentVelocity,double,lNbDofs);
    MAL_VECTOR_DIM(CurrentAcceleration,double,lNbDofs);
    MAL_VECTOR_DIM(PreviousConfiguration,double,lNbDofs) ;
    MAL_VECTOR_DIM(PreviousVelocity,double,lNbDofs);
    MAL_VECTOR_DIM(PreviousAcceleration,double,lNbDofs);
    for(int i=0;i<6;i++)
    {
      PreviousConfiguration[i] =
        PreviousVelocity[i] =
        PreviousAcceleration[i] = 0.0;
    }

    for(unsigned int i=6;i<lNbDofs;i++)
    {
      PreviousConfiguration[i] = InitialPosition[i-6];
        PreviousVelocity[i] =
        PreviousAcceleration[i] = 0.0;
    }

    delete [] dInitPos;

    MAL_VECTOR_RESIZE(m_CurrentConfiguration, m_HDR->numberDof());
    MAL_VECTOR_RESIZE(m_CurrentVelocity, m_HDR->numberDof());
    MAL_VECTOR_RESIZE(m_CurrentAcceleration, m_HDR->numberDof());

    MAL_VECTOR_RESIZE(m_PreviousConfiguration, m_HDR->numberDof());
    MAL_VECTOR_RESIZE(m_PreviousVelocity, m_HDR->numberDof());
    MAL_VECTOR_RESIZE(m_PreviousAcceleration, m_HDR->numberDof());

    SPM_ = new SimplePluginManager();

    initIK();
    init_support();
    readData();
    init_interpolation();
  }

protected:


  int interpolate_support(int it)
  {
    if(it == 0)
      {
        for(int i = 0 ; i < (int)round(timing_[0]/samplingPeriod_) ; ++i )
          {
            lfoot_.push_back(lfs_[0]) ;
            rfoot_.push_back(rfs_[0]) ;
            rhand_.push_back(rhs_[0]) ;
          }
        return 1 ;
      }

    double liftCoef = 0.9;
    double gripperCoef = 1.0;
    double WayPoint_z = 0.05 ;
    double epsilon = 0.001 ;
    vector<double> MP ;
    vector<double> ToMP ;
    double iniPos ;
    double finPos ;
    double TimeInterval ;

    // LEFT FOOT
    MP.clear(); ToMP.clear() ;
    TimeInterval = liftCoef*timing_[it] ; iniPos = lfs_[it-1](0) ; finPos = lfs_[it](0) ;
    LFX_->SetParameters(TimeInterval,iniPos,finPos,ToMP,MP);

    MP.clear(); ToMP.clear() ;
    TimeInterval = liftCoef*timing_[it] ; iniPos = lfs_[it-1](1) ; finPos = lfs_[it](1) ;
    LFY_->SetParameters(TimeInterval,iniPos,finPos,ToMP,MP);

    MP.clear(); ToMP.clear() ;
    TimeInterval = timing_[it] ; iniPos = lfs_[it-1](2) ; finPos = lfs_[it](2) ;
    if (finPos - iniPos > epsilon )
      {
        ToMP.push_back(0.4*TimeInterval);
        MP.push_back(finPos+WayPoint_z);
      }
    else if (finPos - iniPos <= epsilon && finPos - iniPos >= -epsilon )
      {}
    LFZ_->SetParameters(TimeInterval,iniPos,finPos,ToMP,MP);

    // RIGHT FOOT
    MP.clear(); ToMP.clear() ;
    TimeInterval = liftCoef*timing_[it] ; iniPos = rfs_[it-1](0) ; finPos = rfs_[it](0) ;
    RFX_->SetParameters(TimeInterval,iniPos,finPos,ToMP,MP);

    MP.clear(); ToMP.clear() ;
    TimeInterval = liftCoef*timing_[it] ; iniPos = rfs_[it-1](1) ; finPos = rfs_[it](1) ;
    RFY_->SetParameters(TimeInterval,iniPos,finPos,ToMP,MP);

    MP.clear(); ToMP.clear() ;
    TimeInterval = timing_[it] ; iniPos = rfs_[it-1](2) ; finPos = rfs_[it](2) ;
    if (finPos - iniPos > epsilon )
      {
        ToMP.push_back(0.4*TimeInterval);
        MP.push_back(finPos+WayPoint_z);
      }
    else if (finPos - iniPos <= epsilon && finPos - iniPos >= -epsilon )
      {}
    RFZ_->SetParameters(TimeInterval,iniPos,finPos,ToMP,MP);


    // RIGHT HAND
    MP.clear(); ToMP.clear() ;
    TimeInterval = gripperCoef*liftCoef*timing_[it] ; iniPos = rhs_[it-1](0) ; finPos = rhs_[it](0) ;
    RHX_->SetParameters(TimeInterval,iniPos,finPos,ToMP,MP);

    MP.clear(); ToMP.clear() ;
    TimeInterval = gripperCoef*liftCoef*timing_[it] ; iniPos = rhs_[it-1](1) ; finPos = rhs_[it](1) ;
    RHY_->SetParameters(TimeInterval,iniPos,finPos,ToMP,MP);

    MP.clear(); ToMP.clear() ;
    TimeInterval = gripperCoef*timing_[it] ; iniPos = rhs_[it-1](2) ; finPos = rhs_[it](2) ;
    if (finPos - iniPos > epsilon )
      {
        ToMP.push_back(0.4*TimeInterval);
        MP.push_back(finPos+WayPoint_z);
      }
    else if (finPos - iniPos <= epsilon && finPos - iniPos >= -epsilon )
      {}
    RHZ_->SetParameters(TimeInterval,iniPos,finPos,ToMP,MP);

    MP.clear(); ToMP.clear() ;
    TimeInterval = gripperCoef*liftCoef*timing_[it] ; iniPos = rhs_[it-1](3) ; finPos = rhs_[it](3) ;
    RHOMEGA_->SetParameters(TimeInterval,iniPos,finPos,ToMP,MP);


    double UnlockedSwingPeriod = LFX_->FT() ;
    double ss_time = LFZ_->FT() ;
    double EndOfLiftOff = (ss_time-UnlockedSwingPeriod)*0.5;
    double StartLanding = EndOfLiftOff + UnlockedSwingPeriod;

    double timeOfInterpolation = 0.0 ;

    MAL_VECTOR_DIM(aLF,double,5);
    MAL_VECTOR_DIM(aRF,double,5);
    MAL_VECTOR_DIM(aRH,double,5);

    for(int i = 0 ; i < (int)round(timing_[it]/samplingPeriod_) ; ++i )
      {
        if(i*samplingPeriod_ < EndOfLiftOff)
          {
            timeOfInterpolation = 0.0 ;
          }
        else if (i*samplingPeriod_ < StartLanding)
          {
            timeOfInterpolation = i*samplingPeriod_ - EndOfLiftOff ;
          }
        else
          {
            timeOfInterpolation = UnlockedSwingPeriod ;
          }

        double tmp,dtmp,ddtmp;
        LFX_->Compute(timeOfInterpolation,tmp,dtmp,ddtmp);
        aLF(0) = tmp ;
        LFY_->Compute(timeOfInterpolation,tmp,dtmp,ddtmp);
        aLF(1) = tmp ;
        LFZ_->Compute(timeOfInterpolation,tmp,dtmp,ddtmp);
        aLF(2) = tmp ;
        aLF(3) = 0.0;
        aLF(4) = 0.0;

        RFX_->Compute(timeOfInterpolation,tmp,dtmp,ddtmp);
        aRF(0) = tmp ;
        RFY_->Compute(timeOfInterpolation,tmp,dtmp,ddtmp);
        aRF(1) = tmp ;
        RFZ_->Compute(timeOfInterpolation,tmp,dtmp,ddtmp);
        aRF(2) = tmp ;
        aRF(3) = 0.0;
        aRF(4) = 0.0;

        RHX_->Compute(timeOfInterpolation,tmp,dtmp,ddtmp);
        aRH(0) = tmp ;
        RHY_->Compute(timeOfInterpolation,tmp,dtmp,ddtmp);
        aRH(1) = tmp ;
        RHZ_->Compute(timeOfInterpolation,tmp,dtmp,ddtmp);
        aRH(2) = tmp ;
        RHOMEGA_->Compute(timeOfInterpolation,tmp,dtmp,ddtmp);
        aRH(3) = tmp ;
        aRH(4) = 0.0;

        lfoot_.push_back(aLF) ;
        rfoot_.push_back(aRF) ;
        rhand_.push_back(aRH) ;
      }

    return 1 ;
  }

  int com_sampling_correction ()
  {
    deque<MAL_VECTOR_TYPE(double)> com_1ms ;
    deque<MAL_VECTOR_TYPE(double)> com_5ms ;

    double sampling_time = data_time_[1] - data_time_[0] ;
    if (sampling_time - 0.001 < 0.0001 )
      {
        com_1ms = comPos_ ;
      }
    else
      {
          com_1ms.clear();
          for (unsigned int i = 0 ; i < comPos_.size()-1 ; ++i)
            {
              com_1ms.push_back(comPos_[i]);
              com_1ms.push_back( 0.5*(comPos_[i]+comPos_[i+1]) );
            }
      }

    for (unsigned int i = 0 ; i < com_1ms.size() ; i=i+5)
      {
        com_5ms.push_back(com_1ms[i]);
      }

    comPos_ = com_5ms ;
    comPos_.pop_back();
    for (unsigned int i = 0 ; i < initialTime_/samplingPeriod_ ; ++i)
      {
        comPos_.push_front(comPos_.front());
      }
    for (unsigned int i = 0 ; i < finalTime_/samplingPeriod_ ; ++i)
      {
        comPos_.push_back(comPos_.back());
      }

    comSpeed_.resize(comPos_.size());
    comAcc_.resize(comPos_.size());
    data_time_.resize(comPos_.size());
    for(unsigned int i = 0 ; i < data_time_.size() ; ++i)
      data_time_[i] = (double)i * samplingPeriod_ ;


    return 1 ;
  }

  int init_interpolation ()
  {
    vector<double> MP ;
    vector<double> ToMP ;

    LFX_ = new BSplinesFoot() ;
    LFY_ = new BSplinesFoot() ;
    LFZ_ = new BSplinesFoot() ;

    RFX_ = new BSplinesFoot() ;
    RFY_ = new BSplinesFoot() ;
    RFZ_ = new BSplinesFoot() ;

    RHX_ = new BSplinesFoot() ;
    RHY_ = new BSplinesFoot() ;
    RHZ_ = new BSplinesFoot() ;
    RHOMEGA_ = new BSplinesFoot() ;


    //                    time     InitPosition   FinalPosition  InitSpeed
    LFX_->SetParameters(timing_[0],lfs_[0](0),lfs_[0](0),ToMP,MP);
    LFY_->SetParameters(timing_[0],lfs_[0](1),lfs_[0](1),ToMP,MP);
    LFZ_->SetParameters(timing_[0],lfs_[0](2),lfs_[0](2),ToMP,MP);

    RFX_->SetParameters(timing_[0],rfs_[0](0),rfs_[0](0),ToMP,MP);
    RFY_->SetParameters(timing_[0],rfs_[0](1),rfs_[0](1),ToMP,MP);
    RFZ_->SetParameters(timing_[0],rfs_[0](2),rfs_[0](2),ToMP,MP);

    RHX_->SetParameters(timing_[0],rhs_[0](0),rhs_[0](0),ToMP,MP);
    RHY_->SetParameters(timing_[0],rhs_[0](1),rhs_[0](1),ToMP,MP);
    RHZ_->SetParameters(timing_[0],rhs_[0](2),rhs_[0](2),ToMP,MP);
    RHOMEGA_->SetParameters(timing_[0],rhs_[0](3),rhs_[0](3),ToMP,MP);
    return 1;
  }

  int init_support()
  {
    lfs_.clear(); rfs_.clear(); rhs_.clear();
    timing_.clear();

    movingPeriod_  = 1.4 ;
    supportPeriod_ = 0.1 ;
    initialTime_   = 5.0 ;
    finalTime_     = 5.0 ;
    samplingPeriod_ = 0.005 ;

    MAL_VECTOR_DIM(aSLF,double,5);
    MAL_VECTOR_DIM(aSRF,double,5);
    MAL_VECTOR_DIM(aSRH,double,5);

    double init_lfx = 0.00949035 ;
    double init_lfy = 0.095      ;
    double init_lfz = 0.0        ;
                      //
    double init_rfx = 0.00949035 ;
    double init_rfy = -0.095     ;
    double init_rfz = 0.0        ;

    double init_rhx = 0.0418343 ;
    double init_rhy = -0.331008 ;
    double init_rhz = 0.704285  ;

    // 0     5.0 second
    aSLF(0)=init_lfx ; aSRF(0)=init_rfx;aSRH(0)=init_rhx   ;
    aSLF(1)=init_lfy ; aSRF(1)=init_rfy;aSRH(1)=init_rhy   ;
    aSLF(2)=init_lfz ; aSRF(2)=init_rfz;aSRH(2)=init_rhz   ;
    aSLF(3)=0.0 ;      aSRF(3)=0.0 ;    aSRH(3)=0.0         ;
    aSLF(4)=0.0 ;      aSRF(4)=0.0 ;    aSRH(4)=0.0         ;
    lfs_.push_back(aSLF); rfs_.push_back(aSRF); rhs_.push_back(aSRH);
    timing_.push_back(initialTime_+supportPeriod_/2);
//
    // 1     0.7 second
    aSLF(0)=init_lfx ; aSRF(0)=init_rfx;aSRH(0)=0.3         ;
    aSLF(1)=init_lfy ; aSRF(1)=init_rfy;aSRH(1)=init_rhy   ;
    aSLF(2)=init_lfz ; aSRF(2)=init_rfz;aSRH(2)=0.705+1*0.1 ;
    aSLF(3)=0.0 ;      aSRF(3)=0.0 ;    aSRH(3)=-0.451026812;
    aSLF(4)=0.0 ;      aSRF(4)=0.0 ;    aSRH(4)=0.0         ;
    lfs_.push_back(aSLF); rfs_.push_back(aSRF); rhs_.push_back(aSRH);
    timing_.push_back(movingPeriod_);

    // 2     0.1 s
    aSLF(0)=init_lfx ; aSRF(0)=init_rfx;aSRH(0)=0.3         ;
    aSLF(1)=init_lfy ; aSRF(1)=init_rfy;aSRH(1)=init_rhy   ;
    aSLF(2)=init_lfz ; aSRF(2)=init_rfz;aSRH(2)=0.705+1*0.1 ;
    aSLF(3)=0.0 ;      aSRF(3)=0.0 ;    aSRH(3)=-0.451026812;
    aSLF(4)=0.0 ;      aSRF(4)=0.0 ;    aSRH(4)=0.0         ;
    lfs_.push_back(aSLF); rfs_.push_back(aSRF); rhs_.push_back(aSRH);
    timing_.push_back(supportPeriod_);

    // 3     0.7 s
    aSLF(0)=init_lfx+0.3 ; aSRF(0)=init_rfx;aSRH(0)=0.3         ;
    aSLF(1)=init_lfy ;     aSRF(1)=init_rfy;aSRH(1)=init_rhy   ;
    aSLF(2)=0.1 ;          aSRF(2)=init_rfz;aSRH(2)=0.705+1*0.1 ;
    aSLF(3)=0.0 ;          aSRF(3)=0.0 ;    aSRH(3)=-0.451026812;
    aSLF(4)=0.0 ;          aSRF(4)=0.0 ;    aSRH(4)=0.0         ;
    lfs_.push_back(aSLF); rfs_.push_back(aSRF); rhs_.push_back(aSRH);
    timing_.push_back(movingPeriod_);

    // 4    0.1 s
    aSLF(0)=init_lfx+0.3 ; aSRF(0)=init_rfx;aSRH(0)=0.3         ;
    aSLF(1)=init_lfy ;     aSRF(1)=init_rfy;aSRH(1)=init_rhy   ;
    aSLF(2)=0.1 ;          aSRF(2)=init_rfz;aSRH(2)=0.705+1*0.1 ;
    aSLF(3)=0.0 ;          aSRF(3)=0.0 ;    aSRH(3)=-0.451026812;
    aSLF(4)=0.0 ;          aSRF(4)=0.0 ;    aSRH(4)=0.0         ;
    lfs_.push_back(aSLF); rfs_.push_back(aSRF); rhs_.push_back(aSRH);
    timing_.push_back(supportPeriod_);

    // 5    0.7 s
    aSLF(0)=init_lfx+0.3 ; aSRF(0)=init_rfx+0.3 ;  aSRH(0)=0.3         ;
    aSLF(1)=init_lfy ;     aSRF(1)=init_rfy     ;  aSRH(1)=init_rhy  ;
    aSLF(2)=0.1 ;          aSRF(2)=0.1 ;           aSRH(2)=0.705+1*0.1 ;
    aSLF(3)=0.0 ;          aSRF(3)=0.0 ;           aSRH(3)=-0.451026812;
    aSLF(4)=0.0 ;          aSRF(4)=0.0 ;           aSRH(4)=0.0         ;
    lfs_.push_back(aSLF); rfs_.push_back(aSRF); rhs_.push_back(aSRH);
    timing_.push_back(movingPeriod_);

    // 6    0.1 s
    aSLF(0)=init_lfx+0.3 ; aSRF(0)=init_rfx+0.3 ;  aSRH(0)=0.3         ;
    aSLF(1)=init_lfy ;     aSRF(1)=init_rfy     ;  aSRH(1)=init_rhy   ;
    aSLF(2)=0.1 ;          aSRF(2)=0.1 ;           aSRH(2)=0.705+1*0.1 ;
    aSLF(3)=0.0 ;          aSRF(3)=0.0 ;           aSRH(3)=-0.451026812;
    aSLF(4)=0.0 ;          aSRF(4)=0.0 ;           aSRH(4)=0.0         ;
    lfs_.push_back(aSLF); rfs_.push_back(aSRF); rhs_.push_back(aSRH);
    timing_.push_back(supportPeriod_);
//
    // 7    0.7 s
    aSLF(0)=init_lfx+0.3 ;aSRF(0)=init_rfx+0.3 ;  aSRH(0)=0.6         ;
    aSLF(1)=init_lfy ;    aSRF(1)=init_rfy     ;  aSRH(1)=init_rhy   ;
    aSLF(2)=0.1 ;         aSRF(2)=0.1 ;           aSRH(2)=0.705+2*0.1 ;
    aSLF(3)=0.0 ;         aSRF(3)=0.0 ;           aSRH(3)=-0.451026812;
    aSLF(4)=0.0 ;         aSRF(4)=0.0 ;           aSRH(4)=0.0         ;
    lfs_.push_back(aSLF); rfs_.push_back(aSRF); rhs_.push_back(aSRH);
    timing_.push_back(movingPeriod_);

    // 8    0.1 s
    aSLF(0)=init_lfx+0.3 ; aSRF(0)=init_rfx+0.3 ;  aSRH(0)=0.6         ;
    aSLF(1)=init_lfy ;     aSRF(1)=init_rfy     ;  aSRH(1)=init_rhy   ;
    aSLF(2)=0.1 ;          aSRF(2)=0.1 ;           aSRH(2)=0.705+2*0.1 ;
    aSLF(3)=0.0 ;          aSRF(3)=0.0 ;           aSRH(3)=-0.451026812;
    aSLF(4)=0.0 ;          aSRF(4)=0.0 ;           aSRH(4)=0.0         ;
    lfs_.push_back(aSLF); rfs_.push_back(aSRF); rhs_.push_back(aSRH);
    timing_.push_back(supportPeriod_);

    // 9    0.7 s
    aSLF(0)=init_lfx+0.6 ;aSRF(0)=init_rfx+0.3 ;  aSRH(0)=0.6         ;
    aSLF(1)=init_lfy ;    aSRF(1)=init_rfy     ;  aSRH(1)=init_rhy   ;
    aSLF(2)=0.2 ;         aSRF(2)=0.1 ;           aSRH(2)=0.705+2*0.1 ;
    aSLF(3)=0.0 ;         aSRF(3)=0.0 ;           aSRH(3)=-0.451026812;
    aSLF(4)=0.0 ;         aSRF(4)=0.0 ;           aSRH(4)=0.0         ;
    lfs_.push_back(aSLF); rfs_.push_back(aSRF); rhs_.push_back(aSRH);
    timing_.push_back(movingPeriod_);

    // 10    0.1 s
    aSLF(0)=init_lfx+0.6 ; aSRF(0)=init_rfx+0.3 ;  aSRH(0)=0.6         ;
    aSLF(1)=init_lfy ;     aSRF(1)=init_rfy     ;  aSRH(1)=init_rhy   ;
    aSLF(2)=0.2 ;          aSRF(2)=0.1 ;           aSRH(2)=0.705+2*0.1 ;
    aSLF(3)=0.0 ;          aSRF(3)=0.0 ;           aSRH(3)=-0.451026812;
    aSLF(4)=0.0 ;          aSRF(4)=0.0 ;           aSRH(4)=0.0         ;
    lfs_.push_back(aSLF); rfs_.push_back(aSRF); rhs_.push_back(aSRH);
    timing_.push_back(supportPeriod_);

    // 11    0.7 s
    aSLF(0)=init_lfx+0.6 ; aSRF(0)=init_rfx+0.6 ;  aSRH(0)=0.6         ;
    aSLF(1)=init_lfy ;     aSRF(1)=init_rfy     ;  aSRH(1)=init_rhy   ;
    aSLF(2)=0.2 ;          aSRF(2)=0.2 ;           aSRH(2)=0.705+2*0.1 ;
    aSLF(3)=0.0 ;          aSRF(3)=0.0 ;           aSRH(3)=-0.451026812;
    aSLF(4)=0.0 ;          aSRF(4)=0.0 ;           aSRH(4)=0.0         ;
    lfs_.push_back(aSLF); rfs_.push_back(aSRF); rhs_.push_back(aSRH);
    timing_.push_back(movingPeriod_);

    // 12    0.1 s
    aSLF(0)=init_lfx+0.6 ; aSRF(0)=init_rfx+0.6 ;  aSRH(0)=0.6         ;
    aSLF(1)=init_lfy ;     aSRF(1)=init_rfy     ;  aSRH(1)=init_rhy   ;
    aSLF(2)=0.2 ;          aSRF(2)=0.2 ;           aSRH(2)=0.705+2*0.1 ;
    aSLF(3)=0.0 ;          aSRF(3)=0.0 ;           aSRH(3)=-0.451026812;
    aSLF(4)=0.0 ;          aSRF(4)=0.0 ;           aSRH(4)=0.0         ;
    lfs_.push_back(aSLF); rfs_.push_back(aSRF); rhs_.push_back(aSRH);
    //timing_.push_back(supportPeriod_);
    timing_.push_back(finalTime_+supportPeriod_/2);

    return 1 ;
  }

  int readData()
  {
    //std::string dataPath = "/home/mnaveau/devel/mkudruss_data/2015_03_03_18h56m/" ;
    //std::string dataFile = dataPath + "conv_sd_walking_stair_climbing_2_steps_10cm.csv" ;
    //std::string dataPath = "/home/mnaveau/devel/mkudruss_data/2015_03_03_21h_01m/" ;
    //std::string dataFile = dataPath + "conv_sd_walking_stair_climbing_2_steps_10cm_ds.csv" ;
//    std::string dataPath = "/home/mnaveau/devel/mkudruss_data/2015_03_27_10h35m/" ;
//    std::string dataFile = dataPath + "conv_sd_walking_stair_climbing_2_steps_10cm.csv" ;

    //std::string dataPath = "/home/mnaveau/devel/mkudruss_data/2015_04_27_10h_43m/" ;
    //std::string dataFile = dataPath + "conv_sd_walking_stair_climbing_2_steps_10cm_ds.csv" ;

    std::string dataPath = "/home/mnaveau/devel/mkudruss_data/2015_06_04_14h_12m/" ;
    std::string dataFile = dataPath + "conv_sd_walking_stair_climbing_2_steps_10cm_ds.csv" ;

    std::ifstream dataStream ;
    dataStream.open(dataFile.c_str(),std::ifstream::in);
    if (!dataStream.good())
      cout << "cannot open the file" << endl ;
    MAL_VECTOR_DIM(acomPos,double,6);
    MAL_VECTOR_DIM(acomVel,double,6);
    MAL_VECTOR_DIM(acomAcc,double,6);

    data_time_.clear();
    comPos_.clear();
    comSpeed_.clear();
    comAcc_.clear();

    assert(dataStream.good());
    while(dataStream.good())
      {
        double value ;
        dataStream >> value ;
        data_time_.push_back(value);

        dataStream >> acomPos(0) ;
        dataStream >> acomPos(1) ;
        dataStream >> acomPos(2) ;
        dataStream >> acomPos(3) ;
        dataStream >> acomPos(4) ;
        dataStream >> acomPos(5) ;

        acomPos(3) = acomPos(3) *180/M_PI ;
        acomPos(4) = acomPos(4) *180/M_PI ;
        acomPos(5) = acomPos(5) *180/M_PI ;

        comPos_.push_back(acomPos);
        comSpeed_.push_back(acomVel);
        comAcc_.push_back(acomAcc);

        for (unsigned int i = 0 ; i < 10 ; ++i)
          {
            dataStream >> value ;
          }
      }
    dataStream.close();

    data_time_.pop_back();
    comPos_   .pop_back();
    comSpeed_ .pop_back();
    comAcc_   .pop_back();


    return 0;
  }

  void chooseTestProfile()
  {return;}
  void generateEvent()
  {return;}

  void SpecializedRobotConstructor(CjrlHumanoidDynamicRobot *& aHDR)
  {
    dynamicsJRLJapan::ObjectFactory aRobotDynamicsObjectConstructor;
    Chrp2OptHumanoidDynamicRobot *aHRP2HDR = new Chrp2OptHumanoidDynamicRobot( &aRobotDynamicsObjectConstructor );
    aHDR = aHRP2HDR;
  }

  void initIK()
  {
    MAL_VECTOR_DIM(BodyAngles,double,MAL_VECTOR_SIZE(InitialPosition));
    MAL_VECTOR_DIM(waist,double,6);
    for (int i = 0 ; i < 6 ; ++i )
    {
      waist(i) = 0;
    }
    for (unsigned int i = 0 ; i < MAL_VECTOR_SIZE(InitialPosition) ; ++i )
    {
      BodyAngles(i) = InitialPosition(i);
    }
    MAL_S3_VECTOR(lStartingCOMState,double);
    CFRG_ = new ComAndFootRealizationByGeometry((PatternGeneratorInterfacePrivate*)(SPM_));
    CFRG_->setHumanoidDynamicRobot(m_HDR);
    CFRG_->SetHeightOfTheCoM(0.814);
    CFRG_->setSamplingPeriod(samplingPeriod_);
    CFRG_->SetStepStackHandler(new StepStackHandler(SPM_));
    CFRG_->Initialization();
    CFRG_->InitializationCoM(BodyAngles,lStartingCOMState,
                            waist, m_OneStep.LeftFootPosition,
                            m_OneStep.RightFootPosition);
    CFRG_->Initialization();
    CFRG_->SetPreviousConfigurationStage0(m_HDR->currentConfiguration());
    CFRG_->SetPreviousVelocityStage0(m_HDR->currentVelocity());
  }

  void fillInDebugFiles( )
  {
    TestObject::fillInDebugFiles();

    /// \brief Create file .hip/.waist .pos .zmp
    /// --------------------
    ofstream aof;
    string aFileName;
    static int iteration = 0 ;

    if ( iteration == 0 ){
      aFileName = "/opt/grx3.0/HRP2LAAS/etc/mnaveau/";
      aFileName+=m_TestName;
      aFileName+=".pos";
      aof.open(aFileName.c_str(),ofstream::out);
      aof.close();
    }
    ///----
    aFileName = "/opt/grx3.0/HRP2LAAS/etc/mnaveau/";
      aFileName+=m_TestName;
      aFileName+=".pos";
    aof.open(aFileName.c_str(),ofstream::app);
    aof.precision(8);
    aof.setf(ios::scientific, ios::floatfield);
    aof << filterprecision( iteration * 0.005 ) << " "  ; // 1
    for(unsigned int i = 6 ; i < m_CurrentConfiguration.size() ; i++){
      aof << filterprecision( m_CurrentConfiguration(i) ) << " "  ; // 2
    }
    for(unsigned int i = 0 ; i < 9 ; i++){
      aof << 0.0 << " "  ;
    }
    aof << 0.0  << endl ;
    aof.close();

    if ( iteration == 0 ){
      aFileName = "/opt/grx3.0/HRP2LAAS/etc/mnaveau/";
      aFileName+=m_TestName;
      aFileName+=".hip";
      aof.open(aFileName.c_str(),ofstream::out);
      aof.close();
    }
    aFileName = "/opt/grx3.0/HRP2LAAS/etc/mnaveau/";
      aFileName+=m_TestName;
      aFileName+=".hip";
    aof.open(aFileName.c_str(),ofstream::app);
    aof.precision(8);
    aof.setf(ios::scientific, ios::floatfield);
      aof << filterprecision( iteration * 0.005 ) << " "  ; // 1
      aof << filterprecision( m_OneStep.finalCOMPosition.roll[0] * M_PI /180) << " "  ; // 2
      aof << filterprecision( m_OneStep.finalCOMPosition.pitch[0] * M_PI /180 ) << " "  ; // 3
      aof << filterprecision( m_OneStep.finalCOMPosition.yaw[0] * M_PI /180 ) ; // 4
      aof << endl ;
    aof.close();

    if ( iteration == 0 ){
      aFileName = "/opt/grx3.0/HRP2LAAS/etc/mnaveau/";
      aFileName+=m_TestName;
      aFileName+=".zmp";
      aof.open(aFileName.c_str(),ofstream::out);
      aof.close();
    }

    FootAbsolutePosition aSupportState;
    if (m_OneStep.LeftFootPosition.stepType < 0 )
      aSupportState = m_OneStep.LeftFootPosition ;
    else
      aSupportState = m_OneStep.RightFootPosition ;

    aFileName = "/opt/grx3.0/HRP2LAAS/etc/mnaveau/";
      aFileName+=m_TestName;
      aFileName+=".zmp";
    aof.open(aFileName.c_str(),ofstream::app);
    aof.precision(8);
    aof.setf(ios::scientific, ios::floatfield);
      aof << filterprecision( iteration * 0.005 ) << " "  ; // 1
      aof << filterprecision( m_OneStep.ZMPTarget(0) - m_CurrentConfiguration(0)) << " "  ; // 2
      aof << filterprecision( m_OneStep.ZMPTarget(1) - m_CurrentConfiguration(1) ) << " "  ; // 3
      aof << filterprecision( aSupportState.z  - m_CurrentConfiguration(2))  ; // 4
      aof << endl ;
    aof.close();

    iteration++;
  }

  void SpecializedRobotConstructor(   CjrlHumanoidDynamicRobot *& aHDR, CjrlHumanoidDynamicRobot *& aDebugHDR )
  {
    dynamicsJRLJapan::ObjectFactory aRobotDynamicsObjectConstructor;
    Chrp2OptHumanoidDynamicRobot *aHRP2HDR = new Chrp2OptHumanoidDynamicRobot( &aRobotDynamicsObjectConstructor );
    aHDR = aHRP2HDR;
    aDebugHDR = new Chrp2OptHumanoidDynamicRobot(&aRobotDynamicsObjectConstructor);
  }

  double filterprecision(double adb)
  {
    if (fabs(adb)<1e-7)
      return 0.0;

    double ladb2 = adb * 1e7;
    double lintadb2 = trunc(ladb2);
    return lintadb2/1e7;
  }
};

int PerformTests(int argc, char *argv[])
{
#define NB_PROFILES 1
  std::string TestNames = "TestMulticontactInverseGeometry" ;

  TestMulticontactInverseGeometry aTIK(argc,argv,TestNames);
  aTIK.init();
  try{
    if (!aTIK.doTest(std::cout)){
      cout << "Failed test " << endl;
      return -1;
    }
    else
      cout << "Passed test " << endl;
  }
  catch (const char * astr){
    cerr << "Failed on following error " << astr << std::endl;
    return -1; }

  return 0;
}

int main(int argc, char *argv[])
{
  try
  {
    int ret = PerformTests(argc,argv);
    return ret ;
  }
  catch (const std::string& msg)
  {
    std::cerr << msg << std::endl;
  }
  return 1;
}
