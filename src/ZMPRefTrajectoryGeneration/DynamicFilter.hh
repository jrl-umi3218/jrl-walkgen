#ifndef DYNAMICFILTER_HH
#define DYNAMICFILTER_HH

// metapod includes
#include <metapod/models/hrp2_14/hrp2_14.hh>
#include <MotionGeneration/ComAndFootRealizationByGeometry.hh>
#include <metapod/algos/jac_point_chain.hh>
#include "Clock.hh"
#include <boost/fusion/algorithm/iteration/accumulate.hpp>
#include <boost/fusion/include/accumulate.hpp>

#ifndef METAPOD_TYPEDEF
#define METAPOD_TYPEDEF
    typedef double LocalFloatType;
    typedef metapod::Spatial::ForceTpl<LocalFloatType> Force_HRP2_14;
    typedef metapod::hrp2_14<LocalFloatType> Robot_Model;

    typedef metapod::Nodes< Robot_Model, Robot_Model::BODY >::type RootNode;
    typedef metapod::Nodes< Robot_Model, Robot_Model::l_wrist >::type LhandNode;
    typedef metapod::Nodes< Robot_Model, Robot_Model::r_wrist >::type RhandNode;
    typedef metapod::Nodes< Robot_Model, Robot_Model::LARM_LINK0 >::type LshoulderNode;
    typedef metapod::Nodes< Robot_Model, Robot_Model::RARM_LINK0 >::type RshoulderNode;

    typedef metapod::Nodes< Robot_Model, Robot_Model::LLEG_LINK0 >::type LhipNode;
    typedef metapod::Nodes< Robot_Model, Robot_Model::RLEG_LINK0 >::type RhipNode;
    typedef metapod::Nodes< Robot_Model, Robot_Model::l_ankle >::type LankleNode;
    typedef metapod::Nodes< Robot_Model, Robot_Model::r_ankle >::type RankleNode;

    typedef metapod::jac_point_chain < Robot_Model,
    Robot_Model::l_ankle, Robot_Model::LLEG_LINK0,0,true,false> Jac_LF;
    typedef metapod::jac_point_chain < Robot_Model,
    Robot_Model::r_ankle, Robot_Model::RLEG_LINK0,0,true,false> Jac_RF;

    typedef metapod::jac_point_chain < Robot_Model,
    Robot_Model::l_wrist, Robot_Model::LARM_LINK0,0,true,false> Jac_LH;
    typedef metapod::jac_point_chain < Robot_Model,
    Robot_Model::r_wrist, Robot_Model::RARM_LINK0,0,true,false> Jac_RH;

#endif

namespace PatternGeneratorJRL
{

  class DynamicFilter
  {
  public: // Public methods

    // to use the vector of eigen used by metapod
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    /// \brief
    DynamicFilter(SimplePluginManager *SPM,
                  CjrlHumanoidDynamicRobot *aHS
                  );
    ~DynamicFilter();
    /// \brief
    int OffLinefilter(
        const double currentTime,
        const deque<COMState> & inputCOMTraj_deq_,
        const deque<ZMPPosition> & inputZMPTraj_deq_,
        const deque<FootAbsolutePosition> & inputLeftFootTraj_deq_,
        const deque<FootAbsolutePosition> & inputRightFootTraj_deq_,
        const vector<MAL_VECTOR_TYPE(double) > &UpperPart_q,
        const vector<MAL_VECTOR_TYPE(double) > &UpperPart_dq,
        const vector<MAL_VECTOR_TYPE(double) > &UpperPart_ddq,
        deque<COMState> & outputDeltaCOMTraj_deq_);

    int OnLinefilter(const double currentTime,
        const deque<COMState> & inputCOMTraj_deq_,
        const deque<ZMPPosition> inputZMPTraj_deq_,
        const deque<FootAbsolutePosition> & inputLeftFootTraj_deq_,
        const deque<FootAbsolutePosition> & inputRightFootTraj_deq_,
        deque<COMState> & outputDeltaCOMTraj_deq_);

    void init(
        double controlPeriod,
        double interpolationPeriod,
        double PG_T,
        double previewWindowSize,
        double CoMHeight,
        FootAbsolutePosition supportFoot,
        COMState inputCoM
        );

    /// \brief atomic function
    void InverseKinematics(
        const COMState & inputCoMState,
        const FootAbsolutePosition & inputLeftFoot,
        const FootAbsolutePosition & inputRightFoot,
        MAL_VECTOR_TYPE(double) & configuration,
        MAL_VECTOR_TYPE(double) & velocity,
        MAL_VECTOR_TYPE(double) & acceleration,
        double samplingPeriod,
        int stage,
        int iteration);

    /// \brief atomic function allow to compute
    void ComputeZMPMB(
        double samplingPeriod,
        const COMState & inputCoMState,
        const FootAbsolutePosition & inputLeftFoot,
        const FootAbsolutePosition & inputRightFoot,
        vector<double> & ZMPMB,
        unsigned int stage,
        unsigned int iteration);

    void stage0INstage1();

    /// \brief Preview control on the ZMPMBs computed
    int OptimalControl(
        deque<ZMPPosition> & inputdeltaZMP_deq,
        deque<COMState> & outputDeltaCOMTraj_deq_);

  private: // Private methods

    /// \brief Apply the RNEA on the robot model and over the whole trajectory
    /// given by the function "filter"
    void InverseDynamics(MAL_VECTOR_TYPE(double)& configuration,
                         MAL_VECTOR_TYPE(double)& velocity,
                         MAL_VECTOR_TYPE(double)& acceleration);

    void ExtractZMP(vector<double> & ZMPMB) ;

    void computeWaist(const FootAbsolutePosition & inputLeftFoot) ;

    // -------------------------------------------------------------------

  public: // The accessors

    /// \brief setter :
    inline void setControlPeriod(double controlPeriod)
    {controlPeriod_ = controlPeriod ;}

    inline void setInterpolationPeriod(double interpolationPeriod)
    {interpolationPeriod_ = interpolationPeriod ; return ;}

    inline void setPGPeriod(double PG_T)
    {PG_T_ = PG_T ;}

    inline void setPreviewWindowSize_(double previewWindowSize)
    { previewWindowSize_ = previewWindowSize; }

    void setRobotUpperPart(const MAL_VECTOR_TYPE(double) & configuration,
                           const MAL_VECTOR_TYPE(double) & velocity,
                           const MAL_VECTOR_TYPE(double) & acceleration);

    /// \brief getter :
    inline ComAndFootRealizationByGeometry * getComAndFootRealization()
    { return comAndFootRealization_;}

    inline double getControlPeriod()
    {return controlPeriod_ ;}

    inline double getInterpolationPeriod()
    {return interpolationPeriod_ ;}

    inline double getPreviewWindowSize_()
    {return previewWindowSize_ ;}

    inline void getPCerror_(double & errx, double & erry)
    { errx = sxzmp_ ; erry = syzmp_ ; }

    inline Clock * getClock()
    { return &clock_ ; }

    inline deque< vector<double> > zmpmb()
    { return ZMPMB_vec_ ; }

    inline metapod::Vector3dTpl< LocalFloatType >::Type com ()
    {
        double sum_mass = 0.0 ;
        metapod::Vector3dTpl< LocalFloatType >::Type com (0.0,0.0,0.0);
        const metapod::Vector3dTpl< LocalFloatType >::Type const_zero (0.0,0.0,0.0);
        const double init_sum = 0.0 ;
        sum_mass = boost::fusion::accumulate(robot_.nodes , init_sum  , MassSum()      );
        com      = boost::fusion::accumulate(robot_.nodes , const_zero, MassbyComSum() );
        return com / sum_mass ;
    }

    inline metapod::Vector3dTpl< LocalFloatType >::Type waist_pos ()
    {
      RootNode & node_waist = boost::fusion::at_c<Robot_Model::BODY>(robot_.nodes);
      return node_waist.body.iX0.r() ;
    }


  private: // Private members

    /// \brief Time variables
    /// -----------------------------------
    ///
      /// \brief control period of the PG host
      double controlPeriod_;

      /// \brief Interpolation Period for the dynamic filter
      double interpolationPeriod_ ;

      /// \brief Sampling period of the PG host
      double PG_T_;

      /// \brief size of the previw window in second
      double previewWindowSize_ ;

      //------------------------------------------------------
      /// \brief Contain the number of control points
      unsigned int NCtrl_;

      /// \brief Contain the number of interpolation points
      /// inside the Sampling period of the PG host
      unsigned int NbI_ ;

      /// \brief Nb. samplings inside preview window
      unsigned int PG_N_ ;

    /// \brief Inverse Kinematics variables
    /// -----------------------------------
      /// \brief Store a reference to the object to solve posture resolution.
      ComAndFootRealizationByGeometry * comAndFootRealization_;

      /// \brief Buffers for the Inverse Kinematics
      MAL_VECTOR_TYPE(double) aCoMState_;
      MAL_VECTOR_TYPE(double) aCoMSpeed_;
      MAL_VECTOR_TYPE(double) aCoMAcc_;
      MAL_VECTOR_TYPE(double) aLeftFootPosition_;
      MAL_VECTOR_TYPE(double) aRightFootPosition_;

      /// \brief used to compute the ZMPMB from only
      /// com and feet position from outside of the class
      MAL_VECTOR_TYPE(double) ZMPMBConfiguration_ ;
      MAL_VECTOR_TYPE(double) ZMPMBVelocity_ ;
      MAL_VECTOR_TYPE(double) ZMPMBAcceleration_ ;
      MAL_VECTOR_TYPE(double) previousZMPMBConfiguration_ ;
      MAL_VECTOR_TYPE(double) previousZMPMBVelocity_ ;
      MAL_VECTOR_TYPE(double) previousZMPMBAcceleration_ ;


      MAL_VECTOR_TYPE(double) upperPartConfiguration_ ;
      MAL_VECTOR_TYPE(double) previousUpperPartConfiguration_ ;
      MAL_VECTOR_TYPE(double) upperPartVelocity_ ;
      MAL_VECTOR_TYPE(double) previousUpperPartVelocity_ ;
      MAL_VECTOR_TYPE(double) upperPartAcceleration_ ;
      std::vector <unsigned int> upperPartIndex ;
      bool walkingHeuristic_ ;

      /// \brief data of the previous iteration
      bool PreviousSupportFoot_ ; // 1 = left ; 0 = right ;
      Robot_Model::confVector prev_q_, prev_dq_, prev_ddq_;
      double CWx, CWy;

    /// \brief Inverse Dynamics variables
    /// ---------------------------------
      /// \brief Initialize the robot with the autogenerated files
      /// by MetapodFromUrdf
      Robot_Model robot_;

      /// \brief Initialize the robot leg jacobians with the
      /// autogenerated files by MetapodFromUrdf
      Jac_LF::Jacobian jacobian_rf_ ;
      Jac_RF::Jacobian jacobian_lf_ ;

      /// \brief force acting on the CoM of the robot expressed
      /// in the Euclidean Frame
      Force_HRP2_14 m_force ;

      /// \brief Set of configuration vectors (q, dq, ddq, torques)
      Robot_Model::confVector q_, dq_, ddq_;

      /// \brief Used to eliminate the initiale difference between
      /// the zmp and the zmpmb
      bool Once_ ;
      double DInitX_, DInitY_ ;

      /// \brief Buffers the ZMP Multibody computed
      /// from the inverse Dynamics, and the difference between
      /// this zmp and the reference one.
      deque< vector<double> > ZMPMB_vec_ ;
      std::deque<ZMPPosition> deltaZMP_deq_ ;

    /// \brief Optimal Control variables
    /// --------------------------------
      /// \brief Pointer to the Preview Control object.
      PreviewControl *PC_;
      double sxzmp_ , syzmp_ ;
      double deltaZMPx_, deltaZMPy_ ;
      double CoMHeight_ ;

      /// \brief State of the Preview control.
      MAL_MATRIX(deltax_,double);
      MAL_MATRIX(deltay_,double);

      /// \brief time measurement
      Clock clock_;

      /// \brief Stages, used in the analytical inverse kinematic.
      const unsigned int stage0_ ;
      const unsigned int stage1_ ;

  private : // private struct
      struct MassSum
      {
          typedef LocalFloatType result_type;

          template <typename T>
          result_type operator()(const T & t , const result_type & sum_mass ) const
          {
              return ( sum_mass + Robot_Model::inertias[t.id].m() ) ;
          }

          template <typename T>
          result_type operator()(const result_type & sum_mass , const T & t ) const
          {
              return this->operator()(t,sum_mass);
          }
      };

      struct MassbyComSum
      {
          typedef metapod::Vector3dTpl< LocalFloatType >::Type result_type;

          template <typename T>
          result_type operator()(const T & t , const result_type & sum_h ) const
          {
              double mass = Robot_Model::inertias[t.id].m() ;
              return ( sum_h + mass * t.body.iX0.r() + t.body.iX0.E() * Robot_Model::inertias[t.id].h() );
          }
          template <typename T>
          result_type operator()(const result_type & sum_h , const T & t ) const
          {
               return this->operator ()(t,sum_h);
          }
      };

    public : // debug functions
      void Debug(const deque<COMState> & ctrlCoMState,
                 const deque<FootAbsolutePosition> & ctrlLeftFoot,
                 const deque<FootAbsolutePosition> & ctrlRightFoot,
                 const deque<COMState> & inputCOMTraj_deq_,
                 const deque<ZMPPosition> inputZMPTraj_deq_,
                 const deque<FootAbsolutePosition> & inputLeftFootTraj_deq_,
                 const deque<FootAbsolutePosition> & inputRightFootTraj_deq_,
                 const deque<COMState> &outputDeltaCOMTraj_deq_);
  };

}

#endif // DYNAMICFILTER_HH
