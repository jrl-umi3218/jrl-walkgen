/*
 * Copyright 2015,
 *
 * Maximilien Naveau
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
/*! \file DynamicFilter.hh
  \brief This object defines a dynamic filter that modify the CoM on the
ground plan taking into account the whole body motion */

#ifndef DYNAMICFILTER_HH 
#define DYNAMICFILTER_HH

#include <MotionGeneration/ComAndFootRealizationByGeometry.hh>
#include "Clock.hh"

namespace PatternGeneratorJRL
{

  class DynamicFilter
  {
  public: // Public methods

    /// \brief
    DynamicFilter(SimplePluginManager *SPM,
                  PinocchioRobot *aPR
                  );
    ~DynamicFilter();
    /// \brief
    int OffLinefilter(
        const deque<COMState> & inputCOMTraj_deq_,
        const deque<ZMPPosition> & inputZMPTraj_deq_,
        const deque<FootAbsolutePosition> & inputLeftFootTraj_deq_,
        const deque<FootAbsolutePosition> & inputRightFootTraj_deq_,
        const vector<MAL_VECTOR_TYPE(double) > &UpperPart_q,
        const vector<MAL_VECTOR_TYPE(double) > &UpperPart_dq,
        const vector<MAL_VECTOR_TYPE(double) > &UpperPart_ddq,
        deque<COMState> & outputDeltaCOMTraj_deq_);

    int OnLinefilter(const deque<COMState> & inputCOMTraj_deq_,
        const deque<ZMPPosition> & inputZMPTraj_deq_,
        const deque<FootAbsolutePosition> & inputLeftFootTraj_deq_,
        const deque<FootAbsolutePosition> & inputRightFootTraj_deq_,
        deque<COMState> & outputDeltaCOMTraj_deq_);

    void init(
        double controlPeriod,
        double interpolationPeriod,
        double controlWindowSize,
        double previewWindowSize,
        double kajitaPCwindowSize,
        COMState inputCoMState);

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
        MAL_S3_VECTOR_TYPE(double) & ZMPMB,
        unsigned int stage,
        unsigned int iteration);

    void stage0INstage1();

    /// \brief Preview control on the ZMPMBs computed
    int OptimalControl(deque<ZMPPosition> &inputdeltaZMP_deq,
        deque<COMState> & outputDeltaCOMTraj_deq_);

    /// \brief compute the zmpmb from articulated pos vel and acc
    int zmpmb(MAL_VECTOR_TYPE(double)& configuration,
              MAL_VECTOR_TYPE(double)& velocity,
              MAL_VECTOR_TYPE(double)& acceleration,
              MAL_S3_VECTOR_TYPE(double) & zmpmb);

  private: // Private methods

    /// \brief Apply the RNEA on the robot model and over the whole trajectory
    /// given by the function "filter"
    void InverseDynamics(MAL_VECTOR_TYPE(double)& configuration,
                         MAL_VECTOR_TYPE(double)& velocity,
                         MAL_VECTOR_TYPE(double)& acceleration);

    void computeWaist(const FootAbsolutePosition & inputLeftFoot) ;

    // -------------------------------------------------------------------

  public: // The accessors

    void setRobotUpperPart(const MAL_VECTOR_TYPE(double) & configuration,
                           const MAL_VECTOR_TYPE(double) & velocity,
                           const MAL_VECTOR_TYPE(double) & acceleration);

    /// \brief getter :
    inline ComAndFootRealizationByGeometry * getComAndFootRealization()
    { return comAndFootRealization_;}

    inline PinocchioRobot * getPinocchioRobot()
    { return PR_;}

    inline double getControlPeriod()
    {return controlPeriod_ ;}

    inline double getInterpolationPeriod()
    {return interpolationPeriod_ ;}

    inline double getPreviewWindowSize_()
    {return previewWindowSize_ ;}

    inline void getPCerror_(vector<double> & errx,vector<double> & erry)
    { errx = sxzmp_ ; erry = syzmp_ ; }

    inline Clock * getClock()
    { return &clock_ ; }

    inline deque< MAL_S3_VECTOR_TYPE(double) > zmpmb()
    { return ZMPMB_vec_ ; }

  private: // Private members

    /// \brief Time variables
    /// -----------------------------------
    ///
      /// \brief control period of the PG host
      double controlPeriod_;

      /// \brief Interpolation Period for the PG host preview window
      double interpolationPeriod_ ;

      /// \brief size of the kajita PC preview window in second
      double kajitaPCwindowSize_ ;

      /// \brief size of the window containing the controls in second
      double controlWindowSize_ ;

      /// \brief size of the preview window of the PG host in second
      double previewWindowSize_ ;

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

      MAL_VECTOR_TYPE(double) upperPartConfiguration_ ;
      MAL_VECTOR_TYPE(double) previousUpperPartConfiguration_ ;
      MAL_VECTOR_TYPE(double) upperPartVelocity_ ;
      MAL_VECTOR_TYPE(double) previousUpperPartVelocity_ ;
      MAL_VECTOR_TYPE(double) upperPartAcceleration_ ;
      std::vector <unsigned int> upperPartIndex ;
      bool walkingHeuristic_ ;

      /// Class that compute the dynamic and kinematic of the robot
      PinocchioRobot * PR_ ;

      /// \brief Buffers the ZMP Multibody computed
      /// from the inverse Dynamics, and the difference between
      /// this zmp and the reference one.
      /// sampled at interpolation sampling period
      deque< MAL_S3_VECTOR_TYPE(double) > ZMPMB_vec_ ;
      /// sampled at control sampling period
      deque< MAL_S3_VECTOR_TYPE(double) > zmpmb_i_ ;
      /// sampled at control sampling period
      std::deque<ZMPPosition> deltaZMP_deq_ ;

    /// \brief Optimal Control variables
    /// --------------------------------
      /// \brief Pointer to the Preview Control object.
      PreviewControl *PC_;
      /// \brief data needed by the preview control algorithm
      vector<double> sxzmp_ , syzmp_ ;
      vector<double> deltaZMPx_, deltaZMPy_ ;
      double CoMHeight_ ;

      /// \brief State of the Preview control.
      MAL_MATRIX(deltax_,double);
      MAL_MATRIX(deltay_,double);

      /// \brief time measurement
      Clock clock_;

      /// \brief Stages, used in the analytical inverse kinematic.
      const unsigned int stage0_ ;
      const unsigned int stage1_ ;

      const unsigned int MODE_PC_;

    public : // debug functions      
      // to use the vector of eigen used by metapod
      //EIGEN_MAKE_ALIGNED_OPERATOR_NEW

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
