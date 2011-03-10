/*
 * Copyright 2010, 
 *
 * Mehdi    Benallegue
 * Andrei   Herdt
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
/*
 * OrientationsPreview.h
 */

#ifndef ORIENTATIONSPREVIEW_H_
#define ORIENTATIONSPREVIEW_H_



/*! STL includes */
#include <deque>

/*! Framework includes */
#include <PreviewControl/SupportFSM.h>

/*! Framework includes */
#include <jrl/walkgen/pgtypes.hh>
#include <abstract-robot-dynamics/joint.hh>

namespace PatternGeneratorJRL
{
  class OrientationsPreview {
  public:
    OrientationsPreview(const double & SamplingPeriod,
			const unsigned int & SamplingsPreviewed, const double & SSPeriod,
			CjrlJoint *aLeg);
    ~OrientationsPreview();

    void previewOrientations(const double &Time,
			     std::deque<double> &PreviewedSupportAngles,
			     const COMState &TrunkState, COMState &TrunkStateT,
			     double StepDuration, support_state_t CurrentSupport,
			     std::deque<FootAbsolutePosition> &LeftFootAbsolutePositions,
			     std::deque<FootAbsolutePosition> &RightFootAbsolutePositions);

    void verifyAccelerationOfHipJoint(const reference_t &Ref,
				      const COMState &TrunkState, COMState &TrunkStateT,
				      support_state_t CurrentSupport);

  private:
    /*! Angular limitations of the hip joints*/
    double m_lLimitLeftHipYaw, m_uLimitLeftHipYaw, m_lLimitRightHipYaw, m_uLimitRightHipYaw;

    /*! Maximal acceleration of a hip joint*/
    double m_uaLimitHipYaw;

    /*! Upper crossing angle limit between the feet*/
    double m_uLimitFeet;

    /*! Maximal velocity of a foot*/
    double m_uvLimitFoot;

    /*! Single-support duration*/
    double m_SSPeriod;

    /*! Number of sampling in a preview window*/
    double m_N;

    /*! Time between two samplings*/
    double m_T;

    //Polynomial coefficients
    double m_a, m_b, m_c, m_d, m_e;

    //Rotation sense of the trunks angular velocity and acceleration
    double m_signRotVelTrunk, m_signRotAccTrunk;

    //Time period between now and the end of the support phase
    double m_SupportTimePassed;

    bool m_TrunkVelOK, m_TrunkAngleOK;

    /*! In case of double support the next support angle is fixed*/
    unsigned int m_FirstPreviewedFoot;

    /*! Trunkangle at the end of the current support phase*/
    double m_PreviewedTrunkAngleEnd;

    const static double M_EPS;

    /*! The angles of the support and non-support foot*/
    double m_PreviewedMovingAngle, m_PreviewedSupportAngle;

    double m_PreviewedRightFootAngle, m_PreviewedLeftFootAngle;

    double m_PreviousSupportAngle;

    double m_CurrentSupportAngle;

    double m_MeanFootVelDifference;

    /*! Which foot is on the ground in the preview period*/
    int m_PreviewedSupportFoot;


    unsigned int m_FullDebug;


    bool verifyAngleOfHipJoint(support_state_t CurrentSupport,
			       const COMState &TrunkState, COMState &TrunkStateT,
			       double CurrentSupportFootAngle,
			       unsigned int StepNumber);

    void verifyVelocityOfHipJoint(const double &Time, COMState &TrunkStateT,
				  const double &PreviewedSupportFoot, const unsigned int &StepNumber,
				  support_state_t CurrentSupport,
				  const double &CurrentRightFootAngle, const double &CurrentLeftFootAngle,
				  const double &CurrentLeftFootVelocity,
				  const double &CurrentRightFootVelocity);

    double f(double a,double b,double c,double d,double x);

    double df(double a,double b,double c,double d,double x);

  };
}
#endif /* ORIENTATIONSPREVIEW_H_ */
