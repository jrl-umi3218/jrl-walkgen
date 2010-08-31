/*
 * OrientationsPreview.h
 *
 *  Created on: Apr 26, 2010
 *      Author: andrei
 */

#ifndef ORIENTATIONSPREVIEW_H_
#define ORIENTATIONSPREVIEW_H_



/*! STL includes */
#include <deque>

/*! Framework includes */
#include <PreviewControl/SupportState.h>

/*! Framework includes */
#include <walkGenJrl/PGTypes.h>
#include <robotDynamics/jrlJoint.h>

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
			     const SupportState * Support,
			     std::deque<FootAbsolutePosition> &LeftFootAbsolutePositions,
			     std::deque<FootAbsolutePosition> &RightFootAbsolutePositions);

    void verifyAccelerationOfHipJoint(const ReferenceAbsoluteVelocity_t &Ref,
				      const COMState &TrunkState, COMState &TrunkStateT,
				      const SupportState * Support);

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


    bool verifyAngleOfHipJoint(const SupportState * Support,
			       const COMState &TrunkState, COMState &TrunkStateT,
			       double CurrentSupportFootAngle,
			       unsigned int StepNumber);

    void verifyVelocityOfHipJoint(const double &Time, COMState &TrunkStateT,
				  const double &PreviewedSupportFoot, const unsigned int &StepNumber,
				  const SupportState * Support,
				  const double &CurrentRightFootAngle, const double &CurrentLeftFootAngle,
				  const double &CurrentLeftFootVelocity,
				  const double &CurrentRightFootVelocity);

    double f(double a,double b,double c,double d,double x);

    double df(double a,double b,double c,double d,double x);

  };
};
#endif /* ORIENTATIONSPREVIEW_H_ */
