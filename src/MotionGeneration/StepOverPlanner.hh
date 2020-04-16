/*
 * Copyright 2005, 2006, 2007, 2008, 2009, 2010,
 *
 * Bjorn Verrelst
 * Francois Keith
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

/** \file StepOverPlanner.h
    \brief  This object generate all the values for the foot trajectories,
    and the desired ZMP based on a sequence of relative steps.
    If you want to change the reference trajectories, and the planning
    of the foot, this is the object to modify.
*/

#ifndef _STEPOVER_PLANNER_H_
#define _STEPOVER_PLANNER_H_

/*! System includes */
#include <deque>
#include <string>
#include <vector>

/*! MAL */

/*! Abstract Interface for dynamic robot. */
#include <jrl/walkgen/pinocchiorobot.hh>

/*! Humanoid Walking Pattern Generator */

#include <Mathematics/StepOverPolynome.hh>
#include <PreviewControl/PreviewControl.hh>

namespace PatternGeneratorJRL {
class ZMPDiscretization;
class CollisionDetector;
/*! \brief Structure to store the obstacle parameters and positions.*/
struct ObstaclePar_t {
  /*! \brief x, y in meters, theta in DEGREES. */
  double x, y, z, theta;
  /*! \brief  h,w,d in meters and represent the height, width and depth.*/
  double h, w, d;
};
typedef struct ObstaclePar_t ObstaclePar;

/*!\ingroup steppingover
  \brief Object to compute new foot trajectories to step over obstacle
  dynamically*/
class StepOverPlanner {
public:
  /*! \brief  Constructor */
  StepOverPlanner(ObstaclePar &ObstacleParameters, PinocchioRobot *aPR);

  /*! \brief  Destructor */
  ~StepOverPlanner();

  /*! \brief Method which calculates the different relative
    footholds to be set in function of an obstacle in front. */
  void CalculateFootHolds(deque<RelativeFootPosition> &FootHolds);

  /*! \brief Call for polynomial planning of both steps during the obstacle
    stepover */
  void PolyPlanner(deque<COMState> &aCOMBuffer,
                   deque<FootAbsolutePosition> &aLeftFootBuffer,
                   deque<FootAbsolutePosition> &aRightFootBuffer,
                   deque<ZMPPosition> &aZMPPositions);

  /*! function which calculates the polynomial coeficients
    for the first step*/
  void
  PolyPlannerFirstStep(deque<FootAbsolutePosition> &aFirstStepOverFootBuffer);

  /*! function which calculates the polynomial coeficients
    for the first step*/
  void
  PolyPlannerSecondStep(deque<FootAbsolutePosition> &aSecondStepOverFootBuffer);

  /*! function which calculates the polynomial coeficients
    for the changing COM height*/
  void PolyPlannerHip();

  /*! this sets the extra COM buffer calculated in the ZMPMultybody class*/
  void SetExtraBuffer(deque<COMState> aExtraCOMBuffer,
                      deque<FootAbsolutePosition> aExtraRightFootBuffer,
                      deque<FootAbsolutePosition> aExtraLeftFootBuffer);

  /*! this gets the extra COM buPreviewControlffer calculated
    in the ZMPMultybody class*/
  void GetExtraBuffer(deque<COMState> &aExtraCOMBuffer,
                      deque<FootAbsolutePosition> &aExtraRightFootBuffer,
                      deque<FootAbsolutePosition> &aExtraLeftFootBuffer);

  /*! this sets the extra COM buffer calculated in the ZMPMultybody class*/
  void SetFootBuffers(deque<FootAbsolutePosition> aLeftFootBuffer,
                      deque<FootAbsolutePosition> aRightFootBuffer);

  /*! this gets the extra COM buffer calculated in the ZMPMultybody class*/
  void GetFootBuffers(deque<FootAbsolutePosition> &aLeftFootBuffer,
                      deque<FootAbsolutePosition> &aRightFootBuffer);

  /*!  Set obstacle information.*/
  void SetObstacleInformation(ObstaclePar ObstacleParameters);

  /*!  Set the link to the preview control.*/
  void SetPreviewControl(PreviewControl *aPC);

  /*!  Set the link to the ZMP discretization model.*/
  void SetZMPDiscretization(ZMPDiscretization *aZMPDiscr);

  /*!  Set the link with the Dynamic Multi Body model.*/
  void SetDynamicMultiBodyModel(PinocchioRobot *aPR);

  /*! set parameters for the timedistribution of specific stepover points*/
  void TimeDistributeFactor(std::vector<double> &TimeDistrFactor);

  /*! set parameter which decreases the max stepover
    hipheight used during feasibility calculation*/
  void SetDeltaStepOverCOMHeightMax(double aDeltaStepOverCOMHeightMax);

  /*!  create the complete COM and ZMP buffer by the first preview round. */
  void CreateBufferFirstPreview(deque<COMState> &m_COMBuffer,
                                deque<ZMPPosition> &m_ZMPBuffer,
                                deque<ZMPPosition> &m_ZMPRefBuffer);

  /*! Calculates the absolute coordinates (ref frame)
    of a point on the lower legs given in relative coordinates
    in the locale frame
    (whichLeg positive for left leg and negative for right leg) */
  void CalcCoordShankLowerLegPoint(Eigen::MatrixXd RelCoord,
                                   Eigen::MatrixXd &AbsCoord,
                                   Eigen::MatrixXd LegAngles,
                                   Eigen::MatrixXd WaistRot,
                                   Eigen::MatrixXd WaistPos, int WhichLeg);

protected:
  /*! this function will calculate a feasible set
    for the stepleght and hip height during
    double support over the obstacle */
  void DoubleSupportFeasibility();

  /*! Read parameters from interpretor */
  void m_SetObstacleParameters(istringstream &strm);

  /*! Obstacles parameters. */
  ObstaclePar m_ObstacleParameters;
  /*! x, y, z position of obstacle in worldframe
    (point taken on the front plan of the obstacle
    on the floor and in the middel of the width
  */
  Eigen::Vector3d m_ObstaclePosition;

  /*! This is the rotationmatrix from
    obstacle frame to world frame */
  Eigen::Matrix3d m_ObstacleRot;

  // this is the rotationmatrix from world frame to obstacle frame
  Eigen::Matrix3d m_ObstacleRotInv;

  double m_StepOverStepLenght, m_StepOverHipHeight;

  deque<RelativeFootPosition> m_FootHolds;

  Eigen::MatrixXd mBoundCond;
  std::vector<double> mTimeDistr;

  StepOverPolynomeHip4 *m_PolynomeStepOverHipStep2;
  StepOverPolynomeHip4 *m_PolynomeStepOverHipRotation;

  StepOverClampedCubicSpline *m_ClampedCubicSplineStepOverFootX;
  StepOverClampedCubicSpline *m_ClampedCubicSplineStepOverFootY;
  StepOverClampedCubicSpline *m_ClampedCubicSplineStepOverFootZ;
  StepOverClampedCubicSpline *m_ClampedCubicSplineStepOverFootOmega;
  StepOverClampedCubicSpline *m_ClampedCubicSplineStepOverFootOmegaImpact;

  /*! Extra COMState buffer calculated in ZMPMultibody class  */
  deque<COMState> m_ExtraCOMBuffer;

  /*! Extra foot buffers with the same lenght as extra COM buffer
    and representing the two stpes over the obstacle */
  deque<FootAbsolutePosition> m_ExtraRightFootBuffer, m_ExtraLeftFootBuffer;

  /*! Buffers for first preview */
  deque<COMState> m_COMBuffer;
  deque<ZMPPosition> m_ZMPBuffer;

  /*! Buffer of complete foot course to be changed  */
  deque<FootAbsolutePosition> m_RightFootBuffer, m_LeftFootBuffer;

  /*! Buffer of complete ZMP course to be changed */
  deque<ZMPPosition> m_ZMPPositions;

  unsigned int m_StartStepOver;
  unsigned int m_StartDoubleSupp;
  unsigned int m_StartSecondStep;
  unsigned int m_EndStepOver;
  bool m_WhileSpecialSteps;
  unsigned int m_StartPrevStepOver;
  unsigned int m_EndPrevStepOver;
  unsigned int m_StartAfterStepOver;
  unsigned int m_EndAfterStepOver;

  unsigned int m_ExtraBufferLength;

  double m_ModulationSupportCoefficient;
  double m_Tsingle, m_TsingleStepOver;
  double m_Tdble, m_TdbleStepOver;
  double m_TdbleStepOverBeforeAfter, m_TsingleStepOverBeforeAfter;

  double m_tipToAnkle;
  double m_heelToAnkle;
  double m_soleToAnkle;
  double m_heelDistAfter;
  double m_tipDistBefore;

  double m_nominalStepLenght;
  double m_nominalStepWidth;
  double m_NominalCOMStepHeight;
  double m_DeltaStepOverCOMHeightMax;

  double m_KneeAngleBound;

  double m_DiffBetweenComAndWaist;

  double m_WaistRotationStepOver;

  int m_WhoIsFirst;

  Eigen::MatrixXd m_LegLayoutPoint;

  /*! Vector from the Waist to the left and right hip. */
  Eigen::Vector3d m_StaticToTheLeftHip;
  Eigen::Vector3d m_StaticToTheRightHip;

  /*! Displacement between the hip and the foot. */
  Eigen::Vector3d m_Dt;

  /*! Pointer to the Preview Control object. */
  PreviewControl *m_PC;

  /*! Pointer to the ZMPDiscretization object. */
  ZMPDiscretization *m_ZMPDiscr;

  /*! Pointer to the collision detector model.*/
  CollisionDetector *m_CollDet;

  /*! Previous joint values.*/
  Eigen::MatrixXd m_prev_ql;
  Eigen::MatrixXd m_prev_qr;

  /*! Sampling Period.*/
  double m_SamplingPeriod;

  /*! Preview control time.*/
  double m_PreviewControlTime;

  /*! Size of the preview control window.*/
  unsigned int m_NL;

  /*! Final state of the leg joints.*/
  Eigen::MatrixXd Finalql;
  Eigen::MatrixXd Finalqr;

  /*! Fifo for the ZMP ref.*/
  deque<ZMPPosition> m_FIFOZMPRefPositions;

  /*! Fifo for the ZMP ref.*/
  deque<ZMPPosition> m_FIFODeltaZMPPositions;

  /*! Fifo for the COM reference.*/
  deque<COMState> m_FIFOCOMStates;

  /*! Fifo for the positionning of the left foot.*/
  deque<FootAbsolutePosition> m_FIFOLeftFootPosition;

  /*! Fifo for the positionning of the right foot.*/
  deque<FootAbsolutePosition> m_FIFORightFootPosition;

  /*! Error on preview control for the cart model.*/
  double m_sxzmp, m_syzmp;

  /*! Error on preview control for the delta zmp.*/
  double m_sxDeltazmp, m_syDeltazmp;

  /*! State of the Preview control.*/
  Eigen::MatrixXd m_PC1x;
  Eigen::MatrixXd m_PC1y;

  /*! State of the Second Preview control.*/
  Eigen::MatrixXd m_Deltax;
  Eigen::MatrixXd m_Deltay;

  /*! Starting a new step sequences.*/
  bool m_StartingNewSequence;

  /*! Keep the ZMP reference.*/
  deque<ZMPPosition> m_FIFOTmpZMPPosition;

  /*! time distribution at which the specific intermediate points
    for the stepping over splines are to be exerted*/
  std::vector<double> m_TimeDistrFactor;

  /*! Reference to the humanoid specificities.*/
  PinocchioRobot *m_PR;

  /*! Distance from the ankle to the soil.*/
  double m_AnkleSoilDistance;
};

} // namespace PatternGeneratorJRL

#include <MotionGeneration/CollisionDetector.hh>
#include <ZMPRefTrajectoryGeneration/ZMPDiscretization.hh>

#endif /* _STEPOVER_PLANNER_H_ */
