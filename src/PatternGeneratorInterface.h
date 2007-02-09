/** @doc  This object provides a unified interface to access the pattern generator.
            It allows to hide all the computation and hacking to the user.
 
    SVN Information:
            $Id$
            $Author$
            $Date$
            $Revision $
            $Source $
            $Log $
 
 
            Copyright (c) 2005-2006, 
            @author Olivier Stasse
   
            JRL-Japan, CNRS/AIST
 
            All rights reserved.
   
            Redistribution and use in source and binary forms, with or without modification, 
   are permitted provided that the following conditions are met:
   
 * Redistributions of source code must retain the above copyright notice, 
            this list of conditions and the following disclaimer.
 * Redistributions in binary form must reproduce the above copyright notice, 
            this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.
 * Neither the name of the <ORGANIZATION> nor the names of its contributors 
            may be used to endorse or promote products derived from this software without specific prior written permission.
   
            THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS 
            OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY 
            AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER 
            OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, 
            OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS 
            OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) 
            HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, 
            STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING 
            IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */


#ifndef _PATTERN_GENERATOR_INTERFACE_H_
#define _PATTERN_GENERATOR_INTERFACE_H_

#include <sstream>
#include <StepOverPlanner.h>
#include <ZMPPreviewControlWithMultiBodyZMP.h>
#include <ZMPDiscretization.h>
#include <PreviewControl.h>
#include <InverseKinematics.h>
#include <DynamicMultiBody.h>
#include <StepOverPlanner.h>
#include <WaistHeightVariation.h>
#include <UpperBodyMotion.h>
#include <GenerateMotionFromKineoWorks.h>
#include <StepStackHandler.h>
#include <HumanoidSpecificities.h>

namespace PatternGeneratorJRL
{
/** 
    \class
    \ingroup Interface
 */
class PatternGeneratorInterface
{
public:

        /**
                  \brief Constructor 
                  \param strm : Should provide the file to initialize the preview control,
                  the path to the VRML model, and the name of the file containing the VRML model.
         */
        PatternGeneratorInterface(std::istringstream &strm);

        /**
                  \brief Destructor 
         */
        ~PatternGeneratorInterface();

        /**
                  \brief Set the gain factor for the default behavior of the arm 
        \param strm : 
         */
        void m_SetArmParameters(std::istringstream &strm);

        /**
                  \brief Set obstacles parameters for stepping over 
        \param strm : 
         */
        void m_SetObstacleParameters(std::istringstream &strm);

        /**
                  \brief Set the shift of the ZMP height for stepping over. 
        \param strm : 
         */
        void m_SetZMPShiftParameters(std::istringstream &strm);

        /**
                  \brief Set time distribution parameters. 
        \param strm : 
         */
        void m_SetTimeDistrParameters(std::istringstream &strm);

        /**
                  \brief Set upper body motion parameters. 
        \param strm : 
         */
        void m_SetUpperBodyMotionParameters(std::istringstream &strm);

        /**
                  \brief Set the limits of the feasibility (stepping over parameters) 
        \param strm : 
         */
        void m_SetLimitsFeasibility(std::istringstream &strm);

        /**
        \brief Set the walking mode: 

                  0: Normal mode.

                  1: Waist height variation.

                  3: Kineo works mode.

                  4: Following the upper-body motion provided by another plugin. 

        \param strm : 
         */
        void m_WhichWalkMode(std::istringstream &strm);

        /**
                  \brief Read file from Kineoworks. 
        \param strm : 
         */
        void m_ReadFileFromKineoWorks(std::istringstream &strm);

        /**
                  \brief Specify a sequence of step without asking for immediate execution and end sequence. 
        \param strm : 
         */
        void m_PartialStepSequence(istringstream &strm);

        /**
                  \brief This method creates an arc on the step sequence stack. (Just an interface)
        \param strm : 
         */
        void m_CreateArcInStepStack(std::istringstream &strm);

        /**
                  \brief This method creates a centered arc on the step sequence stack.(Just an interface)
        \param strm : 
         */
        void m_CreateArcCenteredInStepStack(std::istringstream &strm);

        /**
                  \brief This method the constraints on the foot dimension for PBW's algorithm
        \param strm : 
         */
        void m_SetPBWConstraint(istringstream &strm);

        /**
                  \brief This method set PBW's algorithm for ZMP trajectory planning.
        \param strm : 
         */
        void m_SetAlgoForZMPTraj(istringstream &strm);

        /**
                  \brief This methods is doing the real job for the arc, but does not prepare for the support foot.
        \param x :
        \param y :
        \param R : 
        \param arc_deg :
        \param SupportFoot :
         */
        void CreateArcInStepStack(  double x,
                                    double y,
                                    double R,
                                    double arc_deg,
                                    int SupportFoot);

        /**
                  \brief This methods is doing the real job for the arc centered, but does not prepare for the support foot.
        \param R :
        \param arc_deg :
        \param SupportFoot :
         */
        void CreateArcCenteredInStepStack( double R,
                                           double arc_deg,
                                           int SupportFoot);

        /**
                  \brief This method is doing the real job for preparing the support foot.
        \param SupportFoot : 
         */
        void PrepareForSupportFoot(int SupportFoot);

        /**
                  \brief Interface to hrpsys to start the realization of  the stacked of step sequences.
        \param strm : 
         */
        void m_FinishAndRealizeStepSequence(std::istringstream &strm);

        /**
                  \brief The method which is really creating the buffer related to the stacked step sequences. 
         */
        void FinishAndRealizeStepSequence();

        /**
                  \brief Prepare to start or stop on a given support foot. (Just an interface)
        \param strm : 
         */
        void m_PrepareForSupportFoot(std::istringstream &strm);

        /**
                  \brief This method is finishing on the last support foot after the last motion.
        \param strm : 
         */
        void m_FinishOnTheLastCorrectSupportFoot(std::istringstream &strm);

        /**
                  \brief Realize a sequence of steps.
        \param strm : 
         */
        void m_StepSequence(std::istringstream &strm);

        /**
                  \brief Set the angle for lift off and landing of the foot.
        \param strm : 
         */
        void m_SetOmega(std::istringstream &strm);

        /**
                  \brief Set the maximal height of the foot trajectory.
        \param strm : 
         */
        void m_SetStepHeight(std::istringstream &strm);

        /**
                  \brief Set the single support time.
        \param strm : 
         */
        void m_SetSingleSupportTime(std::istringstream &strm);

        /**
                  \brief Set the double support time.
        \param strm : 
         */
        void m_SetDoubleSupportTime(std::istringstream &strm);

        /**
                  \brief Set a general parse command (to be used out of a plugin).
        \param strm : 
         */
        int ParseCmd(std::istringstream &strm);

        /**
                  \brief Run One Step of the global control loop aka The Main Method To Be Used.
                  \param : qr, ql : the joint values for the legs (right : qr, and left : ql). 
                  \param : qArmr, qArml : the joint values for the arms ( right: qArmr, and left : qArml)
                  \param : UpperBodyAngles : the angles corresponding to the upper part of the body.
                  \param : ZMPTarget : The target ZMP in the waist reference frame.
                  \param : outWaistPosition: The evaluated waist position.
                  \return: True is there is still some data to send, false otherwise.
         */
        bool RunOneStepOfTheControlLoop(MAL_MATRIX( &qr,double),
                                        MAL_MATRIX( &ql,double),
                                        MAL_MATRIX( &UpperBodyAngles,double),
                                        MAL_MATRIX( &ZMPTarget,double),
                                        COMPosition & outWaistPosition);

        /**
                  \brief Debug control loop.
        \param qr :
        \param ql :
        \param UpperBodyAngles :
        \param outWaistPosition :
        \param localindex :
         */
        void DebugControlLoop(MAL_MATRIX( &qr,double),
                              MAL_MATRIX( &ql,double),
                              MAL_MATRIX( &UpperBodyAngles,double),
                              COMPosition outWaistPosition,
                              int localindex);

        /**
                  \brief  Set the current joint values of the robot.

        note :
                  This method is used to properly initialize the pattern generator.
                  It also updates the state of the robot if other control mechanisms 
                  modifies the upper body part and if this should be taken into account
                  into the pattern generator in the second loop of control. 
                  \param CurrentJointValues
         */
        void SetCurrentJointValues(MAL_MATRIX( &lCurrentJointValues,double));

        /**
                  \brief Returns the walking mode. 
         */
        int GetWalkMode();

        /**
                  \brief  Get the leg joint velocity 
        \param dqr :
        \param dql :
         */
        void GetLegJointVelocity(MAL_MATRIX( &dqr,double), MAL_MATRIX( &dql,double));

        /**
                  \brief Computer Upper body heuristic for normal walking . 
        \param qArmr :
        \param qArml :
        \param aCOMPosition :
        \param RFP :
        \param LFP :
         */
        void ComputeUpperBodyHeuristicForNormalWalking(    MAL_MATRIX( &qArmr, double),
                        MAL_MATRIX( &qArml,double),
                        COMPosition aCOMPosition,
                        FootAbsolutePosition RFP,
                        FootAbsolutePosition LFP );

        /**
                  \brief  Start the creation of steps on line. 
         */
        void StartOnLineStepSequencing();

        /**
                  \brief  Start the creation of steps on line (istringstream interface). 
        \param strm : 
         */
        void m_StartOnLineStepSequencing(istringstream & strm);

        /**
                  \brief  Stop the creation of steps on line. 
         */
        void StopOnLineStepSequencing();

        /**
                  \brief  Stop the creation of steps on line (istringstream interface). 
        \param strm : 
         */
        void m_StopOnLineStepSequencing(istringstream &strm);

        /**
                  \brief  Common Initialization of walking.
                  \param lStartingCOMPosition: For the starting position on the articular space, returns the COM position.
                  \param BodyAnglesIni: Takes the initialization values of the robot.
                  \param InitLeftFootAbsPos: Returns the current absolute position of the left foot for the given posture of the robot.
                  \param InitRightFootAbsPos: Returns the current absolute position of the right foot for the given posture of the robot.
                  \param ClearStepStackHandler: Clean the stack of steps after copy.
         */
        void CommonInitializationOfWalking(MAL_S3_VECTOR(  & lStartingCOMPosition,double),
                                           MAL_MATRIX(  & BodyAnglesIni,double),
                                           FootAbsolutePosition & InitLeftFootAbsPos,
                                           FootAbsolutePosition & InitRightFootAbsPos,
                                           deque<RelativeFootPosition> & lRelativeFootPositions,
                                           vector<double> & lCurrentJointValues,
                                           bool ClearStepStackHandler);

        /**
                  \brief  Expansion of the buffers handling Center of Masse positions, as well as Upper Body Positions. 
        \param aNumber : 
         */
        void ExpandCOMAndUpperBodyPositionsQueues(int aNumber);

        /**
                  \brief  Compute the COM, left and right foot position for a given BodyAngle position 
        \param BodyAnglesIni :
        \param IStartingCOMPosition : 
        \param InitLeftFootAbsPos :
        \param InitRightFootAbsPos :
         */
        void EvaluateStartingCOM(MAL_MATRIX(  & BodyAnglesIni,double),
                                 MAL_S3_VECTOR(  & lStartingCOMPosition,double),
                                 FootAbsolutePosition & InitLeftFootAbsPos,
                                 FootAbsolutePosition & InitRightFootAbsPos);

        /**
        \brief  Compute the COM, left and right foot position for a given BodyAngle position 
        \param BodyAnglesIni :
        \param IStartingCOMPosition : 
        \param InitLeftFootAbsPos :
        \param InitRightFootAbsPos :
         */
        void EvaluateStartingCOM(MAL_MATRIX(  & BodyAnglesIni,double),
                                 MAL_S3_VECTOR(  & lStartingCOMPosition,double),
                                 MAL_S3_VECTOR(  & lStartingWaistPosition,double),
                                 FootAbsolutePosition & InitLeftFootAbsPos,
                                 FootAbsolutePosition & InitRightFootAbsPos);

        /**
        \brief  Add an online step 
        \param X :
        \param Y : 
        \param Theta :
         */
        void AddOnLineStep(double X, double Y, double Theta);

        /**
                  \name SLAM
                  @{
         */

        /**
        \brief  Update the current waist absolute position 
        \param UpdateAbsMotionOrNot :
         */
        void UpdateAbsolutePosition(bool UpdateAbsMotionOrNot);

        /**
        \brief  Get the waist position and orientation as a quaternion, and the planar X-Y orientation in Orientation. 
        \param TQ : TQ[0-2] the position of the Waist and [3-6] the Orientation of the waist (quarternion)
        \param Orientation :
         */
        void getWaistPositionAndOrientation(double TQ[7],double &Orientation);

        /**
         \brief  Set Waist position and Orientation 
        \param TQ : TQ[0-2] the position of the Waist and [3-6] the Orientation of the waist (quarternion)
         */
        void setWaistPositionAndOrientation(double TQ[7]);

        /**
        \brief Get Waist velocity 
        \param dx : velocity in x direction
        \param dy : velocity in y direction
        \param omega : rotation velocity 
         */
        void getWaistVelocity(double &dx,
                              double &dy,
                              double &omega) ;

        /**
        \brief  An other method to get the waist position using  matrix. 
        \param lWaistAbsPos :
         */
        void getWaistPositionMatrix(MAL_MATRIX( &lWaistAbsPos,double));

        /**
                  @}
         */

protected:

        ///  Planner for stepping over an obstacle.
        StepOverPlanner *m_StOvPl;

        ///  Planner for the waist variation for steppingover an obstacle. 
        WaistHeightVariation *m_WaistPlanner;

        ///  Object to handle the stack of relative steps.
        StepStackHandler *m_StepStackHandler;

        ///  Buffer needed to perform the stepping over  obstacle. 
        vector<double> m_ZMPShift;

        ///  Position and parameters related to the obstacle.
        ObstaclePar m_ObstaclePars;

        ///  Boolean on the obstacle's detection
        bool m_ObstacleDetected;

        ///  Variable for delta feasibility limit
        double m_DeltaFeasibilityLimit;

        ///  Object to store the upper body motion.
        UpperBodyMotion *m_UpBody;

        ///  Gain factor for the default arm motion while walking.
        double m_GainFactor;

        ///  Inverse Kinematics of the robot.
        InverseKinematics *m_IK;

        ///  Object to generate a ZMP profile from the step of stacks. It provides a buffer for the ZMP position to be used every dt in the control loop.
        ZMPDiscretization *m_ZMPD;

        ///  The Preview Control object.
        PreviewControl *m_PC;

        ///  The multibody objects.
        DynamicMultiBody *m_DMB,*m_2DMB;

        ///  The object to be used to perform one step of control, and generates the corrected CoM trajectory.
        ZMPPreviewControlWithMultiBodyZMP *m_ZMPpcwmbz;

        ///  Vector for the COM position.
        deque<COMPosition> m_COMBuffer;

        ///  Object needed to perform a path provided by Kineo
        GenerateMotionFromKineoWorks *m_GMFKW;

        ///  Buffer of upper body position related to a plan
        deque<PatternGeneratorJRL::KWNode > m_UpperBodyPositionsBuffer;

        ///  Conversion between the index of the plan and the robot DOFs.
        vector<int> m_ConversionForUpperBodyFromLocalIndexToRobotDOFs;

        ///  Current Joint values of the robot.
        vector<double> m_CurrentJointValues;

        /// Position of the waist Relative:
        MAL_MATRIX( m_WaistRelativePos,double);

        /// Position of the waist Absolute:
        MAL_MATRIX( m_WaistAbsPos,double);

        /// Orientation:
        double m_AbsTheta, m_AbsMotionTheta;

        /// Position of the motion:
        MAL_MATRIX(  m_MotionAbsPos,double);
        MAL_MATRIX(  m_MotionAbsOrientation,double);

        /// Absolute speed:
        MAL_MATRIX(  m_AbsLinearVelocity,double);
        MAL_MATRIX(  m_AbsAngularVelocity,double);

        /// Aboluste acceleration
        MAL_MATRIX(  m_AbsLinearAcc,double);

        /// Keeps track of the last correct support foot.
        int m_KeepLastCorrectSupportFoot;

        /// Boolean to ensure a correct initialization of the step's stack.
        bool m_IncorrectInitialization;

        ///  Buffer of ZMP positions
        deque<ZMPPosition> m_ZMPPositions;

        ///  Buffer of Absolute foot position (World frame)
        deque<FootAbsolutePosition> m_FootAbsolutePositions;


        ///  Buffer of absolute foot position.
        deque<FootAbsolutePosition> m_LeftFootPositions,m_RightFootPositions;

        ///  Buffer of absolute Hand position.
        deque<FootAbsolutePosition> m_LeftHandPositions,m_RightHandPositions;

        ///  Time Distribution factor
        vector<double> m_TimeDistrFactor;

        ///  Buffer of current Upper Body motion.
        vector<double> m_UpperBodyMotion;

        ///  Store the difference between the Center Of Mass and the Waist position.
        double m_DiffBetweenComAndWaist[3];

        ///  Store the debug mode.
        int m_DebugMode;

        ///  Store the number of degree of freedoms
        int m_DOF;

        ///  Store the height of the arm.
        double m_ZARM;

        ///  Store the leg's joint velocity.
        MAL_MATRIX( m_dql,double);
        MAL_MATRIX( m_dqr,double);

        /**! Local copy of Preview Control parameters. */
        ///  Sampling period of the control loop.
        double m_SamplingPeriod;

        ///  Window of the preview control
        double m_PreviewControlTime;

        ///  Height of the CoM.
        double m_Zc;

        ///  Store the local Single support time, and the Double support time.
        float m_TSsupport, m_TDsupport;

        ///  Discrete size of the preview control window
        unsigned int m_NL;

        ///  Local time while performing the control loop.
        unsigned long int m_count;


        ///  Maximal value for the arms in front of the robot
        double m_Xmax;

        ///  Variables used to compute speed for other purposes.
        MAL_MATRIX( m_prev_qr,double);
        MAL_MATRIX( m_prev_ql,double);
        MAL_MATRIX( m_prev_dqr, double);
        MAL_MATRIX( m_prev_dql, double);


        /// Debug variables.
        MAL_MATRIX( m_Debug_prev_qr, double);
        MAL_MATRIX(m_Debug_prev_ql,double);
        MAL_MATRIX( m_Debug_prev_dqr, double);
        MAL_MATRIX(m_Debug_prev_dql,double);
        MAL_MATRIX( m_Debug_prev_UpperBodyAngles,double);
        MAL_MATRIX( m_Debug_prev_qr_RefState, double);
        MAL_MATRIX(m_Debug_prev_ql_RefState,double);

        double m_Debug_prev_qWaistYaw, m_Debug_prev_dqWaistYaw;
        MAL_S3_VECTOR(,double) m_Debug_prev_P, m_Debug_prev_L;
        bool m_FirstPrint, m_FirstRead;

        bool m_ShouldBeRunning;

        ///  To handle a new step.
        bool m_NewStep;
        double m_NewStepX, m_NewStepY, m_NewTheta;

        /// Store the current relative state of the waist
        COMPosition m_CurrentWaistState;

        /// current time period for the control
        double m_dt;

        ///  Constraint on X and Y
        double m_ConstraintOnX, m_ConstraintOnY;

        ///  Sampling of the QP.
        double m_QP_T;

        ///  Preview window
        unsigned int m_QP_N;

        ///  Boolean to use PBW ZMP planner.
        unsigned char m_BoolPBWAlgo;

        ///  Humanoid specific data object handler.
        HumanoidSpecificities *m_HS;

};
};
#endif

