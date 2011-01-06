/*
 * Copyright 2010, 
 *
 * Mehdi      Benallegue
 * Andrei     Herdt
 * Olivier    Stasse
 * 
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
/* This object generate matrix representation of linear
   constraint based on foot position.
   It handles a stack of constraint on a sliding mode 
   for QP solving. */

#ifndef _FOOT_CONSTRAINTS_AS_LINEAR_SYSTEM_FOR_VEL_REF_H_
#define _FOOT_CONSTRAINTS_AS_LINEAR_SYSTEM_FOR_VEL_REF_H_

#include <vector>
#include <deque>
#include <string>
#include <sstream>

#include <jrl/mal/matrixabstractlayer.hh>

#include <abstract-robot-dynamics/humanoid-dynamic-robot.hh>


#include <jrl/walkgen/pgtypes.hh>
#include <privatepgtypes.h>
#include <Mathematics/ConvexHull.h>
#include <Mathematics/FootHalfSize.hh>
#include <SimplePlugin.h>
#include <PreviewControl/SupportFSM.h>

namespace PatternGeneratorJRL
{

  /// \brief Generate a stack of inequality constraints for the whole preview window.
  class FootConstraintsAsLinearSystemForVelRef:public SimplePlugin
  {

    //
    // Public member functions
    //
  public:

    /// \name Constructors and destructors.
    /// \{
    FootConstraintsAsLinearSystemForVelRef (SimplePluginManager * aSPM,
                                            CjrlHumanoidDynamicRobot * aHS);
    ~FootConstraintsAsLinearSystemForVelRef ();
    /// \}

    /// \brief Build a queue of inequalities
    ///
    /// \param LeftFootAbsolutePositions used to get the current position of the left foot
    /// \param RightFootAbsolutePositions used to get the current position of the right foot
    /// \param ZMPInequalitiesDeque constraints on the ZMP relative to the foot centers
    /// \param FeetPosInequalitiesDeque constraints on the feet placements relative to previous foot placements
    /// \param RefVel the velocity reference
    /// \param CurrentTime
    /// \param QP_N number of instants in the preview horizon
    /// \param SupportFSM the finite state machine for the preview of support states
    /// \param CurrentSupport
    /// \param PrwSupport output of SupportFSM
    /// \param PreviewedSupportAngles deque of support orientations previewed previously
    /// \param NbOfConstraints total number of previewed constraints
    int buildConstraintInequalities (std::deque < FootAbsolutePosition >
                                     &LeftFootAbsolutePositions,
                                     std::deque < FootAbsolutePosition >
                                     &RightFootAbsolutePositions,
                                     std::deque <
                                     linear_inequality_ff_t >
                                     &ZMPInequalitiesDeque,
                                     std::deque <
                                     linear_inequality_ff_t >
                                     &FeetPosInequalitiesDeque,
                                     reference_t & RefVel,
                                     double CurrentTime, double QP_N,
                                     SupportFSM * SupportFSM,
                                     support_state_t & CurrentSupport,
                                     support_state_t & PrwSupport,
                                     std::deque <
                                     double >&PreviewedSupportAngles,
                                     int &NbOfConstraints);

    /// \brief Adapt vertices to the support foot and its orientation
    ///
    /// \param ZMPConstrVertices vertices of the ZMP convex hull
    /// \param FeetPosConstrVertices vertices of the foot position convex hull
    /// \param ZMPConvHullOrientation
    /// \param FeetPosConvHullOrientation
    /// \param PrwSupport previewed support state
    /// \return 0
    int setVertices (std::vector < CH_Point > & ZMPConstrVertices,
                     std::vector < CH_Point > & FeetPosConstrVertices,
                     double & ZMPConvHullOrientation,
                     double & FeetPosConvHullOrientation,
                     support_state_t & PrwSupport);

    /// \brief Compute the linear inequalities \f${\bf A}{\bf x} \geq {\bf b}\f$ associated with the
    /// convex hull specified by a vector of points.
    ///
    /// \param aVecOfPoints a vector of vertices
    /// \param D left hand side of the inequalities
    /// \param Dc right hand side of the inequalities
    /// \param PrwSupport previewed support state
    /// \return 0
    int computeLinearSystem (std::vector < CH_Point > & aVecOfPoints,
                             MAL_MATRIX (&D, double),
                             MAL_MATRIX (&Dc, double),
                             support_state_t & PrwSupport);

    /// \brief Reimplement the interface of SimplePluginManager
    ///
    /// \param[in] Method: The method to be called.
    /// \param[in] Args: Arguments of the methods.
    virtual void CallMethod (std::string & Method, std::istringstream & Args);

    //
    // Private member functions
    //
  private:

    /// \brief Initialize
    void initFPConstrArrays ();

    /// \brief Initialize the convex hulls for the constraints
    ///
    /// \return 0
    int initConvexHulls ();

    /// \brief Define the dimensions of the feet
    ///
    /// \param aHS object of the robot
    /// \return 0
    int setFeetDimensions (CjrlHumanoidDynamicRobot * aHS);

    /// \brief Initialize the constraint hulls
    ///
    /// return 0
    int initFeetConstraints ();

    //
    // Private members
    //
  private:
    /// \brief Reference to the Humanoid Specificities.
      CjrlHumanoidDynamicRobot * m_HS;

    /// \brief convex hull
      std::vector < CH_Point > m_ConvexHullVertices;


    /// \brief Vertices defining the constraints on the feet positions
    double *m_FPosConstrVerticesX;
    double *m_FPosConstrVerticesY;
    double m_LeftFPosConstrVerticesX[5];
    double m_RightFPosConstrVerticesX[5];
    double m_LeftFPosConstrVerticesY[5];
    double m_RightFPosConstrVerticesY[5];

    /// \brief Vertices defining the constraints on the zmp positions
    double *m_ZMPConstrVerticesX;
    double *m_ZMPConstrVerticesY;
    double m_LeftZMPConstrVerticesX[4];
    double m_RightZMPConstrVerticesX[4];
    double m_LeftZMPConstrVerticesY[4];
    double m_RightZMPConstrVerticesY[4];
    double m_LeftDSZMPConstrVerticesX[4];
    double m_RightDSZMPConstrVerticesX[4];
    double m_LeftDSZMPConstrVerticesY[4];
    double m_RightDSZMPConstrVerticesY[4];

    /// \brief Some coefficients
    ///
    /// For symmetrical constraints: The points of the left foot are counted clockwise.
    double *m_lxcoefsRight;
    double *m_lycoefsRight;
    double *m_lxcoefsLeft;
    double *m_lycoefsLeft;

    /// \brief Half foot size
    FootHalfSize m_LeftFootSize, m_RightFootSize;

    /// \brief Security margins (default 40 cm)
    double m_SecurityMarginX;
    double m_SecurityMarginY;

    /// \brief Distance between the feet in the double support phase
    double m_DSFeetDistance;

    /// \brief Foot specificities.
    CjrlFoot *m_RightFoot;
    CjrlFoot *m_LeftFoot;

    unsigned int m_FullDebug;

  };
}
#endif                          /* _FOOT_CONSTRAINTS_AS_LINEAR_SYSTEM_FOR_VEL_REF_H_ */
