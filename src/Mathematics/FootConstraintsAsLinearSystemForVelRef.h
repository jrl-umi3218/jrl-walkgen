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
    // Public types
    //
  public:
    const static int ZMP_CONSTRAINTS = 0;
    const static int FOOT_CONSTRAINTS = 1;

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


    /// \brief Adapt vertices to the support foot and its orientation
    ///
    /// \param[out] ConvexHull vertices of the ZMP convex hull
    /// \param[in] Orientation
    /// \param[in] SupportState
    /// \param[in] constraints_type
    /// \return 0
    int setVertices( convex_hull_t & ConvexHull,
		     double Orientation,
		     const support_state_t & SupportState,
		     int constraints_type);

    /// \brief Compute the linear inequalities \f${\bf A}{\bf x} \geq {\bf b}\f$ associated with the
    /// convex hull specified by a vector of points.
    ///
    /// \param[in] aVecOfPoints a vector of vertices
    /// \param[out] D left hand side of the inequalities
    /// \param[out] Dc right hand side of the inequalities
    /// \param[in] PrwSupport previewed support state
    /// \return 0
    int computeLinearSystem (const convex_hull_t & ConvexHull,
                             MAL_MATRIX (&D, double),
                             MAL_MATRIX (&Dc, double),
                             const support_state_t & PrwSupport) const;

    /// \brief Compute the linear inequalities \f${\bf A}{\bf x} \geq {\bf b}\f$ associated with the
    /// convex hull specified by a vector of points.
    ///
    /// \param[in] aVecOfPoints a vector of vertices
    /// \param[out] D_x left hand side of the constraints
    /// \param[out] D_y left hand side of the constraints
    /// \param[out] Dc right hand side of the constraints
    /// \param[in] PrwSupport previewed support state
    /// \return 0
    int computeLinearSystem (const convex_hull_t & ConvexHull,
			     double * D_x, double * D_y, double * d,
			     const support_state_t & PrwSupport) const;

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
    int setFeetDimensions ( CjrlHumanoidDynamicRobot * aHS );

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

    /// \brief Vertices defining the constraints on the feet positions
    double *m_FPosConstrVerticesX;
    double *m_FPosConstrVerticesY;

    struct edges_s
    {
      convex_hull_t
      leftSS,
        rightSS,
        rightDS,
        leftDS;
    };

    struct edges_s m_FootPosEdges, m_ZMPPosEdges;



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

    CjrlFoot * m_RightFoot, * m_LeftFoot;

  };
}
#endif                          /* _FOOT_CONSTRAINTS_AS_LINEAR_SYSTEM_FOR_VEL_REF_H_ */
