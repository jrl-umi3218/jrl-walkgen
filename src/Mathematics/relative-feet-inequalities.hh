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

#ifndef _RELATIVE_FEET_INEQUALITIES_
#define _RELATIVE_FEET_INEQUALITIES_

#include <jrl/walkgen/pinocchiorobot.hh>


#include <jrl/walkgen/pgtypes.hh>
#include <privatepgtypes.hh>
#include <Mathematics/ConvexHull.hh>
#include <Mathematics/FootHalfSize.hh>
#include <SimplePlugin.hh>
#include <PreviewControl/SupportFSM.hh>

namespace PatternGeneratorJRL
{

  /// \brief Generate a stack of inequalities relative to feet centers for
  /// the whole preview window.
  class RelativeFeetInequalities:public SimplePlugin
  {

    //
    // Public types
    //
  public:

    //
    // Public member functions
    //
  public:

    /// \name Constructors and destructors.
    /// \{
    RelativeFeetInequalities (SimplePluginManager * aSPM,
                              PinocchioRobot * aPR);
    ~RelativeFeetInequalities ();
    /// \}

    /// \brief Adapt vertices to the support foot and its orientation
    ///
    /// \param[out] ConvexHull Vertices of the convex hull
    /// \param[in] SupportState Corresponding support state
    /// \param[in] Type CoP/Feet
    void set_vertices( convex_hull_t & ConvexHull,
                       const support_state_t & SupportState,
                       ineq_e type);

    /// \brief Adapt inequalities to the support foot and its orientation
    ///
    /// \param[out] ConvexHull
    /// \param[in] SupportState
    /// \param[in] Type CoP/Feet/CoM
    void set_inequalities( convex_hull_t & ConvexHull,
                           const support_state_t & Support, ineq_e type);

    /// \brief Compute the linear inequalities \f${\bf A}{\bf x} \geq
    /// {\bf b}\f$ associated with the
    /// convex hull specified by a vector of points.
    ///
    /// \param[out] aVecOfPoints a vector of vertices
    /// \param[in] PrwSupport previewed support state
    void compute_linear_system ( convex_hull_t & ConvexHull,
                                 const support_state_t & PrwSupport) const;

    /// \brief Reimplement the interface of SimplePluginManager
    ///
    /// \param[in] Method: The method to be called.
    /// \param[in] Args: Arguments of the methods.
    virtual void CallMethod (std::string & Method, std::istringstream & Args);


    /// \brief Reimplement the interface of SimplePluginManager
    ///
    /// \param[in] Method: The method to be called.
    /// \param[in] Args: Arguments of the methods.
    void getFeetSize(FootHalfSize & leftFootSize, FootHalfSize & rightFootSize);

    inline double DSFeetDistance()
    {
      return DSFeetDistance_;
    }

    //
    // Private member functions
    //
  private:

    /// \brief Initialize the convex hulls for the constraints
    ///
    /// \return 0
    int init_convex_hulls ();

    /// \brief Define the dimensions of the feet
    ///
    /// \param aHS object of the robot
    /// \return 0
    int set_feet_dimensions ( PinocchioRobot * aPR );

    /// \brief Initialize the constraint hulls
    ///
    /// return 0
    int init_feet_constraints ();

    //
    // Private members
    //
  private:

    struct edges_s
    {
      convex_hull_t
      LeftSS,
        RightSS,
        RightDS,
        LeftDS;
    };
    struct edges_s FootPosEdges_, ZMPPosEdges_;


    /// \brief Polyhedral hull
    convex_hull_t CoMHull_;

    double LeftFPosEdgesX_[5], LeftFPosEdgesY_[5];
    double RightFPosEdgesX_[5], RightFPosEdgesY_[5];

    /// \brief Half foot size
    FootHalfSize LeftFootSize_, RightFootSize_;

    /// \brief Security margins (default 40 cm)
    double SecurityMarginX_;
    double SecurityMarginY_;

    /// \brief Distance between the feet in the double support phase
    double DSFeetDistance_;

  };
}
#endif                          /* _RELATIVE_FEET_INEQUALITIES_ */
