/* This object generate matrix representation of linear
   constraint based on foot position.
   It handles a stack of constraint on a sliding mode 
   for QP solving.

   Copyright (c) 2009, 
   Olivier Stasse
   
   JRL-Japan, CNRS/AIST

   All rights reserved.
   
   Redistribution and use in source and binary forms, with or without modification, 
   are permitted provided that the following conditions are met:
   
   * Redistributions of source code must retain the above copyright notice, 
   this list of conditions and the following disclaimer.
   * Redistributions in binary form must reproduce the above copyright notice, 
   this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.
   * Neither the name of the CNRS/AIST nor the names of its contributors 
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

#ifndef _FOOT_CONSTRAINTS_AS_LINEAR_SYSTEM_H_
#define _FOOT_CONSTRAINTS_AS_LINEAR_SYSTEM_H_

#include <vector>
#include <deque>
#include <string>
#include <sstream>

#include <MatrixAbstractLayer/MatrixAbstractLayer.h>

#include <dynamicsJRLJapan/HumanoidSpecificities.h>

#include <walkGenJrl/walkGenJrl_API.h>
#include <walkGenJrl/PGTypes.h>
#include <walkGenJrl/Mathematics/ConvexHull.h>
#include <walkGenJrl/SimplePlugin.h>

namespace PatternGeneratorJRL
{
  /*! This class generates matrix representation of linear
   constraint based on foot position.
   It handles a stack of constraint on a sliding mode 
   for QP solving.
   */
  class WALK_GEN_JRL_EXPORT FootConstraintsAsLinearSystem: public SimplePlugin
    {
    public:

      /*! Constructor */
      FootConstraintsAsLinearSystem(SimplePluginManager *aSPM, 
				    dynamicsJRLJapan::HumanoidSpecificities *aHS);

      /*! Destructor */
      ~FootConstraintsAsLinearSystem();

      /*! Compute the linear system \f${\bf A}{\bf x} \geq {\bf b}\f$ associated with the 
	set of points specified by aVecOfPoints. aVecOfPoints is supposed to represent
	the convex hull of the robot contact points with the ground.
       */
      int ComputeLinearSystem(std::vector<CH_Point> aVecOfPoints,
			      MAL_MATRIX(&A,double),
			      MAL_MATRIX(&B,double));

      /*!  Build a queue of constraint Inequalities based on a list of Foot Absolute
	Position.
       */
      int BuildLinearConstraintInequalities(std::deque< FootAbsolutePosition> &LeftFootAbsolutePositions,
					    std::deque<FootAbsolutePosition> &RightFootAbsolutePositions,
					    std::deque<LinearConstraintInequality_t *> &
					    QueueOfLConstraintInequalities,
					    double ConstraintOnX,
					    double ConstraintOnY);

      /*!  Build a queue of constraint Inequalities based on a list of Foot Absolute Position.  */
      int BuildLinearConstraintInequalities2(std::deque< FootAbsolutePosition> &LeftFootAbsolutePositions,
					     std::deque<FootAbsolutePosition> &RightFootAbsolutePositions,
					     std::deque<LinearConstraintInequality_t *> &
					     QueueOfLConstraintInequalities,
					     double ConstraintOnX,
					     double ConstraintOnY);

      /*! Reimplement the interface of SimplePluginManager 
	\param[in] Method: The method to be called.
	\param[in] Args: Arguments of the methods.
       */
      virtual void CallMethod(std::string & Method, std::istringstream &Args);

    private:

      /* ! Reference on the Humanoid Specificities. */
      dynamicsJRLJapan::HumanoidSpecificities * m_HS;
      
    };
};
#endif /* _FOOT_CONSTRAINTS_AS_LINEAR_SYSTEM_H_ */
