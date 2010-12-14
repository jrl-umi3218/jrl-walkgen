/*
 * Copyright 2010,
 *
 * Andrei   Herdt
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
/*! This object provides intermediate elements and operations for the construction of a QP.
 */



#include <Mathematics/intermediate-qp-matrices.hh> //TODO: Merge with ZMPVelocityReferencedQPDebug.cpp


using namespace PatternGeneratorJRL;



IntermedQPMat::IntermedQPMat()
{

}


IntermedQPMat::~IntermedQPMat()
{
  //TODO:
}


void 
IntermedQPMat::getTermMatrices(standard_ls_form_t & TMat, int ObjectiveType)
{
  TMat.nSt = m_StateMatrices.NbStepsPrw;

  switch(ObjectiveType)
    {
    case MEAN_VELOCITY:
      TMat.U = m_MeanVelocity.U;
      TMat.UT = m_MeanVelocity.UT;
      TMat.Sc_x = MAL_RET_A_by_B(m_MeanVelocity.S,m_StateMatrices.CoM.x);
      TMat.Sc_x = MAL_RET_A_by_B(m_MeanVelocity.S,m_StateMatrices.CoM.y);
      TMat.weight = m_MeanVelocity.weight;
      TMat.ref_x = m_StateMatrices.RefX;
      TMat.ref_y = m_StateMatrices.RefY;
    case INSTANT_VELOCITY:
      TMat.U = m_InstantVelocity.U;
      TMat.UT = m_InstantVelocity.UT;
      TMat.Sc_x = MAL_RET_A_by_B(m_InstantVelocity.S,m_StateMatrices.CoM.x);
      TMat.Sc_y = MAL_RET_A_by_B(m_InstantVelocity.S,m_StateMatrices.CoM.y);
      TMat.weight = m_InstantVelocity.weight;
      TMat.ref_x = m_StateMatrices.RefX;
      TMat.ref_y = m_StateMatrices.RefY;
    case COP_CENTERING:
      TMat.U = m_COPCentering.U;
      TMat.UT = m_COPCentering.UT;
      TMat.V = m_StateMatrices.V;
      TMat.VT = m_StateMatrices.VT;
      TMat.Sc_x = MAL_RET_A_by_B(m_COPCentering.S,m_StateMatrices.CoM.x);
      TMat.Sc_x = MAL_RET_A_by_B(m_COPCentering.S,m_StateMatrices.CoM.y);
      TMat.weight = m_COPCentering.weight;
      TMat.ref_x = m_StateMatrices.Vc*m_StateMatrices.fx;
      TMat.ref_y = m_StateMatrices.Vc*m_StateMatrices.fy;
    case JERK:
      TMat.U = m_Jerk.U;
      TMat.UT = m_Jerk.UT;
      TMat.Sc_x = MAL_RET_A_by_B(m_Jerk.S,m_StateMatrices.CoM.x);
      TMat.Sc_x = MAL_RET_A_by_B(m_Jerk.S,m_StateMatrices.CoM.y);
      TMat.weight = m_Jerk.weight;
      TMat.ref_x = 0.0*m_StateMatrices.RefX;
      TMat.ref_y = 0.0*m_StateMatrices.RefY;
    }
}


com_t IntermedQPMat::operator ()() const
{
  return m_StateMatrices.CoM;
}
void IntermedQPMat::operator ()( com_t CoM )
{
  m_StateMatrices.CoM = CoM;
}
