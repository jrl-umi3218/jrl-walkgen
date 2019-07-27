/*
 * Copyright 2010,
 *
 * Andrei Herdt
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

#include "portability/gettimeofday.hh"

#ifdef WIN32
# include <Windows.h>
#endif /* WIN32 */

#include <time.h>

#include <iostream>
#include <fstream>

#include <Mathematics/qld.hh>
#include <ZMPRefTrajectoryGeneration/ZMPVelocityReferencedQP.hh>

#include <Debug.hh>
using namespace std;
using namespace PatternGeneratorJRL;


void ZMPVelocityReferencedQP::debugConstructor()
{

  if (m_FastFormulationMode==QLDANDLQ)
    {
      RESETDEBUG6("/tmp/dtQLD.dat");
      RESETDEBUG6("/tmp/InfosQLD.dat");
      RESETDEBUG6("/tmp/Check2DLIPM_QLDANDLQ.dat");
    }


  if (m_FastFormulationMode==PLDP)
    {
      RESETDEBUG6("/tmp/dtPLDP.dat");
      RESETDEBUG6("/tmp/Check2DLIPM_PLDP.dat");
    }

  if(m_FullDebug>2)
    {
      ofstream aof;
      aof.open("/tmp/Trunk.dat",ofstream::out);
      aof.close();
      aof.open("/tmp/time.dat",ofstream::out);
      aof.close();
      aof.open("/tmp/FootPositionsT.dat",ofstream::out);
      aof.close();
    }
}

void ZMPVelocityReferencedQP::debugMatrix(const char * filename,
                                          enum MatrixType type)
{
  ofstream aof;
  aof.open(filename, ofstream::out);
  debugMatrix(aof,type);
  aof.close();
}

void ZMPVelocityReferencedQP::debugMatrix(ostream &aos,
                                          enum MatrixType type)
{
  MAL_MATRIX_TYPE(double) * aMalMatrix=0;
  int lnbcols=0,lnbrows=0;

  switch (type)
    {
    case M_PPU:
      lnbrows = lnbcols = 2*m_QP_N;
      aMalMatrix = &m_PPu;
      break;
    case M_PZU:
      lnbrows = m_QP_N;
      lnbcols = m_QP_N;
      aMalMatrix = & m_PZu;
      break;
    case M_VPU:
      lnbrows = lnbcols = 2*m_QP_N;
      aMalMatrix = & m_VPu;
      break;
    case M_PPX:
      lnbrows = 2*m_QP_N;
      lnbcols = 6;
      aMalMatrix = & m_PPx;
      break;
    case M_PZX:
      lnbrows = m_QP_N;
      lnbcols =3 ;
      aMalMatrix = & m_PZx;
      break;
    case M_VPX:
      lnbrows = 2*m_QP_N;
      lnbcols = 6;
      aMalMatrix = & m_VPx;
      break;
    case M_U:
      lnbrows = m_QP_N;
      lnbcols = m_PrwSupport.StepNumber;
      aMalMatrix = & m_U;
      break;
    case M_LQ:
      lnbrows = 2*m_QP_N;
      lnbcols = 2*m_QP_N;
      aMalMatrix = & m_LQ;
      break;
    case M_ILQ:
      lnbrows = 2*m_QP_N;
      lnbcols = 2*m_QP_N;
      aMalMatrix = & m_iLQ;
      break;
    }

  for(int i=0; i<lnbrows; i++)
    {
      for(int j=0; j<lnbcols; j++)
        aos << (*aMalMatrix)(i,j) << " ";
      aos << std::endl;
    }

}


