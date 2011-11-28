/*
 * Copyright 2010, 
 *
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
/* This object generate the reference value for the
   ZMP based on a polynomail representation
   of the ZMP following 
   "Experimentation of Humanoid Walking Allowing Immediate
   Modification of Foot Place Based on Analytical Solution"
   Morisawa, Harada, Kajita, Nakaoka, Fujiwara, Kanehiro, Hirukawa, 
   ICRA 2007, 3989--39994
*/

#define ODEBUG2(x)
#define ODEBUG3(x) cerr << "AnalyticalMorisawa :" << x << endl
#define RESETDEBUG5(y) { ofstream DebugFile; DebugFile.open(y,ofstream::out); DebugFile.close();}
#define ODEBUG5(x,y) { ofstream DebugFile; DebugFile.open(y,ofstream::app); DebugFile << "PGI: " << x << endl; DebugFile.close();}
#if 0
#define ODEBUG(x) cerr << "AnalyticalMorisawa :" <<  x << endl
#else
#define ODEBUG(x)
#endif

#if 0
#define RESETDEBUG4(y) { ofstream DebugFile; DebugFile.open(y,ofstream::out); DebugFile.close();}
#define ODEBUG4(x,y) { ofstream DebugFile; DebugFile.open(y,ofstream::app); \
    DebugFile << "PGI: " << x << endl; DebugFile.close();}
#define _DEBUG_4_ACTIVATED_ 1
#else
#define RESETDEBUG4(y)
#define ODEBUG4(x,y)
#endif

#define ODEBUG6(x,y)

#include <fstream>
#include <ZMPRefTrajectoryGeneration/AnalyticalMorisawaAbstract.hh>

typedef double doublereal;
typedef int integer;

namespace PatternGeneratorJRL
{


  AnalyticalMorisawaAbstract::AnalyticalMorisawaAbstract(SimplePluginManager *lSPM)
    : ZMPRefTrajectoryGeneration(lSPM)
  {
    m_NumberOfStepsInAdvance = 0;
    m_NumberOfIntervals = 0;
    m_DeltaTj.clear();
    m_Omegaj.clear();
    m_VerboseLevel=0;
  }


  AnalyticalMorisawaAbstract::~AnalyticalMorisawaAbstract()
  {

  }


  bool AnalyticalMorisawaAbstract::SetNumberOfStepsInAdvance(int lNumberOfStepsInAdvance)
  {
    m_NumberOfStepsInAdvance = lNumberOfStepsInAdvance;
    return true;
  }


  int AnalyticalMorisawaAbstract::GetNumberOfStepsInAdvance()
  {
    return m_NumberOfStepsInAdvance;
  }

  int AnalyticalMorisawaAbstract::GetNumberOfIntervals()
  {
    return m_NumberOfIntervals;
  }

  int AnalyticalMorisawaAbstract::GetOmegaj(vector<double> &GetOmegaj)
  {
    GetOmegaj = m_Omegaj;
    return 1;
  }

  int AnalyticalMorisawaAbstract::GetDeltaTj(vector<double> &GetTj)
  {
    GetTj = m_DeltaTj;
    return 1;
  }

  int AnalyticalMorisawaAbstract::SetDeltaTj(vector<double> &GetTj)
  {
    m_DeltaTj = GetTj;
    return 0;
  }

  bool AnalyticalMorisawaAbstract::GetPolynomialWeights(vector<double> &PolynomialWeights)
  {
    unsigned int r = MAL_VECTOR_SIZE(m_y);
    PolynomialWeights.resize(r);
    for(unsigned int i=0;i<r;++i)
      PolynomialWeights[i] = m_y[i];
    return true;
  }

  void AnalyticalMorisawaAbstract::GetPolynomialDegrees(vector<unsigned int> &lPolynomialDegrees)
  {
    lPolynomialDegrees = m_PolynomialDegrees;
  }

  std::ostream & operator<<(std::ostream &os, const AnalyticalMorisawaAbstract &obj)
  {
    os << "Matrix Z:" << endl;
    os << obj.m_Z << endl;

    os << "Vector W:" <<endl;
    os << obj.m_w << endl;

    os << "Vector Y:" << endl;
    os << obj.m_y << endl;

    os << "Time intervals:" << endl;
    for(unsigned int i=0;i<obj.m_DeltaTj.size();i++)
      os << obj.m_DeltaTj[i] << " ";
    os << endl;
    
    os << "Step Types:" << endl;
    for(unsigned int i=0;i<obj.m_StepTypes.size();i++)
      os << obj.m_StepTypes[i] << " ";
    os << endl;

    os << "Omegaj:" << endl;
    for(unsigned int i=0;i<obj.m_Omegaj.size();i++)
      os << obj.m_Omegaj[i] << " ";
    os << endl;
    return os;
  }
  
  void AnalyticalMorisawaAbstract::displayDeltaTj(ostream &aos)
  {
    aos << "AnalyticalMorisawaCompact:";
    for(unsigned int li=0;
	li < m_DeltaTj.size();
	li++)
      {
	aos << m_DeltaTj[li] ;
	if (li < m_DeltaTj.size()-1)
	  aos << " ";
      }
    aos << endl;
  }
  
}
