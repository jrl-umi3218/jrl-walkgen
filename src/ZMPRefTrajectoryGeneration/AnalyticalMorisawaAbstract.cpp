/* This object generate the reference value for the
   ZMP based on a polynomail representation
   of the ZMP following 
   "Experimentation of Humanoid Walking Allowing Immediate
   Modification of Foot Place Based on Analytical Solution"
   Morisawa, Harada, Kajita, Nakaoka, Fujiwara, Kanehiro, Hirukawa, 
   ICRA 2007, 3989--39994


   Copyright (c) 2007, 
   Olivier Stasse,
   
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
#include <walkGenJrl/ZMPRefTrajectoryGeneration/AnalyticalMorisawaAbstract.h>

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
    
  }
}
