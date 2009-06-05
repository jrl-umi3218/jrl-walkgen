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

   For more information on the license please look at License.txt 
   in the root directory.

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
}
