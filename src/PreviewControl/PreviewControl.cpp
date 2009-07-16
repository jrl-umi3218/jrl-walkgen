/** Object to perform preview control on a cart model.

   Copyright (c) 2005-2006, 
   @author Olivier Stasse, Ramzi Sellouati
   
   JRL-Japan, CNRS/AIST

   All rights reserved.

   For more information on the license please look at License.txt 
   in the root directory.
   
*/

#include <fstream>
#include <Debug.h>

#include <PreviewControl/PreviewControl.h>

using namespace::PatternGeneratorJRL;

PreviewControl::PreviewControl(SimplePluginManager *lSPM,
			       unsigned int defaultMode,
			       bool lAutoComputeWeights)
  : SimplePlugin(lSPM)
{

  m_AutoComputeWeights = lAutoComputeWeights;
  m_DefaultWeightComputationMode = defaultMode;

  m_SamplingPeriod = 0.0;
  m_PreviewControlTime = 0.0;
  m_Zc = 0.0;
  m_SizeOfPreviewWindow = 0;
  
  MAL_MATRIX_RESIZE(m_A,3,3);
  MAL_MATRIX_RESIZE(m_B,3,1);
  MAL_MATRIX_RESIZE(m_C,1,3);

  MAL_MATRIX_RESIZE(m_Kx,1,3);
  m_Ks = 0;


  ODEBUG("Identification: " << this);
  std::string aMethodName[3] = 
    {":samplingperiod",
     ":previewcontroltime",
     ":comheight"};
  
  for(int i=0;i<3;i++)
    {
      if (!RegisterMethod(aMethodName[i]))
	{
	  std::cerr << "Unable to register " << aMethodName << std::endl;
	}
      else
	{
	  ODEBUG("Succeed in registering " << aMethodName[i]);
	}

    }

}

PreviewControl::~PreviewControl()
{

}

double PreviewControl::SamplingPeriod() const
{
  return m_SamplingPeriod; 
}

double PreviewControl::PreviewControlTime() const
{ 
  return m_PreviewControlTime; 
}


double PreviewControl::GetHeightOfCoM() const
{
  return m_Zc;
}

void PreviewControl::SetSamplingPeriod(double lSamplingPeriod)
{
  if (m_SamplingPeriod != lSamplingPeriod)
    m_Coherent = false;

  m_SamplingPeriod = lSamplingPeriod;

  if (m_AutoComputeWeights)
    ComputeOptimalWeights(m_DefaultWeightComputationMode);
}

void PreviewControl::SetPreviewControlTime(double lPreviewControlTime)
{
  if (m_PreviewControlTime != lPreviewControlTime)
    m_Coherent = false;
    
  m_PreviewControlTime = lPreviewControlTime;

  if (m_AutoComputeWeights)
    ComputeOptimalWeights(m_DefaultWeightComputationMode);

}

void PreviewControl::SetHeightOfCoM(double lHeightOfCom)
{
  if (m_Zc!=lHeightOfCom)
    m_Coherent = false;

  m_Zc = lHeightOfCom;

  if (m_AutoComputeWeights)
    ComputeOptimalWeights(m_DefaultWeightComputationMode);

}

bool PreviewControl::IsCoherent()
{
  return m_Coherent;
}

void PreviewControl::ReadPrecomputedFile(string aFileName)
{
  std::ifstream aif;

  aif.open(aFileName.c_str(),std::ifstream::in);
  if (aif.is_open())
    {
      aif >> m_Zc;
      aif >> m_SamplingPeriod;
      aif >> m_PreviewControlTime;
      
      
      float r;
      for(int i=0;i<3;i++)
	{
	  aif >> r;
	  m_Kx(0,i)=r ;
	}

      aif >> r;
      m_Ks=r;

      
      m_SizeOfPreviewWindow = (unsigned int)(m_PreviewControlTime/
					     m_SamplingPeriod);
      MAL_MATRIX_RESIZE(m_F,m_SizeOfPreviewWindow,1);

      for(unsigned int i=0;i<m_SizeOfPreviewWindow;i++)
	{
	  aif >> r;
	  m_F(i,0)=r;
	}
      //      cout << (*m_F) << endl;
      double T = m_SamplingPeriod;
      m_A(0,0) = 1.0; m_A(0,1) =   T; m_A(0,2) = T*T/2.0;
      m_A(1,0) = 0.0; m_A(1,1) = 1.0; m_A(1,2) = T;
      m_A(2,0) = 0.0; m_A(2,1) = 0.0; m_A(2,2) = 1.0;

      m_B(0,0) = T*T*T/6.0;
      m_B(1,0) = T*T/2.0;
      m_B(2,0) = T;

      m_C(0,0) = 1.0;
      m_C(0,1) = 0.0;
      m_C(0,2) = -m_Zc/9.81;

      
      m_Coherent = true;

      aif.close();
    }
  else 
    cerr << "PreviewControl - Unable to open " << aFileName << endl;
    
}

void PreviewControl::ComputeOptimalWeights(unsigned int mode)
{
  /*! \brief Solver to compute optimal weights */
  OptimalControllerSolver *anOCS;

  double T = m_SamplingPeriod;
  m_A(0,0) = 1.0; m_A(0,1) =   T; m_A(0,2) = T*T/2.0;
  m_A(1,0) = 0.0; m_A(1,1) = 1.0; m_A(1,2) = T;
  m_A(2,0) = 0.0; m_A(2,1) = 0.0; m_A(2,2) = 1.0;
  
  m_B(0,0) = T*T*T/6.0;
  m_B(1,0) = T*T/2.0;
  m_B(2,0) = T;
  
  m_C(0,0) = 1.0;
  m_C(0,1) = 0.0;
  m_C(0,2) = -m_Zc/9.81;
  ODEBUG(" m_Zc: " << m_Zc << " " << m_C(0,2));
  
  MAL_MATRIX(, double) lF,lK;

  double Q,R;
  Q = 1.0;
  R = 1e-5;
  int Nl;
  Nl = (int)(m_PreviewControlTime/T);

  if (mode==OptimalControllerSolver::MODE_WITHOUT_INITIALPOS)
    {

      // Build the derivated system
      MAL_MATRIX_DIM(Ax,double,4,4);
      MAL_MATRIX(tmpA,double);
      MAL_MATRIX_DIM(bx,double,4,1);
      MAL_MATRIX(tmpb,double);
      MAL_MATRIX_DIM(cx,double,1,4);
      
      tmpA = MAL_RET_A_by_B(m_C,m_A);
      
      Ax(0,0)= 1.0;
      for(int i=0;i<3;i++)
	{
	  Ax(0,i+1) = tmpA(0,i);
	  for(int j=0;j<3;j++)
	    Ax(i+1,j+1) = m_A(i,j);
	}

      
      tmpb = MAL_RET_A_by_B(m_C,m_B);
      bx(0,0) = tmpb(0,0);
      for(int i=0;i<3;i++)
	{
	  bx(i+1,0) = m_B(i,0);
	}
      
      cx(0,0) =1.0;
      
      anOCS = new PatternGeneratorJRL::OptimalControllerSolver(Ax,bx,cx,Q,R,Nl);
      
      anOCS->ComputeWeights(OptimalControllerSolver::MODE_WITHOUT_INITIALPOS);
      
      anOCS->GetF(m_F);

      anOCS->GetK(lK);
      
      delete anOCS;
    }
  else if (mode==OptimalControllerSolver::MODE_WITH_INITIALPOS )
    {
      anOCS = new PatternGeneratorJRL::OptimalControllerSolver(m_A,m_B,m_C,Q,R,Nl);
      
      anOCS->ComputeWeights(PatternGeneratorJRL::OptimalControllerSolver::MODE_WITH_INITIALPOS);
      
      anOCS->GetF(m_F);

      anOCS->GetK(lK);

      delete anOCS;
    }
  
  m_Ks = lK(0,0);
  for (int i=0;i<3;i++)
    m_Kx(0,i) = lK(0,i);

  ODEBUG("m_Ks: " <<m_Ks);
  ODEBUG("m_Kx(0,0): " << m_Kx(0,0) << " " <<
	  "m_Kx(0,1): " << m_Kx(0,1) << " " <<
	  "m_Kx(0,2): " << m_Kx(0,2) );
  
  
  m_SizeOfPreviewWindow = (unsigned int)(m_PreviewControlTime/
					 m_SamplingPeriod);
  MAL_MATRIX_RESIZE(m_F,m_SizeOfPreviewWindow,1);
  
  m_Coherent = true;
}

int PreviewControl::OneIterationOfPreview(MAL_MATRIX( &x, double), 
					  MAL_MATRIX(& y, double),
					  double & sxzmp, double & syzmp,
					  deque<PatternGeneratorJRL::ZMPPosition> & ZMPPositions,
					  unsigned int lindex,
					  double & zmpx2, double & zmpy2,
					  bool Simulation)
{

  double ux=0.0, uy=0.0;

  MAL_MATRIX_DIM(r,double,1,1);

  // Compute the command.
  r = MAL_RET_A_by_B(m_Kx,x);
  ux = - r(0,0) + m_Ks * sxzmp ;

  if(ZMPPositions.size()<m_SizeOfPreviewWindow)
    {
      ODEBUG("ZMPPositions.size()<m_SizeOfPreviewWindow:" <<
	      ZMPPositions.size()<< " " << m_SizeOfPreviewWindow);
      exit(0);
    }
  
  for(unsigned int i=0;i<m_SizeOfPreviewWindow;i++)
    ux += m_F(i,0)* ZMPPositions[lindex+i].px;

  r = MAL_RET_A_by_B(m_Kx, y);

  uy = - r(0,0) + m_Ks * syzmp;
  for(unsigned int i=0;i<m_SizeOfPreviewWindow;i++)
    uy += m_F(i,0)* ZMPPositions[lindex+i].py;
  
  x = MAL_RET_A_by_B(m_A,x) + ux * m_B;
  y = MAL_RET_A_by_B(m_A,y) + uy * m_B;
   
  zmpx2 = (MAL_RET_A_by_B(m_C,x))(0,0);
  zmpy2 = (MAL_RET_A_by_B(m_C,y))(0,0);
  
  if (Simulation)
    {
      sxzmp += (ZMPPositions[lindex].px - zmpx2);
      syzmp += (ZMPPositions[lindex].py - zmpy2);
    }
  

  
  return 0;
}

int PreviewControl::OneIterationOfPreview1D(MAL_MATRIX( &x, double), 
					    double & sxzmp,
					    deque<double> & ZMPPositions,
					    unsigned int lindex,
					    double & zmpx2,
					    bool Simulation)
{

  double ux=0.0;

  MAL_MATRIX_DIM(r,double,1,1);

  // Compute the command.
  r = MAL_RET_A_by_B(m_Kx,x);
  ux = - r(0,0) + m_Ks * sxzmp ;
  
  ODEBUG( "x: " << x);
  ODEBUG(" ux phase 1: " << ux);
  if(ZMPPositions.size()<m_SizeOfPreviewWindow)
    {
      ODEBUG("ZMPPositions.size()<m_SizeOfPreviewWindow:" <<
	      ZMPPositions.size()<< " " << m_SizeOfPreviewWindow);
      exit(0);
    }
  
  for(unsigned int i=0;i<m_SizeOfPreviewWindow;i++)
    ux += m_F(i,0)* ZMPPositions[lindex+i];
  ODEBUG(" ux preview window phase: " << ux );
  x = MAL_RET_A_by_B(m_A,x) + ux * m_B;
   
  zmpx2 = (MAL_RET_A_by_B(m_C,x))(0,0);
  
  if (Simulation)
    {
      sxzmp += (ZMPPositions[lindex] - zmpx2);
    }
  
  ODEBUG("zmpx: " << zmpx2 );
  ODEBUG("sxzmp: " << sxzmp);
  ODEBUG("********");

  return 0;
}

int PreviewControl::OneIterationOfPreview1D(MAL_MATRIX( &x, double), 
					    double & sxzmp,
					    vector<double> & ZMPPositions,
					    unsigned int lindex,
					    double & zmpx2,
					    bool Simulation)
{

  double ux=0.0;

  MAL_MATRIX_DIM(r,double,1,1);

  // Compute the command.
  r = MAL_RET_A_by_B(m_Kx,x);
  ux = - r(0,0) + m_Ks * sxzmp ;
  
  ODEBUG( "x: " << x);
  ODEBUG(" ux phase 1: " << ux);
  if(ZMPPositions.size()<m_SizeOfPreviewWindow)
    {
      ODEBUG("ZMPPositions.size()< m_SizeOfPreviewWindow:" <<
	      ZMPPositions.size()<< " " << m_SizeOfPreviewWindow);
      exit(0);
    }
  
  int TestSize = ZMPPositions.size()-lindex - m_SizeOfPreviewWindow;

  if (TestSize>=0)
    {
      for(unsigned int i=0;i<m_SizeOfPreviewWindow;i++)
	ux += m_F(i,0)* ZMPPositions[lindex+i];
    }
  else
    {
      ODEBUG("Case where TestSize<0 (lindex:" <<lindex << 
	      " , ZMPPositions.size(): " << ZMPPositions.size() << 
	      " , m_SizeOfPreviewWindow: " << m_SizeOfPreviewWindow);
      for(unsigned int i=lindex;i<ZMPPositions.size();i++)
	ux += m_F(i,0)* ZMPPositions[i];

      int StillToRealized = m_SizeOfPreviewWindow - ZMPPositions.size() + lindex;
      for(unsigned int i=0;i<(unsigned int)StillToRealized ;i++)
	ux += m_F(i,0)* ZMPPositions[i];
    }
  ODEBUG(" ux preview window phase: " << ux );
  x = MAL_RET_A_by_B(m_A,x) + ux * m_B;
   
  zmpx2 = (MAL_RET_A_by_B(m_C,x))(0,0);
  
  if (Simulation)
    {
      sxzmp += (ZMPPositions[lindex] - zmpx2);
    }
  
  ODEBUG("zmpx: " << zmpx2 );
  ODEBUG("sxzmp: " << sxzmp);
  ODEBUG("********");

  return 0;
}


void PreviewControl::print()
{
  cout << "Zc: " <<  m_Zc <<endl;
  cout << "Sampling Period: " << m_SamplingPeriod <<endl;
  cout << "Preview control time window: "<<m_PreviewControlTime<<endl;
  
  for(int i=0;i<3;i++)
    cout << m_Kx(0,i) << " ";
  cout << endl;
  
  cout << "Ks "<< m_Ks << endl;
  
  cout << "F:"<<endl;
  for(unsigned int i=0;i<m_SizeOfPreviewWindow;i++)
    cout << m_F(i,0) << endl;
  
}
void PreviewControl::CallMethod(std::string & Method, std::istringstream &strm)
{
  if (Method==":samplingperiod")
    {
      std::string aws;
      if (strm.good())
	{
	  double lSamplingPeriod;
	  strm >> lSamplingPeriod;
	  SetSamplingPeriod(lSamplingPeriod);
	}
    }
  else if (Method==":previewcontroltime")
    {
      std::string aws;
      if (strm.good())
	{
	  double lpreviewcontroltime;
	  strm >> lpreviewcontroltime;
	  SetPreviewControlTime(lpreviewcontroltime);
	}
    }
  else if (Method==":comheight")
    { std::string aws;
      if (strm.good())
	{
	  double lcomheight;
	  strm >> lcomheight;
	  SetHeightOfCoM(lcomheight);
	}
    }
  else if (Method==":computeweightsofpreview")
    { 
      std::string aws;
      if (strm.good())
	{
	  string initialpos;
	  strm >> initialpos;
	  if (initialpos=="withinitialpos")
	    ComputeOptimalWeights(OptimalControllerSolver::MODE_WITH_INITIALPOS);
	  else if (initialpos=="withoutinitialpos")
	    ComputeOptimalWeights(OptimalControllerSolver::MODE_WITHOUT_INITIALPOS);
	}
    }
}
