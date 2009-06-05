/*! \file This object filters an analytical ZMP trajectory through preview control .

   Copyright (c) 2009, 
   Olivier Stasse

   JRL-Japan, CNRS/AIST

   Created: 2009/06/01 


   All rights reserved.
   
   For more information on the license please look at License.txt 
   in the root directory.

*/
#define ODEBUG(x)

#define ODEBUG3(x) cerr << "FilteringAnalyticalTrajectoryByPreviewControl" << ": " << __FUNCTION__ \
                       << "(# " << __LINE__ << "): "<< x << endl

#include <walkGenJrl/ZMPRefTrajectoryGeneration/FilteringAnalyticalTrajectoryByPreviewControl.h>


using namespace PatternGeneratorJRL;

FilteringAnalyticalTrajectoryByPreviewControl::FilteringAnalyticalTrajectoryByPreviewControl
(SimplePluginManager *lSPM,
 AnalyticalZMPCOGTrajectory* lAnalyticalZMPCOGTrajectory,
 PreviewControl * lPreviewControl) : SimplePlugin(lSPM)
{
  m_StartingTime = 0.0;
  m_Duration  = 0.0;
  m_SamplingPeriod = 0.0;

  m_AnalyticalZMPCOGTrajectory = 0;
  m_PreviewControl = 0;

  /*! Initialize the state vector used by the preview controller */
  MAL_MATRIX_RESIZE(m_ComState,3,1);
  m_ComState(0,0) = 0.0;
  m_ComState(1,0) = 0.0;
  m_ComState(2,0) = 0.0;
  SetAnalyticalTrajectory(lAnalyticalZMPCOGTrajectory);
  SetPreviewControl(lPreviewControl);

  m_LocalBufferIndex=0;

  std::string aMethodName[2] = 
    {":samplingperiod",
     ":previewcontroltime"};
  
  for(int i=0;i<2;i++)
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

void FilteringAnalyticalTrajectoryByPreviewControl::SetAnalyticalTrajectory(AnalyticalZMPCOGTrajectory *lAZCT)
{
  m_AnalyticalZMPCOGTrajectory = lAZCT;
}


void FilteringAnalyticalTrajectoryByPreviewControl::SetPreviewControl(PreviewControl *lPreviewControl)
{

  m_PreviewControl = lPreviewControl;
  m_LocalBufferIndex = 0;
  if (m_PreviewControl!=0)
    {
      m_PreviewControlTime = m_PreviewControl->PreviewControlTime();
      m_SamplingPeriod = m_PreviewControl->SamplingPeriod();

      Resize();
    }
}

void FilteringAnalyticalTrajectoryByPreviewControl::Resize()
{
  if ((m_SamplingPeriod!=0.0) && (m_PreviewControlTime!=0.0))
    {
      unsigned int DataBufferSize = (unsigned int ) (m_PreviewControlTime/
						     m_SamplingPeriod);
      ODEBUG3("DataBufferSize:" << DataBufferSize);
      m_DataBuffer.resize(DataBufferSize);
    }
}

FilteringAnalyticalTrajectoryByPreviewControl::~FilteringAnalyticalTrajectoryByPreviewControl()
{
}

bool FilteringAnalyticalTrajectoryByPreviewControl::FillInWholeBuffer(double FirstValueofZMPProfil,
								      double DeltaTj0 )
{
  ODEBUG("m_PreviewControl : " << m_PreviewControl << 
	 " m_AnalyticalZMPCOGTrajectory : " << m_AnalyticalZMPCOGTrajectory);

  if ((m_PreviewControl==0) || (m_AnalyticalZMPCOGTrajectory==0))
    return false;

  if (!m_PreviewControl->IsCoherent())
    m_PreviewControl->ComputeOptimalWeights(OptimalControllerSolver::MODE_WITH_INITIALPOS);

  m_Duration = DeltaTj0;

  double PreviewWindowTime = m_PreviewControl->PreviewControlTime();
  m_StartingTime = m_AnalyticalZMPCOGTrajectory->GetAbsoluteTimeReference();
  double DeltaT = m_PreviewControl->SamplingPeriod();

  unsigned int SizeOfBuffer = (unsigned int)(PreviewWindowTime/DeltaT);
  if (m_DataBuffer.size()!=SizeOfBuffer)
    m_DataBuffer.resize(SizeOfBuffer);

  unsigned int lDataBufferIndex = 0;
  double lZMP;
  double t=0;
  for( unsigned int lDataBufferIndex = 0;lDataBufferIndex<m_DataBuffer.size();t+=DeltaT,lDataBufferIndex++)
    {
      double r=0.0;
      if (t<DeltaTj0)
	{
	  m_AnalyticalZMPCOGTrajectory->ComputeZMP(t+m_StartingTime,lZMP);
	  r = FirstValueofZMPProfil - lZMP;
	}

      m_DataBuffer[lDataBufferIndex] = r;

    }

  /*! Initialize the state vector used by the preview controller */
  m_ComState(0,0) = 0.0;
  m_ComState(1,0) = 0.0;
  m_ComState(2,0) = 0.0;

  m_LocalBufferIndex = 0;
  return true;
}

bool FilteringAnalyticalTrajectoryByPreviewControl::UpdateOneStep(double t,
								  double &ZMPValue,
								  double &CoMValue,
								  double &CoMSpeedValue)
{
  ODEBUG("time:" << t << " m_StartingTime: " << 
	  m_StartingTime << " " << m_Duration + m_StartingTime);
  if ((t<m_StartingTime) || (t>m_Duration+m_StartingTime))
    return false;

  double lsxzmp =0.0;
  m_PreviewControl->OneIterationOfPreview1D(m_ComState,lsxzmp,m_DataBuffer,m_LocalBufferIndex,
					    ZMPValue,false);
  
  CoMValue = m_ComState(0,0);
  CoMSpeedValue = m_ComState(1,0);

  m_LocalBufferIndex++;
  if (m_LocalBufferIndex>=(int)m_DataBuffer.size())
    m_LocalBufferIndex = 0;
  return true;
}

/*! \brief Overloading method of SimplePlugin */
void FilteringAnalyticalTrajectoryByPreviewControl::CallMethod(std::string &Method,
							       std::istringstream &strm)
{
    if (Method==":samplingperiod")
    {
      std::string aws;
      if (strm.good())
	{
	  strm >> m_SamplingPeriod;
	  Resize();
	}
    }
  else if (Method==":previewcontroltime")
    {
      std::string aws;
      if (strm.good())
	{
	  strm >> m_PreviewControlTime;
	  Resize();
	}
    }

}
