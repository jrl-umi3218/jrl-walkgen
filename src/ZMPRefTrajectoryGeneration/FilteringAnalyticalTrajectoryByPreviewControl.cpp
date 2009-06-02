/*! \file This object filters an analytical ZMP trajectory through preview control .

   Copyright (c) 2009, 
   Olivier Stasse

   JRL-Japan, CNRS/AIST

   Created: 2009/06/01 


   All rights reserved.
   
   For more information on the license please look at License.txt 
   in the root directory.

*/

#include <walkGenJrl/ZMPRefTrajectoryGeneration/FilteringAnalyticalTrajectoryByPreviewControl.h>


using namespace PatternGeneratorJRL;

FilteringAnalyticalTrajectoryByPreviewControl::FilteringAnalyticalTrajectoryByPreviewControl
(AnalyticalZMPCOGTrajectory* lAnalyticalZMPCOGTrajectory,
 PreviewControl * lPreviewControl)
{
  MAL_MATRIX_RESIZE(m_ComState,3,1);
  /*! Initialize the state vector used by the preview controller */
  m_ComState(0,0) = 0.0;
  m_ComState(1,0) = 0.0;
  m_ComState(2,0) = 0.0;
  SetAnalyticalTrajectory(lAnalyticalZMPCOGTrajectory);
  SetPreviewControl(lPreviewControl);

  m_LocalBufferIndex=0;
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
      unsigned int DataBufferSize = (unsigned int ) (m_PreviewControl->PreviewControlTime()/
						     m_PreviewControl->SamplingPeriod());
      m_DataBuffer.resize(DataBufferSize);
    }
}

FilteringAnalyticalTrajectoryByPreviewControl::~FilteringAnalyticalTrajectoryByPreviewControl()
{
}

bool FilteringAnalyticalTrajectoryByPreviewControl::FillInWholeBuffer(double FirstValueofZMPProfil,
								      double DeltaTj0 )
{
  if ((m_PreviewControl==0) || (m_AnalyticalZMPCOGTrajectory==0))
    return false;

  double PreviewWindowTime = m_PreviewControl->PreviewControlTime();
  double StartingTime = m_AnalyticalZMPCOGTrajectory->GetAbsoluteTimeReference();
  double DeltaT = m_PreviewControl->SamplingPeriod();

  unsigned int lDataBufferIndex = 0;
  double lZMP;

  for(double t=0;t<PreviewWindowTime;t+=DeltaT)
    {
      double r=0.0;
      if (t<DeltaTj0)
	{
	  m_AnalyticalZMPCOGTrajectory->ComputeZMP(t+StartingTime,lZMP);
	  r = FirstValueofZMPProfil - lZMP;
	}

      m_DataBuffer[lDataBufferIndex] = r;

      lDataBufferIndex++;
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

  double lsxzmp =0.0;
  m_PreviewControl->OneIterationOfPreview1D(m_ComState,lsxzmp,m_DataBuffer,m_LocalBufferIndex,
					    m_ZMP,false);
  
  ZMPValue = m_ZMP;
  CoMValue = m_ComState(0,0);
  CoMSpeedValue = m_ComState(1,0);

  m_LocalBufferIndex++;
  if (m_LocalBufferIndex>=m_DataBuffer.size())
    m_LocalBufferIndex = 0;
  return true;
}

