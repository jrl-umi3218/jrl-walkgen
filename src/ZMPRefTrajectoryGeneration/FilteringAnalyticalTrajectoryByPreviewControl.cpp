/*
 * Copyright 2008, 2009, 2010,
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
/*! \file This object filters an analytical ZMP trajectory through 
  preview control. */
#include <iostream>
#include <fstream>
#include "Debug.hh"
#include "FilteringAnalyticalTrajectoryByPreviewControl.hh"


using namespace PatternGeneratorJRL;

FilteringAnalyticalTrajectoryByPreviewControl::
FilteringAnalyticalTrajectoryByPreviewControl
(SimplePluginManager *lSPM,
 AnalyticalZMPCOGTrajectory* lAnalyticalZMPCOGTrajectory,
 PreviewControl * lPreviewControl) : SimplePlugin(lSPM)
{
  m_StartingTime = 0.0;
  m_Duration  = 0.0;
  m_SamplingPeriod = 0.0;
  m_Tsingle = 0.0;

  m_AnalyticalZMPCOGTrajectory = 0;
  m_PreviewControl = 0;

  /*! Initialize the state vector used by the preview controller */
  m_ComState.resize(3,1);
  m_ComState(0,0) = 0.0;
  m_ComState(1,0) = 0.0;
  m_ComState(2,0) = 0.0;
  SetAnalyticalTrajectory(lAnalyticalZMPCOGTrajectory);
  SetPreviewControl(lPreviewControl);

  m_LocalBufferIndex=0;

  std::string aMethodName[3] =
    {
     ":samplingperiod",
     ":previewcontroltime",
     ":singlesupporttime"
    };

  for(int i=0; i<3; i++)
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

void FilteringAnalyticalTrajectoryByPreviewControl::
SetAnalyticalTrajectory
(AnalyticalZMPCOGTrajectory *lAZCT)
{
  m_AnalyticalZMPCOGTrajectory = lAZCT;
}


void FilteringAnalyticalTrajectoryByPreviewControl::
SetPreviewControl
( PreviewControl *lPreviewControl)
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
#if 0
  if ((m_SamplingPeriod!=0.0) &&
      (m_PreviewControlTime!=0.0) &&
      (m_Tsingle!=0.0))
    {
      unsigned int DataBufferSize = (unsigned int ) ((m_Tsingle
                                                      +m_PreviewControlTime)/
                                                     m_SamplingPeriod);
      ODEBUG3("m_Tsingle: " << m_Tsingle << " DataBufferSize:"
              << DataBufferSize);
      m_DataBuffer.resize(DataBufferSize);
    }
#endif
}

FilteringAnalyticalTrajectoryByPreviewControl::
~FilteringAnalyticalTrajectoryByPreviewControl()
{
}

bool FilteringAnalyticalTrajectoryByPreviewControl::
FillInWholeBuffer
(double FirstValueofZMPProfil,
 double DeltaTj0 )
{
  ODEBUG("m_PreviewControl : " << m_PreviewControl <<
         " m_AnalyticalZMPCOGTrajectory : " << m_AnalyticalZMPCOGTrajectory);

  if ((m_PreviewControl==0) || (m_AnalyticalZMPCOGTrajectory==0))
    return false;

  if (!m_PreviewControl->IsCoherent())
    m_PreviewControl->ComputeOptimalWeights
      ( OptimalControllerSolver::MODE_WITH_INITIALPOS);

  m_Duration = DeltaTj0;

  double PreviewWindowTime = m_PreviewControl->PreviewControlTime();
  m_StartingTime = m_AnalyticalZMPCOGTrajectory->GetAbsoluteTimeReference();
  double DeltaT = m_PreviewControl->SamplingPeriod();

  unsigned int SizeOfBuffer = (unsigned int)((DeltaTj0+PreviewWindowTime)
                                             /DeltaT);
  ODEBUG("SizeOfBuffer: " <<SizeOfBuffer<< " Duration : "<<m_Duration);
  if (m_DataBuffer.size()!=SizeOfBuffer)
    m_DataBuffer.resize(SizeOfBuffer);

  double lZMP;
  double t=0;

  if (0)
    {
      ofstream aof;
      static unsigned int nbofmodifs = 0;
      char Buffer[1024];
      sprintf(Buffer,"Diff_%05d.dat",nbofmodifs++);
      aof.open(Buffer,ofstream::out);
    }
  // On the interval of the newly changed first foot.
  for( unsigned int lDataBufferIndex = 0; lDataBufferIndex<m_DataBuffer.size();
       t+=DeltaT,lDataBufferIndex++)
    {
      double r=0.0;
      if (t<DeltaTj0)
        {
          m_AnalyticalZMPCOGTrajectory->ComputeZMP(t+m_StartingTime,lZMP);
          // The difference between the desired ZMP (FirstValueofZMPProfil)
          // and the analytical value is computed.
          r = FirstValueofZMPProfil - lZMP;
        }

      m_DataBuffer[lDataBufferIndex] = r;
      // aof << r << endl;
    }
  //aof.close();

  /*! Initialize the state vector used by the preview controller */
  m_ComState(0,0) = 0.0;
  m_ComState(1,0) = 0.0;
  m_ComState(2,0) = 0.0;

  m_ZMPPCValue = 0;

  m_LocalBufferIndex = 0;
  return true;
}

bool FilteringAnalyticalTrajectoryByPreviewControl::
UpdateOneStep
(double t,
 double &ZMPValue,
 double &CoMValue,
 double &CoMSpeedValue)
{
  ODEBUG("time:" << t << " m_StartingTime: " <<
         m_StartingTime << " " << m_Duration + m_StartingTime << " ( "
         << m_Duration << " ) "
         << " LBI:" << m_LocalBufferIndex);
  if ((t<m_StartingTime) || (t>m_Duration+m_StartingTime) || (m_Duration==0.0))
    return false;

  double lsxzmp =0.0;
  m_PreviewControl->OneIterationOfPreview1D(m_ComState,lsxzmp,m_DataBuffer,
                                            m_LocalBufferIndex,
                                            m_ZMPPCValue,false);

  ZMPValue = m_ZMPPCValue;
  CoMValue = m_ComState(0,0);
  CoMSpeedValue = m_ComState(1,0);

  m_LocalBufferIndex++;
  if (m_LocalBufferIndex>=(int)m_DataBuffer.size())
    m_LocalBufferIndex = 0;
  return true;
}

/*! \brief Overloading method of SimplePlugin */
void FilteringAnalyticalTrajectoryByPreviewControl::
CallMethod
(std::string &Method,
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
  else if (Method==":singlesupporttime")
    {
      std::string aws;
      if (strm.good())
        {
          strm >> m_Tsingle;
          Resize();
        }
    }

}
