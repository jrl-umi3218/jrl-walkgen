/*
 * Copyright 2008, 2009, 2010,
 *
 * Torea Foissotte
 * Olivier Stasse
 *
 * JRL, CNRS/AIST
 *
 * This file is part of jrl-walkgen.
 * jrl-walkgen is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * jrl-walkgen is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Lesser Public License for more details.
 * You should have received a copy of the GNU Lesser General Public License
 * along with jrl-walkgen.  If not, see <http://www.gnu.org/licenses/>.
 *
 *  Research carried out within the scope of the
 *  Joint Japanese-French Robotics Laboratory (JRL)
 */

/* This object handles several intervals for the foot trajectory generation. */
#include <iostream>
#include <iomanip>

#include "Debug.hh"
#include "FootTrajectoryGeneration/FootTrajectoryGenerationMultiple.hh"

using namespace PatternGeneratorJRL;

FootTrajectoryGenerationMultiple::
FootTrajectoryGenerationMultiple
(SimplePluginManager *lSPM,
 PRFoot *aFoot)
  : SimplePlugin(lSPM)
{
  m_Foot = aFoot ;
  m_Sensitivity=0.0;
}

FootTrajectoryGenerationMultiple::
~FootTrajectoryGenerationMultiple()
{
  for(unsigned int i=0;
      i<m_SetOfFootTrajectoryGenerationObjects.size();
      i++)
    {
      delete m_SetOfFootTrajectoryGenerationObjects[i];
    }
}

void FootTrajectoryGenerationMultiple::
SetNumberOfIntervals
(int lNumberOfIntervals)
{
  if (m_SetOfFootTrajectoryGenerationObjects.size()==
      (unsigned int)lNumberOfIntervals)
    return;

  for(unsigned int i=0; i<m_SetOfFootTrajectoryGenerationObjects.size(); i++)
    {
      delete m_SetOfFootTrajectoryGenerationObjects[i];
    }

  m_SetOfFootTrajectoryGenerationObjects.resize(lNumberOfIntervals);
  for(unsigned int i=0;
      i<m_SetOfFootTrajectoryGenerationObjects.size();
      i++)
    {
      m_SetOfFootTrajectoryGenerationObjects[i] =
        new FootTrajectoryGenerationStandard
        (getSimplePluginManager(),m_Foot);
      m_SetOfFootTrajectoryGenerationObjects[i]->
        InitializeInternalDataStructures();
    }
  m_NatureOfIntervals.resize(lNumberOfIntervals);
}

int FootTrajectoryGenerationMultiple::
GetNumberOfIntervals() const
{
  return static_cast<int>(m_SetOfFootTrajectoryGenerationObjects.size());
}


void FootTrajectoryGenerationMultiple::
SetTimeIntervals
(const vector<double> &lDeltaTj)
{
  m_DeltaTj = lDeltaTj;
  m_RefTime.resize(lDeltaTj.size());
  double reftime=0.0;

  for(unsigned int li=0; li<m_DeltaTj.size(); li++)
    {
      m_RefTime[li] = reftime;
      ODEBUG(" m_RefTime["<< li <<"]: " << setprecision(12)
             << m_RefTime[li] << " reftime: "
             << setprecision(12) << reftime );
      reftime+=m_DeltaTj[li];
    }

}

void FootTrajectoryGenerationMultiple::
GetTimeIntervals(vector<double> &lDeltaTj)
  const
{
  lDeltaTj = m_DeltaTj;
}

bool FootTrajectoryGenerationMultiple::
Compute(int axis, double t, double &result)
{
  t -= m_AbsoluteTimeReference;
  result = -1.0;
  double reftime=0;
  ODEBUG(" ====== CoM ====== ");
  ODEBUG(" t: " << t << " reftime :" << reftime << " m_Sensitivity: "
         << m_Sensitivity <<" m_DeltaTj.size(): "<< m_DeltaTj.size() );

  for(unsigned int j=0; j<m_DeltaTj.size(); j++)
    {
      ODEBUG(" t: " << t << " reftime :" << reftime
             << " Tj["<<j << "]= "<< m_DeltaTj[j]);

      if (((t+m_Sensitivity)>=reftime) &&
          (t<=reftime+m_DeltaTj[j]+m_Sensitivity))
        {
          double deltaj=0.0;
          deltaj = t-reftime;

          if (m_SetOfFootTrajectoryGenerationObjects[j]!=0)
            {
              result =
                m_SetOfFootTrajectoryGenerationObjects[j]->
                Compute(axis,deltaj);
            }
          return true;
        }

      reftime+=m_DeltaTj[j];
    }
  ODEBUG(" reftime :" << reftime );

  return false;
}


bool FootTrajectoryGenerationMultiple::
Compute
(double t,
 FootAbsolutePosition & aFootAbsolutePosition,
 unsigned int IndexInterval)
{
  double deltaj =
    t - m_AbsoluteTimeReference - m_RefTime[IndexInterval];
  ODEBUG("IndexInterval : " << IndexInterval );


  // Use polynoms
  //m_SetOfFootTrajectoryGenerationObjects[IndexInterval]->
  // ComputeAllWithPolynom(aFootAbsolutePosition,deltaj);

  // Use BSplines
  m_SetOfFootTrajectoryGenerationObjects[IndexInterval]->
    ComputeAllWithBSplines(aFootAbsolutePosition,deltaj);


  aFootAbsolutePosition.stepType =
    m_NatureOfIntervals[IndexInterval];

  /*   ofstream aof;
       string aFileName;

       aFileName ="ex.dat";
       aof.open(aFileName.c_str(),ofstream::app);
       aof << "deltaj " << deltaj << " t " << t << " m_Absoulute "
       << m_AbsoluteTimeReference << " Ref " << m_RefTime[IndexInterval]
       << " j " << IndexInterval << " foot  "<< aFootAbsolutePosition.x
       << " "<< aFootAbsolutePosition.z << endl;
       aof.close();*/

  return true;
}


bool FootTrajectoryGenerationMultiple::
Compute
(double t, FootAbsolutePosition & aFootAbsolutePosition)
{
  t -= m_AbsoluteTimeReference;
  double reftime=0;
  ODEBUG(" ====== Foot ====== " << m_DeltaTj.size());
  ODEBUG("t: " << setprecision(12) << t
         << " reftime :" << reftime <<
         " m_Sensitivity: "<< m_Sensitivity
         <<" m_DeltaTj.size(): "<< m_DeltaTj.size() );

  for(unsigned int j=0; j<m_DeltaTj.size(); j++)
    {
      ODEBUG("t: " << t << " reftime :"
             << setprecision(12) << reftime <<
             " Tj["<<j << "]= " << setprecision(12)
             << m_DeltaTj[j]
             <<" max limit: " << setprecision(12)
             << (reftime+m_DeltaTj[j]+m_Sensitivity) );

      ODEBUG(" Tj["<<j << "]= " << setprecision(12) << m_DeltaTj[j] );

      if (((t+m_Sensitivity)>=reftime) &&
          (t<=reftime+m_DeltaTj[j]+m_Sensitivity))
        {
          double deltaj=0.0;
          deltaj = t-reftime;

          if (m_SetOfFootTrajectoryGenerationObjects[j]!=0)
            {
              //m_SetOfFootTrajectoryGenerationObjects[j]->
              //ComputeAllWithPolynom(aFootAbsolutePosition,deltaj);
              m_SetOfFootTrajectoryGenerationObjects[j]->
                ComputeAllWithBSplines(aFootAbsolutePosition,deltaj);
              aFootAbsolutePosition.stepType = m_NatureOfIntervals[j];
            }
          ODEBUG("t: " << t << " reftime :" << setprecision(12)
                 << reftime
                 << " AbsoluteTimeReference : "
                 << m_AbsoluteTimeReference
                 << " Tj["<<j << "]= " << setprecision(12) << m_DeltaTj[j]
                 <<" max limit: " << setprecision(12)
                 << (reftime+m_DeltaTj[j]+m_Sensitivity) );
          ODEBUG("X: " << aFootAbsolutePosition.x <<
                 " Y: " << aFootAbsolutePosition.y <<
                 " Z: " << aFootAbsolutePosition.z <<
                 " Theta: " << aFootAbsolutePosition.theta <<
                 " Omega: " << aFootAbsolutePosition.omega <<
                 " stepType: " << aFootAbsolutePosition.stepType <<
                 " NI: " << m_NatureOfIntervals[j] <<
                 " interval : " << j);

          return true;
        }

      reftime+=m_DeltaTj[j];
    }
  ODEBUG(" reftime :" << reftime <<
         " m_AbsoluteReferenceTime" << m_AbsoluteTimeReference);
  ODEBUG("t: " << setprecision(12) << t
         << " reftime :" << reftime
         << " m_Sensitivity: "<< m_Sensitivity
         << " m_DeltaTj.size(): "<< m_DeltaTj.size() );

  return false;
}

/*! This method specifies the nature of the interval.
 */
int FootTrajectoryGenerationMultiple::
SetNatureInterval(unsigned int IntervalIndex,
                  int Nature)
{
  if (IntervalIndex>=m_NatureOfIntervals.size())
    return -1;
  m_NatureOfIntervals[IntervalIndex] = Nature;
  return 0;

}

/*! This method returns the nature of the interval.
 */
int FootTrajectoryGenerationMultiple::
GetNatureInterval(unsigned int IntervalIndex) const
{
  if (IntervalIndex>=m_NatureOfIntervals.size())
    return -100;

  return  m_NatureOfIntervals[IntervalIndex];
}


/*! This method specifies the parameters for each of the polynome
  used by this object. In this case, as it is used for the 3rd
  order polynome. The polynome to which those parameters are set
  is specified with PolynomeIndex.
  @param PolynomeIndex: Set to which axis the parameters will be applied.
  @param TimeInterval: Set the time base of the polynome.
  @param Position: Set the final position of the polynome at TimeInterval.
  @param InitPosition: Initial position when computing the polynome at t=0.0.
  @param InitSpeed: Initial speed when computing the polynome at t=0.0.
*/
int FootTrajectoryGenerationMultiple::
SetParametersWithInitPosInitSpeed
(unsigned int IntervalIndex,
 int AxisReference,
 double TimeInterval,
 double FinalPosition,
 double InitPosition,
 double InitSpeed,
 vector<double> middlePos
 )
{
  if (IntervalIndex>=
      m_SetOfFootTrajectoryGenerationObjects.size())
    return -1;

  m_SetOfFootTrajectoryGenerationObjects[IntervalIndex]->
    SetParametersWithInitPosInitSpeed
    (AxisReference,
     TimeInterval,
     FinalPosition,
     InitPosition,
     InitSpeed,
     middlePos);
  return 0;
}


/*! This method specifies the parameters for each of the polynome used by this
  object. In this case, as it is used for the 3rd order polynome. The polynome
  to which those parameters are set is specified with PolynomeIndex.
  @param PolynomeIndex: Set to which axis the parameters will be applied.
  @param TimeInterval: Set the time base of the polynome.
  @param Position: Set the final position of the polynome at TimeInterval.
*/
int FootTrajectoryGenerationMultiple::
SetParameters
(unsigned int IntervalIndex,
 int AxisReference,
 double TimeInterval,
 double FinalPosition)
{
  if (IntervalIndex>=
      m_SetOfFootTrajectoryGenerationObjects.size())
    return -1;

  return SetParametersWithInitPosInitSpeedInitAcc
    ( IntervalIndex,
      AxisReference,
      TimeInterval,
      FinalPosition,
      0.0,0.0,0.0 );
}
/*! This method specifies the parameters for each of the polynome used by this
  object. In this case, as it is used for the 3rd order polynome. The polynome
  to which those parameters are set is specified with PolynomeIndex.
  @param PolynomeIndex: Set to which axis the parameters will be applied.
  @param AxisReference: Index to the axis to be used.
  @param TimeInterval: Set the time base of the polynome.
  @param FinalPosition: Set the final position of the polynome at TimeInterval.
  @param InitPosition: Initial position when computing the polynome at
  t= m_AbsoluteTimeReference.
  @param InitSpeed: Initial speed when computing the polynome at
  t=m_AbsoluteTimeReference.
  @param InitAcc: Initial speed when computing the polynome at
  t=m_AbsoluteTimeReference.
*/
int FootTrajectoryGenerationMultiple::
SetParametersWithInitPosInitSpeedInitAcc
(unsigned int IntervalIndex,
 int AxisReference,
 double TimeInterval,
 double FinalPosition,
 double InitPosition,
 double InitSpeed,
 double InitAcc,
 vector<double> middlePos)
{
  if (IntervalIndex>=
      m_SetOfFootTrajectoryGenerationObjects.size())
    return -1;


  m_SetOfFootTrajectoryGenerationObjects[IntervalIndex]->
    SetParameters
    (AxisReference,
     TimeInterval,
     FinalPosition,
     InitPosition,
     InitSpeed,
     InitAcc,
     middlePos);
  return 0;
}

/*! This method get the parameters for each of the polynome used by this
  object. In this case, as it is used for the 3rd order polynome. The polynome
  to which those parameters are set is specified with PolynomeIndex.
  @param PolynomeIndex: Set to which axis the parameters will be applied.
  @param TimeInterval: Set the time base of the polynome.
  @param Position: Set the final position of the polynome at TimeInterval.
  @param InitPosition: Initial position when computing the polynome at t=0.0.
  @param InitSpeed: Initial speed when computing the polynome at t=0.0.
*/
int FootTrajectoryGenerationMultiple::
GetParametersWithInitPosInitSpeed
(unsigned int IntervalIndex,
 int AxisReference,
 double &TimeInterval,
 double &FinalPosition,
 double &InitPosition,
 double &InitSpeed)
{
  if (IntervalIndex>=
      m_SetOfFootTrajectoryGenerationObjects.size())
    return -1;


  m_SetOfFootTrajectoryGenerationObjects[IntervalIndex]->
    GetParametersWithInitPosInitSpeed
    (AxisReference,
     TimeInterval,
     FinalPosition,
     InitPosition,
     InitSpeed);

  return 0;
}



double FootTrajectoryGenerationMultiple::
GetAbsoluteTimeReference() const
{
  return m_AbsoluteTimeReference;
}

void FootTrajectoryGenerationMultiple::
SetAbsoluteTimeReference
(double lAbsoluteTimeReference)
{
  m_AbsoluteTimeReference = lAbsoluteTimeReference;
}

void FootTrajectoryGenerationMultiple::
CallMethod
(std::string &, //Method,
 std::istringstream & ) //strm)
{

}
int FootTrajectoryGenerationMultiple::
DisplayIntervals() const
{
  for(unsigned int i=0; i<m_DeltaTj.size(); i++)
    {
      std::cout << "m_DeltaTj["<<i<<"]="<<m_DeltaTj[i] << std::endl;
    }
  return 0;
}

FootTrajectoryGenerationMultiple &
FootTrajectoryGenerationMultiple::operator=
(const FootTrajectoryGenerationMultiple & aFTGM)
{

  /* Specify the number of intervals. */
  SetNumberOfIntervals(aFTGM.GetNumberOfIntervals());

  /* Copy time interval */
  std::vector<double> lDeltaTj;
  aFTGM.GetTimeIntervals(lDeltaTj);
  SetTimeIntervals(lDeltaTj);

  /* Copy nature of intervals */
  for(unsigned int li=0; li<lDeltaTj.size(); li++)
    SetNatureInterval(li,aFTGM.GetNatureInterval(li));

  /* Set absolute time reference */
  SetAbsoluteTimeReference(aFTGM.GetAbsoluteTimeReference());

  /* Copy the parameters */
  unsigned int li=0 ;
  int lk=0 ;
  for(li=0; li<lDeltaTj.size(); li++)
    {
      double TI, FP, IP, IS;
      for(lk=0; lk<6; lk++)
        {
          GetParametersWithInitPosInitSpeed(li,lk,TI,FP,IP,IS);
          /*! Special case when TI is equal to zero */
          if (TI==0.0)
            SetParameters(li,lk,TI,FP);
          else
            SetParametersWithInitPosInitSpeed(li,lk,TI,FP,IP,IS);
        }
    }
  return *this;
}
