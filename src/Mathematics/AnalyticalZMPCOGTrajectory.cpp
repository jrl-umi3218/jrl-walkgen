/*
 * Copyright 2008, 2009, 2010,
 *
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

/** \file AnalyticalZMPCOGTrajectory.h
    \brief This object deals with analytical ZMP and CoG trajectories. */
#include <iostream>
#include <fstream>
#include <math.h>
#include <Mathematics/AnalyticalZMPCOGTrajectory.hh>

#include <Debug.hh>
using namespace std;

namespace PatternGeneratorJRL
{

  AnalyticalZMPCOGTrajectory::AnalyticalZMPCOGTrajectory(int lNbOfIntervals)
  {
    SetNumberOfIntervals(lNbOfIntervals);
    m_AbsoluteTimeReference = 0.0;
    m_Sensitivity = 0.0;
  }


  AnalyticalZMPCOGTrajectory::~AnalyticalZMPCOGTrajectory()
  {
    FreePolynomes();
  }

  void AnalyticalZMPCOGTrajectory::FreePolynomes()
  {
    if(m_ListOfCOGPolynomials.size()>0)
      {
        for(unsigned int i=0; i<m_ListOfCOGPolynomials.size(); i++)
          {
            if (m_ListOfCOGPolynomials[i]!=0)
              delete m_ListOfCOGPolynomials[i];
          }
      }
    if(m_ListOfZMPPolynomials.size()>0)
      {
        for(unsigned int i=0; i<m_ListOfZMPPolynomials.size(); i++)
          {
            if (m_ListOfZMPPolynomials[i]!=0)
              delete m_ListOfZMPPolynomials[i];
          }
      }
  }

  void AnalyticalZMPCOGTrajectory::SetNumberOfIntervals(unsigned int
                                                        lNbOfIntervals)
  {
    m_NbOfIntervals = lNbOfIntervals;

    m_V.resize(m_NbOfIntervals);
    m_W.resize(m_NbOfIntervals);
    m_DeltaTj.resize(m_NbOfIntervals);
    m_omegaj.resize(m_NbOfIntervals);
    m_PolynomialDegree.resize(m_NbOfIntervals);
    FreePolynomes();
    m_ListOfCOGPolynomials.resize(m_NbOfIntervals);
    for(unsigned int i=0; i<m_ListOfCOGPolynomials.size(); i++)
      m_ListOfCOGPolynomials[i] =0;

    m_ListOfZMPPolynomials.resize(m_NbOfIntervals);
    for(unsigned int i=0; i<m_ListOfZMPPolynomials.size(); i++)
      m_ListOfZMPPolynomials[i] =0;

  }

  bool AnalyticalZMPCOGTrajectory::ComputeCOM(double t, double &r)
  {
    t -= m_AbsoluteTimeReference;
    r = -1.0;
    double reftime=0;
    ODEBUG(" ====== CoM ====== ");
    for(unsigned int j=0; j<m_DeltaTj.size(); j++)
      {
        ODEBUG("t: " << t << " reftime :" << reftime << " Tj["<<j << "]= "<<
               m_DeltaTj[j]);
        if (((t+m_Sensitivity)>=reftime) &&
            (t<=reftime+m_DeltaTj[j]+m_Sensitivity))
          {
            double deltaj=0.0;
            deltaj = t-reftime;

            r = cosh(m_omegaj[j]*deltaj) * m_V[j] +
              sinh(m_omegaj[j]*deltaj) * m_W[j];
            ODEBUG( " coefficients hyperbolique : " << r << \
                    " m_omegaj["<<j<<"]=" << m_omegaj[j] <<          \
                    " deltaj " << deltaj <<                   \
                    " m_V["<<j<<"]:" << m_V[j] <<             \
                    " m_W["<<j<<"]:" << m_W[j] <<
                    " m_T["<<j<<"]:" << m_DeltaTj[j]);
            if (m_ListOfCOGPolynomials[j]!=0)
              {
                double add2r;
                add2r = m_ListOfCOGPolynomials[j]->Compute(deltaj);
                ODEBUG( " add2r: " << add2r <<" " \
                        << "Polynomial ("<<j
                        <<") with degree (supposed:"<<  m_PolynomialDegree[j]
                        <<")"<<endl \
                        << "Coefficients:");
                //           m_ListOfCOGPolynomials[j]->print();
                r+=add2r;
              }
            else
              return false;
            return true;
          }

        reftime+=m_DeltaTj[j];
      }
    return false;
  }


  bool AnalyticalZMPCOGTrajectory::ComputeCOMSpeed(double t, double &r)
  {
    t -= m_AbsoluteTimeReference;
    r = -1.0;
    double reftime=0;
    ODEBUG(" ====== CoM ====== ");
    for(unsigned int j=0; j<m_DeltaTj.size(); j++)
      {
        ODEBUG("t: " << t << " reftime :" << reftime << " Tj["<<j << "]= "<<
               m_DeltaTj[j]);
        if (((t+m_Sensitivity)>=reftime) &&
            (t<=reftime+m_DeltaTj[j]+m_Sensitivity))
          {
            double deltaj=0.0;
            deltaj = t-reftime;

            r = m_omegaj[j] * sinh(m_omegaj[j]*deltaj) * m_V[j] +
              m_omegaj[j] * cosh(m_omegaj[j]*deltaj) * m_W[j];
            ODEBUG( " coefficients hyperbolique : " << r << \
                    " m_omegaj[j]=" << m_omegaj[j] << \
                    " deltaj " << deltaj <<  \
                    " m_V[j]:" << m_V[j] <<  \
                    " m_W[j]:" << m_W[j] );
            if (m_ListOfCOGPolynomials[j]!=0)
              {
                double add2r;
                add2r = m_ListOfCOGPolynomials[j]->ComputeDerivative(deltaj);
                ODEBUG( " add2r: " << add2r <<" " \
                        << "Polynomial ("<<j
                        << ") with degree (supposed:"<<  m_PolynomialDegree[j]
                        << ")"<<endl \
                        << "Coefficients:");
                //           m_ListOfCOGPolynomials[j]->print();
                r+=add2r;
              }
            else
              return false;
            return true;
          }

        reftime+=m_DeltaTj[j];
      }
    return false;
  }

  bool AnalyticalZMPCOGTrajectory::ComputeCOM(double t, double &r, int j)
  {
    double deltaj=0.0;
    deltaj = t- m_AbsoluteTimeReference - m_RefTime[j];
    r = cosh(m_omegaj[j]*deltaj) * m_V[j] +
      sinh(m_omegaj[j]*deltaj) * m_W[j];
    r += m_ListOfCOGPolynomials[j]->Compute(deltaj);
    return true;
  }

  bool AnalyticalZMPCOGTrajectory::ComputeCOMSpeed(double t, double &r, int j)
  {
    double deltaj=0.0;
    deltaj = t- m_AbsoluteTimeReference - m_RefTime[j];

    r = m_omegaj[j] * sinh(m_omegaj[j]*deltaj) * m_V[j] +
      m_omegaj[j] * cosh(m_omegaj[j]*deltaj) * m_W[j];
    r += m_ListOfCOGPolynomials[j]->ComputeDerivative(deltaj);
    ODEBUG("ComputeCOMSpeed: " << r);
    return true;
  }

  bool AnalyticalZMPCOGTrajectory::ComputeCOMAcceleration(double t, double &r,
                                                          int j)
  {
    double deltaj=0.0;
    deltaj = t- m_AbsoluteTimeReference - m_RefTime[j];

    r = m_omegaj[j] * m_omegaj[j] * cosh(m_omegaj[j]*deltaj) * m_V[j] +
      m_omegaj[j] * m_omegaj[j] * sinh(m_omegaj[j]*deltaj) * m_W[j];
    r += m_ListOfCOGPolynomials[j]->ComputeSecDerivative(deltaj);
    ODEBUG("ComputeCOMAcceleration: " << r);
    return true;
  }

  bool AnalyticalZMPCOGTrajectory::ComputeZMP(double t, double &r)
  {
    r = -1.0;
    t -= m_AbsoluteTimeReference;
    double reftime =0.0;
    ODEBUG(" ====== ZMP ====== " );
    for(unsigned int j=0; j<m_DeltaTj.size(); j++)
      {
        if (((t+m_Sensitivity)>=reftime) &&
            (t<=reftime+m_DeltaTj[j]+m_Sensitivity))
          {
            double deltaj=0.0;
            deltaj = t-reftime;
            r = 0;
            ODEBUG(" coefficients hyperbolique : " << r << \
                   " m_omegaj[j]=" << m_omegaj[j] <<       \
                   " deltaj " << deltaj <<          \
                   " m_V[j]:" << m_V[j] <<          \
                   " m_W[j]:" << m_W[j] );

            if (m_ListOfZMPPolynomials[j]!=0)
              {
                double add2r;
                add2r = m_ListOfZMPPolynomials[j]->Compute(deltaj);
                ODEBUG( " add2r: " << add2r << " " );
                ODEBUG("Polynomial ("<<j
                       <<") with degree (supposed:"<<  m_PolynomialDegree[j] <<
                       ")");
                ODEBUG("Coefficients:");
                //           m_ListOfZMPPolynomials[j]->print();
                r+=add2r;
              }
            else
              return false;
            return true;
          }
        reftime+=m_DeltaTj[j];
      }
    return false;
  }

  bool AnalyticalZMPCOGTrajectory::ComputeZMPSpeed(double t, double &r)
  {
    r = -1.0;
    t -= m_AbsoluteTimeReference;
    double reftime=0;
    ODEBUG(" ====== CoM ====== ");
    for(unsigned int j=0; j<m_DeltaTj.size(); j++)
      {
        ODEBUG("t: " << t << " reftime :" << reftime << " Tj["<<j << "]= "<<
               m_DeltaTj[j]);
        if (((t+m_Sensitivity)>=reftime) &&
            (t<=reftime+m_DeltaTj[j]+m_Sensitivity))
          {
            double deltaj=0.0;
            deltaj = t-reftime;

            r = 0.0;
            ODEBUG( " coefficients hyperbolique : " << r << \
                    " m_omegaj[j]=" << m_omegaj[j] << \
                    " deltaj " << deltaj <<  \
                    " m_V[j]:" << m_V[j] <<  \
                    " m_W[j]:" << m_W[j] );
            if (m_ListOfCOGPolynomials[j]!=0)
              {
                double add2r;
                add2r = m_ListOfCOGPolynomials[j]->ComputeDerivative(deltaj);
                ODEBUG( " add2r: " << add2r <<" " \
                        << "Polynomial ("<<j
                        <<") with degree (supposed:"
                        <<  m_PolynomialDegree[j] << ")"<<endl \
                        << "Coefficients:");
                //           m_ListOfCOGPolynomials[j]->print();
                r+=add2r;
              }
            else
              return false;
            return true;
          }

        reftime+=m_DeltaTj[j];
      }
    return false;
  }

  bool AnalyticalZMPCOGTrajectory::ComputeZMP(double t, double &r, int j)
  {
    double deltaj=0.0;
    deltaj = t-m_AbsoluteTimeReference - m_RefTime[j];
    r = m_ListOfZMPPolynomials[j]->Compute(deltaj);
    return true;
  }

  void AnalyticalZMPCOGTrajectory::
  SetCoGHyperbolicCoefficients
  (vector<double> &lV,
   vector<double> &lW)
  {
    if ((int)lV.size()==m_NbOfIntervals)
      m_V = lV;
    if ((int)lW.size()==m_NbOfIntervals)
      m_W = lW;
  }

  void AnalyticalZMPCOGTrajectory::
  SetStartingTimeIntervalsAndHeightVariation
  (vector<double> &lTj,
   vector<double> &lomegaj)
  {
    if ((int)lTj.size()==m_NbOfIntervals)
      {
        m_DeltaTj = lTj;
        m_RefTime.resize(lTj.size());
        double reftime = 0;
        for(unsigned int j=0; j<m_DeltaTj.size(); j++)
          {
            m_RefTime[j] = reftime;
            reftime += m_DeltaTj[j];
          }
      }
    else
      cerr << "Pb while initializing the time intervals. "
           << lTj.size() << " " <<
        m_NbOfIntervals << endl;
    if ((int)lomegaj.size()==m_NbOfIntervals)
      m_omegaj = lomegaj;
    else
      cerr << "Pb while initializing the omega " <<endl;
  }

  void AnalyticalZMPCOGTrajectory::SetPolynomialDegrees(vector<unsigned int>
                                                        &lPolynomialDegree)
  {
    if ((int)lPolynomialDegree.size()==m_NbOfIntervals)
      m_PolynomialDegree=lPolynomialDegree;

    for(unsigned int i=0; i<m_PolynomialDegree.size(); i++)
      {

        ODEBUG("i:" << i << " " << m_PolynomialDegree[i]);
        if (m_ListOfCOGPolynomials[i]!=0)
          delete m_ListOfCOGPolynomials[i];
        m_ListOfCOGPolynomials[i] = new Polynome(m_PolynomialDegree[i]);

        //      m_ListOfCOGPolynomials[i]->print();
        if (m_ListOfZMPPolynomials[i]!=0)
          delete m_ListOfZMPPolynomials[i];
        m_ListOfZMPPolynomials[i] = new Polynome(m_PolynomialDegree[i]);
        //      m_ListOfZMPPolynomials[i]->print();

      }
  }


  void AnalyticalZMPCOGTrajectory::
  GetPolynomialDegrees
  (vector<unsigned int>
   &lPolynomialDegree) const
  {
    lPolynomialDegree=m_PolynomialDegree;
  }

  void AnalyticalZMPCOGTrajectory::
  GetNumberOfIntervals
  (unsigned int
   &lNbOfIntervals) const
  {
    lNbOfIntervals = m_NbOfIntervals;
  }

  void AnalyticalZMPCOGTrajectory::
  GetHyperbolicCoefficients
  (vector<double> &lV,
   vector<double> &lW) const
  {
    lV = m_V;
    lW = m_W;
  }

  void AnalyticalZMPCOGTrajectory::
  GetStartingPointAndHeightVariation
  (vector<double> &lTj,
   vector<double> &lomegaj)
  {
    lTj = m_DeltaTj;
    lomegaj = m_omegaj;
  }

  bool AnalyticalZMPCOGTrajectory::
  GetFromListOfCOGPolynomials
  (unsigned int j,
   Polynome * &aPoly ) const
  {
    aPoly = 0;

    if ((int)j<m_NbOfIntervals)
      {
        aPoly = m_ListOfCOGPolynomials[j];
        return true;
      }
    return false;
  }

  bool AnalyticalZMPCOGTrajectory::
  GetFromListOfZMPPolynomials
  (unsigned int j,
   Polynome * &aPoly ) const
  {
    aPoly = 0;

    if ((int)j<m_NbOfIntervals)
      {
        aPoly = m_ListOfZMPPolynomials[j];
        return true;
      }
    return false;
  }

  void AnalyticalZMPCOGTrajectory::
  TransfertOneIntervalCoefficientsFromCOGTrajectoryToZMPOne
  (unsigned int intervalindex,
   double &lCoMZ,
   double &lZMPZ)
  {

    vector<double> CoefsFromCOG;
    vector<double> CoefsForZMP;

    if ((m_ListOfCOGPolynomials[intervalindex]!=0) &&
        (m_ListOfZMPPolynomials[intervalindex]!=0))
      {

        m_ListOfCOGPolynomials[intervalindex]->GetCoefficients(CoefsFromCOG);
        //       m_ListOfCOGPolynomials[j]->print();
        CoefsForZMP.resize(CoefsFromCOG.size());
        double sTc = fabs(9.86/(lCoMZ-lZMPZ));
        CoefsForZMP[m_PolynomialDegree[intervalindex]-1] =
          CoefsFromCOG[m_PolynomialDegree[intervalindex]-1];
        CoefsForZMP[m_PolynomialDegree[intervalindex]] =
          CoefsFromCOG[m_PolynomialDegree[intervalindex]];
        for(unsigned int i=0; i<m_PolynomialDegree[intervalindex]-1; ++i)
          CoefsForZMP[i] = CoefsFromCOG[i] - ((double)(i+1.0)*(double)(i+2.0))*
            (CoefsFromCOG[i+2]/sTc);
        m_ListOfZMPPolynomials[intervalindex]->SetCoefficients(CoefsForZMP);

      }

  }

  void AnalyticalZMPCOGTrajectory::
  TransfertCoefficientsFromCOGTrajectoryToZMPOne
  (vector<double> &lCoMZ,
   vector<double> &lZMPZ)
  {
    for(int j=0; j<m_NbOfIntervals; j++)
      {
        double lCoMZ2 = lCoMZ[j];
        double lZMPZ2 = lZMPZ[j];
        TransfertOneIntervalCoefficientsFromCOGTrajectoryToZMPOne
          (j,lCoMZ2,lZMPZ2);
      }

  }


  void AnalyticalZMPCOGTrajectory::
  Building3rdOrderPolynomial
  (unsigned int anIntervalj,
   double pjTjm1,
   double pjTj)
  {
    vector<double> CoefsForZMP, CoefsForCOG;
    CoefsForZMP.resize(4);
    CoefsForCOG.resize(4);
    double lTj;

    if (m_ListOfZMPPolynomials[anIntervalj]!=0)
      {
        // Compute the coefficients for the ZMP polynomial.
        lTj = m_DeltaTj[anIntervalj];

        CoefsForZMP[0] = pjTjm1;
        CoefsForZMP[1] = 0;
        CoefsForZMP[2] = (pjTj - pjTjm1)*3.0/(lTj * lTj);
        CoefsForZMP[3] = -2.0 * CoefsForZMP[2] /(3.0 * lTj);
        m_ListOfZMPPolynomials[anIntervalj]->SetCoefficients(CoefsForZMP);
      }

    double Omegac = m_omegaj[anIntervalj];
    if (m_ListOfCOGPolynomials[anIntervalj]!=0)
      {
        // Compute the coefficients for the COG polynomial.
        CoefsForCOG[3] = CoefsForZMP[3];
        CoefsForCOG[2] = CoefsForZMP[2];
        CoefsForCOG[1] = CoefsForZMP[1] + CoefsForCOG[3] * 6/(Omegac *Omegac);
        CoefsForCOG[0] = CoefsForZMP[0] + CoefsForCOG[2] * 2/(Omegac *Omegac);
        m_ListOfCOGPolynomials[anIntervalj]->SetCoefficients(CoefsForCOG);
      }
  }

  double AnalyticalZMPCOGTrajectory::FluctuationMaximal()
  {

    double Tmax;
    vector<double> CoefsForZMP;
    m_ListOfZMPPolynomials[0]->GetCoefficients(CoefsForZMP);

    Tmax = m_AbsoluteTimeReference  - 2* CoefsForZMP[2] * m_DeltaTj[0]/
      (2.0 * CoefsForZMP[2] + 3.0 * CoefsForZMP[3] * m_DeltaTj[0]);

    return Tmax;

  }

  bool AnalyticalZMPCOGTrajectory::GetIntervalIndexFromTime(double t,
                                                            unsigned int &j)
  {
    unsigned int prev_j = 0 ;
    return GetIntervalIndexFromTime(t, j, prev_j);
  }

  bool AnalyticalZMPCOGTrajectory::
  GetIntervalIndexFromTime
  (double t,
   unsigned int &j, unsigned int &prev_j)
  {
    ODEBUG("Here "<< m_DeltaTj.size());
    t -= m_AbsoluteTimeReference;
    double reftime=0;
    ODEBUG(" ====== CoM ====== ");
    for(unsigned int lj=prev_j; lj<m_DeltaTj.size(); lj++)
      {
        ODEBUG("t: " << t << " reftime :" << reftime << " Tj["<<j << "]= "<<
               m_DeltaTj[j]);
        if (((t+m_Sensitivity)>=reftime) &&
            (t<=reftime+m_DeltaTj[lj]+m_Sensitivity))
          {
            j = lj;
            return true;
          }
        reftime+=m_DeltaTj[lj];
      }
    return false;
  }

  ostream& operator <<(ostream &os,const AnalyticalZMPCOGTrajectory &obj)
  {

    vector<double> lV,lW;
    vector<unsigned int> lPolynomialDegrees;

    obj.GetPolynomialDegrees(lPolynomialDegrees);

    unsigned int lNbOfIntervals;
    obj.GetNumberOfIntervals(lNbOfIntervals);
    obj.GetHyperbolicCoefficients(lV,lW);


    for(unsigned int i=0; i<lNbOfIntervals; i++)
      {
        os << "==Interval== " << i << " == " <<endl;
        vector<double> CoefsForZMP, CoefsForCOG;
        Polynome *aPolynome;

        obj.GetFromListOfZMPPolynomials(i,aPolynome);
        aPolynome->GetCoefficients(CoefsForZMP);
        obj.GetFromListOfCOGPolynomials(i,aPolynome);
        aPolynome->GetCoefficients(CoefsForCOG);
        os << " COG: " << endl;
        for(unsigned int j=0; j<=lPolynomialDegrees[i]; j++)
          {
            os << CoefsForCOG[j] << " ";
          }
        os << "V: " << lV[i] << " W: " << lW[i];
        os << endl;
        os << " ZMP: " << endl;
        for(unsigned int j=0; j<=lPolynomialDegrees[i]; j++)
          {
            os << CoefsForZMP[j] << " ";
          }
        os << endl;

      }
    return os;
  }

}

