/*! \file AnalyticalZMPCOGTrajectory.h
  \brief Polynomial trajectory for the ZMP and the CoG

   Copyright (c) 2007
   @author Olivier Stasse
   
   JRL-Japan, CNRS/AIST

   All rights reserved.
   
   Redistribution and use in source and binary forms, with or without modification, 
   are permitted provided that the following conditions are met:
   
   * Redistributions of source code must retain the above copyright notice, 
   this list of conditions and the following disclaimer.
   * Redistributions in binary form must reproduce the above copyright notice, 
   this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.
   * Neither the name of the CNRS and AIST nor the names of its contributors 
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

#ifndef _ANALYTICAL_COG_H_
#define _ANALYTICAL_COG_H_

#include <vector>
#include <iostream>

#include <walkGenJrl/Mathematics/Polynome.h>


namespace PatternGeneratorJRL
{

  /*! AnalyticalZMPCOGTrajectory represents the ZMP and the COG trajectories
      based on the following formula:
      
   */
  class AnalyticalZMPCOGTrajectory
    {

    public:
      
      /*! Constructor */
      AnalyticalZMPCOGTrajectory(int lNbOfIntervals=0);

      /*! Destructor */
      ~AnalyticalZMPCOGTrajectory();

      /*! Compute the current value according
	to time.
	@param t: the time,
	@param r: the result,
	@return Returns true if the function has been
	computed, false otherwise.
      */
      bool ComputeCOM(double t,double &r);

      /*! Compute the current CoM speed value according
	to time.
	@param t: the time,
	@param r: the result,
	@return Returns true if the function has been
	computed, false otherwise.
      */
      bool ComputeCOMSpeed(double t,double &r);

      /*! Compute the current value according
	to time and the index of the interval.
	To be efficient this method does not have
	any boundary check.
	@param t: the time,
	@param i: the numero of the interval
	@param r: the result,
	@return Returns true if the function has been
	computed, false otherwise.
      */
      bool ComputeCOM(double t,double &r, int i);


      /*! Compute the current value according
	to time.
	@param t: the time,
	@param r: the result,
	@return Returns true if the function has been
	computed, false otherwise.
      */
      bool ComputeZMP(double t,double &r);

      /*! Compute the current ZMP speed value according
	to time.
	@param t: the time,
	@param r: the result,
	@return Returns true if the function has been
	computed, false otherwise.
      */
      bool ComputeZMPSpeed(double t,double &r);

      /*! Compute the current value according
	to time.
	@param t: the time,
	@param i: the numero of the interval
	@param r: the result,
	@return Returns true if the function has been
	computed, false otherwise.
      */
      bool ComputeZMP(double t,double &r, int i);
      
      /*! \name Setter and Getter@{ */
      
      /*! \brief Set the number of Intervals for this
	trajectory. */
      void SetNumberOfIntervals(unsigned int lNbOfIntervals);
      
      
      /*! \brief Set the coefficients for the sinuse and cosinues
	function. */
      void SetCoGHyperbolicCoefficients(vector<double> &lV,
					vector<double> &lW);
      
      /*! \brief Set the starting point and the height variation. */
      void SetStartingTimeIntervalsAndHeightVariation(vector<double> &lDeltaTj,
						      vector<double> &lomegaj);
      
      /*! \brief Set the degree of each polynomials for the CoG
	Remark: the size of the vector of degrees should match the number
	of intervals.
       */
      void SetPolynomialDegrees(vector<unsigned int> &lPolynomialDegree);

      /*! \brief Get the degree of each polynomials for the CoG.
       */
      void GetPolynomialDegrees(vector<unsigned int> &lPolynomialDegree) const;

      /*! \brief Set the number of Intervals for this
	trajectory. */
      void GetNumberOfIntervals(unsigned int &lNbOfIntervals) const;
            
      /*! \brief Get the coefficients for the sinuse and cosinues
	function. */
      void GetHyperbolicCoefficients(vector<double> &lV,
				      vector<double> &lW) const;

      /*! \brief Get the starting point and the height variation. */
      void GetStartingPointAndHeightVariation(vector<double> &lTj,
					      vector<double> &lomegaj);

      /*! \brief Get the polynomial at interval j for the CoG 
	Remark: The call to this function assume that the 
	method SetPolynomialDegree has been call beforehand. 
	
       */
      bool GetFromListOfCOGPolynomials(unsigned int j, Polynome * & aPoly ) const;

      /*! Get the polynomial at interval j for the ZMP 
	Remark: The call to this function assume that the 
	method SetPolynomialDegree has been call beforehand. 	
       */
      bool GetFromListOfZMPPolynomials(unsigned int j, Polynome * & aPoly ) const;

      /*! \brief Transfert the coefficients from the COG trajectory to the ZMP for all intervals. 
	@param lCOMZ: Profile of the height CoM for each interval.
	@param lZMPZ: Profile of the height ZMP for each interval.
       */
      void TransfertCoefficientsFromCOGTrajectoryToZMPOne(vector<double> &lCOMZ,
							  vector<double> &lZMPZ);

      /*! \brief Transfert the coefficients from the COG trajectory to the ZMP. 
	@param IntervalIndex: Number of the interval.
	@param lCOMZ: Value of the CoM height for this interval.
	@param lZMPZ: Value of the ZMP height for this interval.
       */
      void TransfertOneIntervalCoefficientsFromCOGTrajectoryToZMPOne(unsigned int IntervalIndex,
								     double &lCOMZ,
								     double &lZMPZ);


      /*! Build the coefficients of a third order COG polynomial according
	to  the OmegaC and the value of the ZMP at the beginning and the end
	of the interval, and assuming that the speed is set to zero.  */
      void Building3rdOrderPolynomial(unsigned int anIntervalj,
				      double pjTjm1,
				      double pjTj);

      /*@} */
      
      /*! \brief Returns the maximal fluctuation for the first segment of this trajectory. */
      double FluctuationMaximal();

      friend std::ostream& operator <<(std::ostream &os,const AnalyticalZMPCOGTrajectory &obj);

      /*! \brief Absolute Time reference of this trajectory. */
      double  GetAbsoluteTimeReference() const
	{ return m_AbsoluteTimeReference; }

      /*! \brief Set Absolute time reference of this trajectory */
      void SetAbsoluteTimeReference(double anAbsoluteTimeReference)
      { m_AbsoluteTimeReference = anAbsoluteTimeReference; }

      /*! \brief Get the index of the interval according to the time. */
      bool GetIntervalIndexFromTime(double t, unsigned int &j);
      
      /*! \brief Get the index of the interval according to the time,
	and the previous value of the interval. */
      bool GetIntervalIndexFromTime(double t, unsigned int &j, unsigned int &prev_j);
      
    protected:
      
      /*! Number of intervals */
      int m_NbOfIntervals;

      /*! List of coefficients for the hyperbolics
	cosine function*/
      vector<double> m_V;
      
      /*! List of coefficients for the hyperbolics
	sine function*/
      vector<double> m_W;

      /*! List of temporal starting point. */
      vector<double> m_DeltaTj;

      /*! List of omega, i.e. height variation along
	the trajectory. */
      vector<double> m_omegaj;
      
      /*! List of reference time for the interval. */
      vector<double> m_RefTime;

      /*! List of polynomial degrees for the CoM*/
      vector<unsigned int> m_PolynomialDegree;
      
      /*! List of polynomials for the COG */
      vector<Polynome *> m_ListOfCOGPolynomials;

      /*! List of polynomials for the ZMP */
      vector<Polynome *> m_ListOfZMPPolynomials;

      /*! Intern method to free the polynomials */
      void FreePolynomes();

      /*! Store the absolute time reference */
      double m_AbsoluteTimeReference;
    };

  std::ostream& operator <<(std::ostream &os,const AnalyticalZMPCOGTrajectory &obj);
};
#endif /* _ANALYTICAL_COG_H_ */
