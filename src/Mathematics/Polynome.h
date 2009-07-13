/** \file Polynome.h
    \brief Polynomes object for trajectories. 
    Initial polynome.

    CVS Information:
    $Id: Polynome.h,v 1.2 2006-01-18 06:34:58 stasse Exp $
    $Author: stasse $
    $Date: 2006-01-18 06:34:58 $
    $Revision: 1.2 $
    $Source: /home/CVSREPOSITORY/PatternGeneratorJRL/src/Polynome.h,v $
    $Log: Polynome.h,v $
    Revision 1.2  2006-01-18 06:34:58  stasse
    OS: Updated the names of the contributors, the documentation
    and added a sample file for WalkPlugin


   Copyright (c) 2005-2006, 
   @author Olivier Stasse, Ramzi Sellouati, Francois Keith,
   
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

#ifndef _POLYNOME_H_
#define _POLYNOME_H_

#include <vector>
#include <walkGenJrl/walkGenJrl_API.h>

using namespace::std;

namespace PatternGeneratorJRL
{

  /** Class for computing trajectories */
  class WALK_GEN_JRL_EXPORT Polynome
    {

    public:
      /*! Constructor */
      Polynome(int Degree);

      /*! Destructor */
      ~Polynome();

      /*! Compute the value. */
      double Compute(double t);

      /*! Compute the value of the derivative. */
      double ComputeDerivative(double t);
      
      /*! Get the coefficients. */
      void GetCoefficients(std::vector<double> &lCoefficients) const;

      /*! Set the coefficients. */
      void SetCoefficients(std::vector<double> &lCoefficients);

      /*! Print the coefficient. */
      void print();

    protected:

      /// Degree of the polynome
      int m_Degree;

      /// Vector of coefficients.
      std::vector<double> m_Coefficients;
    };
};
#endif /* _POLYNOME_H_*/
