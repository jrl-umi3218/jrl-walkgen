/** @doc Polynomes object for trajectories.
   All references are from Kajita san's book

   $Id: PolynomeFoot.h,v 1.2 2006-01-18 06:34:58 stasse Exp $
   $Author: stasse $
   $Date: 2006-01-18 06:34:58 $
   $Revision: 1.2 $
   $Source: /home/CVSREPOSITORY/PatternGeneratorJRL/src/PolynomeFoot.h,v $
   $Log: PolynomeFoot.h,v $
   Revision 1.2  2006-01-18 06:34:58  stasse
   OS: Updated the names of the contributors, the documentation
   and added a sample file for WalkPlugin


   Copyright (c) 2005-2006, 
   @author Olivier Stasse, Ramzi Sellouati
   
   JRL-Japan, CNRS/AIST

   All rights reserved.
   
   Redistribution and use in source and binary forms, with or without modification, 
   are permitted provided that the following conditions are met:
   
   * Redistributions of source code must retain the above copyright notice, 
   this list of conditions and the following disclaimer.
   * Redistributions in binary form must reproduce the above copyright notice, 
   this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.
   * Neither the name of the <ORGANIZATION> nor the names of its contributors 
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


#ifndef _POLYNOME_FOOT_H_
#define _POLYNOME_FOOT_H_

#ifdef max
#undef max
#undef min
#endif

#include <vector>
#include <Polynome.h>
#ifdef _VNL_MATRIX_
#include <VNL/matrix.h>
#include <VNL/vector.h>
#endif

namespace PatternGeneratorJRL
{
  /// Polynome used for X,Y and Theta trajectories.
  class Polynome3 : public Polynome
    {
    public:
      /** Constructor:
       FT: Final time
       FP: Final position */
      Polynome3(double FT, double FP);

      /// Set the parameters
      void SetParameters(double FT, double FP);
      /// Destructor.
      ~Polynome3();
    };
  /// Polynome used for Z trajectory.
  class Polynome4 : public Polynome
    {
    public:
      /** Constructor:
       FT: Final time
       MP: Middle position */
      Polynome4(double FT, double MP);
  
      /// Set the parameters
      void SetParameters(double FT, double MP);

      /// Destructor.
      ~Polynome4();
    };
  /// Polynome used for X,Y and Theta trajectories.
  class Polynome5 : public Polynome
    {
    public:
      /** Constructor:
	  FT: Final time
	  FP: Final position */
      Polynome5(double FT, double FP);

      /// Set the parameters
      void SetParameters(double FT, double FP);
      /// Destructor.
      ~Polynome5();
    };

  /// Polynome used for Z trajectory.
  class Polynome6 : public Polynome
    {
    public:
      /// Constructor:
      /// FT: Final time
      /// MP: Middle position
      Polynome6(double FT, double MP);
  
      /// Set the parameters
      void SetParameters(double FT, double MP);

      /// Destructor.
      ~Polynome6();
    };

};
#endif /* _POLYNOME_FOOT_H_ */
