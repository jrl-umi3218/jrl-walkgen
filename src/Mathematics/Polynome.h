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

   Please see License.txt for further information on license.   
*/

#ifndef _POLYNOME_H_
#define _POLYNOME_H_

#include <vector>


using namespace::std;

namespace PatternGeneratorJRL
{

  /** Class for computing trajectories */
  class  Polynome
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
