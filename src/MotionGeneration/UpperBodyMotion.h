/** @doc This object generate all the values for the foot trajectories,
   and the desired ZMP based on a sequence of relative steps.
   If you want to change the reference trajectories, and the planning
   of the foot, thOn pageis is the object to modify.

   Copyright (c) 2005-2009, 
   @author Bjorn Verrelst,
   Olivier Stasse
   Francois Keith, 

   JRL-Japan, CNRS/AIST

   All rights reserved.

   Please see License.txt for further information on license.   
*/

#ifndef _UPPER_BODY_MOTION_
#define _UPPER_BODY_MOTION_

#include <MatrixAbstractLayer/MatrixAbstractLayer.h>

#include <vector>
#include <string>
//#define FULL_POLYNOME

using namespace::std;


#include <Mathematics/PolynomeFoot.h>



namespace PatternGeneratorJRL
{
   
  class  UpperBodyMotion
    {
      public :
    
      /// Constructor
      UpperBodyMotion();

      /// Destructor
      ~UpperBodyMotion();
      
      void GenerateDataFile(string aFileName, int LenghtDataArray);

      void ReadDataFile(string aFileName, MAL_MATRIX(&UpperBodyAngles,double));
      
      void WriteDataFile(string aFileName, MAL_MATRIX(&UpperBodyAngles,double));
	
    
   protected:


     
   };
};
#endif /* _UPPER_BODY_MOTION_*/
